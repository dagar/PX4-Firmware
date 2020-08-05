/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "uORBDeviceNode.hpp"

#include "uORBUtils.hpp"
#include "uORBManager.hpp"

#include "SubscriptionCallback.hpp"

#ifdef ORB_COMMUNICATOR
#include "uORBCommunicator.hpp"
#endif /* ORB_COMMUNICATOR */

#ifdef __PX4_NUTTX
static int orb_open(cdev::file_t *filp) { return static_cast<uORB::DeviceNode *>(filp->f_inode->i_private)->open(filp); }
static int orb_close(cdev::file_t *filp) { return static_cast<uORB::DeviceNode *>(filp->f_inode->i_private)->close(filp); }
static ssize_t orb_read(cdev::file_t *filp, char *buffer, size_t buflen) { return static_cast<uORB::DeviceNode *>(filp->f_inode->i_private)->read(filp, buffer, buflen); }
static ssize_t orb_write(cdev::file_t *filp, const char *buffer, size_t buflen) { return static_cast<uORB::DeviceNode *>(filp->f_inode->i_private)->write(filp, buffer, buflen); }
static int orb_ioctl(cdev::file_t *filp, int cmd, unsigned long arg) { return static_cast<uORB::DeviceNode *>(filp->f_inode->i_private)->ioctl(filp, cmd, arg); }
static int orb_poll(cdev::file_t *filp, px4_pollfd_struct_t *fds, bool setup) { return static_cast<uORB::DeviceNode *>(filp->f_inode->i_private)->poll(filp, fds, setup); }

const struct file_operations fops = {
open	: orb_open,
close	: orb_close,
read	: orb_read,
write	: orb_write,
seek	: nullptr,
ioctl	: orb_ioctl,
poll	: orb_poll,
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
unlink	: nullptr
#endif
};

#endif // PX4_NUTTX


uORB::DeviceNode::DeviceNode(const struct orb_metadata *meta, const uint8_t instance, const char *path,
			     ORB_PRIO priority, uint8_t queue_size) :
	_devname(path),
	_meta(meta),
	_priority(priority),
	_instance(instance),
	_queue_size(queue_size)
{
	px4_sem_init(&_lock, 0, 1);
}

uORB::DeviceNode::~DeviceNode()
{
	unregister_driver(_devname);

	delete[] _data;
	delete[] _pollset;

	px4_sem_destroy(&_lock);
}

int uORB::DeviceNode::init()
{
	// uORB::Utils::node_mkpath(nodepath, meta, instance);

	int ret = register_driver(_devname, &fops, 0666, (void *)this);

	return ret;
}

int uORB::DeviceNode::open(cdev::file_t *filp)
{
	/* is this a publisher? */
	if (filp->f_oflags == PX4_F_WRONLY) {

		lock();
		mark_as_advertised();
		unlock();

		/* now complete the open */
		return PX4_OK;
	}

	/* is this a new subscriber? */
	if (filp->f_oflags == PX4_F_RDONLY) {

		/* allocate subscriber data */
		SubscriberData *sd = new SubscriberData{};

		if (nullptr == sd) {
			return -ENOMEM;
		}

		/* If there were any previous publications, allow the subscriber to read them */
		const unsigned gen = published_message_count();
		sd->generation = gen - (_queue_size < gen ? _queue_size : gen);

		filp->f_priv = (void *)sd;

		add_internal_subscriber();

		return PX4_OK;
	}

	if (filp->f_oflags == 0) {
		return PX4_OK;
	}

	/* can only be pub or sub, not both */
	return -EINVAL;
}

int uORB::DeviceNode::close(cdev::file_t *filp)
{
	if (filp->f_oflags == PX4_F_RDONLY) { /* subscriber */
		SubscriberData *sd = static_cast<SubscriberData *>(filp->f_priv);

		if (sd != nullptr) {
			remove_internal_subscriber();

			delete sd;
			sd = nullptr;
		}
	}

	return PX4_OK;
}

bool
uORB::DeviceNode::copy_locked(void *dst, unsigned &generation) const
{
	bool updated = false;

	if ((dst != nullptr) && (_data != nullptr)) {
		const unsigned current_generation = _generation.load();

		if (current_generation > generation + _queue_size) {
			// Reader is too far behind: some messages are lost
			generation = current_generation - _queue_size;
		}

		if ((current_generation == generation) && (generation > 0)) {
			/* The subscriber already read the latest message, but nothing new was published yet.
			 * Return the previous message
			 */
			--generation;
		}

		memcpy(dst, _data + (_meta->o_size * (generation % _queue_size)), _meta->o_size);

		if (generation < current_generation) {
			++generation;
		}

		updated = true;
	}

	return updated;
}

bool
uORB::DeviceNode::copy(void *dst, unsigned &generation)
{
	ATOMIC_ENTER;

	bool updated = copy_locked(dst, generation);

	ATOMIC_LEAVE;

	return updated;
}

ssize_t uORB::DeviceNode::read(cdev::file_t *filp, char *buffer, size_t buflen)
{
	/* if the object has not been written yet, return zero */
	if (_data == nullptr) {
		return 0;
	}

	/* if the caller's buffer is the wrong size, that's an error */
	if (buflen != _meta->o_size) {
		return -EIO;
	}

	SubscriberData *sd = static_cast<SubscriberData *>(filp->f_priv);

	/*
	 * Perform an atomic copy & state update
	 */
	ATOMIC_ENTER;

	// if subscriber has an interval track the last update time
	if (sd->update_interval) {
		sd->update_interval->last_update = hrt_absolute_time();
	}

	copy_locked(buffer, sd->generation);

	ATOMIC_LEAVE;

	return _meta->o_size;
}

ssize_t uORB::DeviceNode::write(cdev::file_t *filp, const char *buffer, size_t buflen)
{
	/*
	 * Writes are legal from interrupt context as long as the
	 * object has already been initialised from thread context.
	 *
	 * Writes outside interrupt context will allocate the object
	 * if it has not yet been allocated.
	 *
	 * Note that filp will usually be NULL.
	 */
	if (nullptr == _data) {

#ifdef __PX4_NUTTX

		if (!up_interrupt_context()) {
#endif /* __PX4_NUTTX */

			lock();

			/* re-check size */
			if (nullptr == _data) {
				_data = new uint8_t[_meta->o_size * _queue_size];
			}

			unlock();

#ifdef __PX4_NUTTX
		}

#endif /* __PX4_NUTTX */

		/* failed or could not allocate */
		if (nullptr == _data) {
			return -ENOMEM;
		}
	}

	/* If write size does not match, that is an error */
	if (_meta->o_size != buflen) {
		return -EIO;
	}

	/* Perform an atomic copy. */
	ATOMIC_ENTER;
	/* wrap-around happens after ~49 days, assuming a publisher rate of 1 kHz */
	unsigned generation = _generation.fetch_add(1);

	memcpy(_data + (_meta->o_size * (generation % _queue_size)), buffer, _meta->o_size);

	// callbacks
	for (auto item : _callbacks) {
		item->call();
	}

	for (unsigned i = 0; i < _max_pollwaiters; i++) {
		if (nullptr != _pollset[i]) {
			// If the topic looks updated to the subscriber, go ahead and notify them.
			if (appears_updated((cdev::file_t *)_pollset[i]->priv)) {
				_pollset[i]->revents |= _pollset[i]->events & POLLIN;
				px4_sem_post(_pollset[i]->sem);
			}
		}
	}

	ATOMIC_LEAVE;

	return _meta->o_size;
}

int uORB::DeviceNode::store_poll_waiter(px4_pollfd_struct_t *fds)
{
	// Look for a free slot.
	for (unsigned i = 0; i < _max_pollwaiters; i++) {
		if (nullptr == _pollset[i]) {

			/* save the pollfd */
			_pollset[i] = fds;

			return PX4_OK;
		}
	}

	return -ENFILE;
}

int uORB::DeviceNode::poll(cdev::file_t *filep, px4_pollfd_struct_t *fds, bool setup)
{
	int ret = PX4_OK;

	if (setup) {
		/*
		 * Save the file pointer in the pollfd for the subclass'
		 * benefit.
		 */
		fds->priv = (void *)filep;

		/*
		 * Lock against poll_notify() and possibly other callers (protect _pollset).
		 */
		ATOMIC_ENTER;

		/*
		 * Try to store the fds for later use and handle array resizing.
		 */
		while ((ret = store_poll_waiter(fds)) == -ENFILE) {

			// No free slot found. Resize the pollset. This is expensive, but it's only needed initially.

			if (_max_pollwaiters >= 256 / 2) { //_max_pollwaiters is uint8_t
				ret = -ENOMEM;
				break;
			}

			const uint8_t new_count = _max_pollwaiters > 0 ? _max_pollwaiters * 2 : 1;
			px4_pollfd_struct_t **prev_pollset = _pollset;

#ifdef __PX4_NUTTX
			// malloc uses a semaphore, we need to call it enabled IRQ's
			px4_leave_critical_section(flags);
#endif
			px4_pollfd_struct_t **new_pollset = new px4_pollfd_struct_t *[new_count];

#ifdef __PX4_NUTTX
			flags = px4_enter_critical_section();
#endif

			if (prev_pollset == _pollset) {
				// no one else updated the _pollset meanwhile, so we're good to go
				if (!new_pollset) {
					ret = -ENOMEM;
					break;
				}

				if (_max_pollwaiters > 0) {
					memset(new_pollset + _max_pollwaiters, 0, sizeof(px4_pollfd_struct_t *) * (new_count - _max_pollwaiters));
					memcpy(new_pollset, _pollset, sizeof(px4_pollfd_struct_t *) * _max_pollwaiters);
				}

				_pollset = new_pollset;
				_pollset[_max_pollwaiters] = fds;
				_max_pollwaiters = new_count;

				// free the previous _pollset (we need to unlock here which is fine because we don't access _pollset anymore)
#ifdef __PX4_NUTTX
				px4_leave_critical_section(flags);
#endif

				if (prev_pollset) {
					delete[](prev_pollset);
				}

#ifdef __PX4_NUTTX
				flags = px4_enter_critical_section();
#endif

				// Success
				ret = PX4_OK;
				break;
			}

#ifdef __PX4_NUTTX
			px4_leave_critical_section(flags);
#endif
			// We have to retry
			delete[] new_pollset;
#ifdef __PX4_NUTTX
			flags = px4_enter_critical_section();
#endif
		}

		if (ret == PX4_OK) {
			/*
			 * Check to see whether we should send a poll notification
			 * immediately.
			 */
			if (appears_updated(filep)) {
				fds->revents |= fds->events & POLLIN;
			}

			/* yes? post the notification */
			if (fds->revents != 0) {
				px4_sem_post(fds->sem);
			}
		}

		ATOMIC_LEAVE;

	} else {
		ATOMIC_ENTER;

		/*
		 * Handle a teardown request.
		 */
		for (unsigned i = 0; i < _max_pollwaiters; i++) {
			if (fds == _pollset[i]) {
				_pollset[i] = nullptr;
				break;
			}
		}

		ATOMIC_LEAVE;
	}

	return ret;
}

int uORB::DeviceNode::ioctl(cdev::file_t *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case ORBIOCUPDATED: {
			ATOMIC_ENTER;
			*(bool *)arg = appears_updated(filp);
			ATOMIC_LEAVE;
			return PX4_OK;
		}

	case ORBIOCSETINTERVAL: {
			int ret = PX4_OK;
			lock();

			SubscriberData *sd = static_cast<SubscriberData *>(filp->f_priv);

			if (arg == 0) {
				if (sd->update_interval) {
					delete (sd->update_interval);
					sd->update_interval = nullptr;
				}

			} else {
				if (sd->update_interval) {
					sd->update_interval->interval = arg;

				} else {
					sd->update_interval = new UpdateIntervalData();

					if (sd->update_interval) {
						sd->update_interval->interval = arg;

					} else {
						ret = -ENOMEM;
					}
				}
			}

			unlock();
			return ret;
		}

	case ORBIOCGADVERTISER:
		*(uintptr_t *)arg = (uintptr_t)this;
		return PX4_OK;

	case ORBIOCGPRIORITY:
		*(int *)arg = get_priority();
		return PX4_OK;

	case ORBIOCSETQUEUESIZE: {
			lock();
			int ret = update_queue_size(arg);
			unlock();
			return ret;
		}

	case ORBIOCGETINTERVAL: {
			SubscriberData *sd = static_cast<SubscriberData *>(filp->f_priv);

			if (sd->update_interval) {
				*(unsigned *)arg = sd->update_interval->interval;

			} else {
				*(unsigned *)arg = 0;
			}
		}

		return OK;

	case ORBIOCISADVERTISED:
		*(unsigned long *)arg = _advertised;

		return OK;

	default:
		return -ENOTTY;
	}
}

ssize_t
uORB::DeviceNode::publish(const orb_metadata *meta, orb_advert_t handle, const void *data)
{
	uORB::DeviceNode *devnode = (uORB::DeviceNode *)handle;
	int ret;

	/* check if the device handle is initialized and data is valid */
	if ((devnode == nullptr) || (meta == nullptr) || (data == nullptr)) {
		errno = EFAULT;
		return PX4_ERROR;
	}

	/* check if the orb meta data matches the publication */
	if (devnode->_meta != meta) {
		errno = EINVAL;
		return PX4_ERROR;
	}

	/* call the devnode write method with no file pointer */
	ret = devnode->write(nullptr, (const char *)data, meta->o_size);

	if (ret < 0) {
		errno = -ret;
		return PX4_ERROR;
	}

	if (ret != (int)meta->o_size) {
		errno = EIO;
		return PX4_ERROR;
	}

#ifdef ORB_COMMUNICATOR
	/*
	 * if the write is successful, send the data over the Multi-ORB link
	 */
	uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();

	if (ch != nullptr) {
		if (ch->send_message(meta->o_name, meta->o_size, (uint8_t *)data) != 0) {
			PX4_ERR("Error Sending [%s] topic data over comm_channel", meta->o_name);
			return PX4_ERROR;
		}
	}

#endif /* ORB_COMMUNICATOR */

	return PX4_OK;
}

int uORB::DeviceNode::unadvertise(orb_advert_t handle)
{
	if (handle == nullptr) {
		return -EINVAL;
	}

	uORB::DeviceNode *devnode = (uORB::DeviceNode *)handle;

	/*
	 * We are cheating a bit here. First, with the current implementation, we can only
	 * have multiple publishers for instance 0. In this case the caller will have
	 * instance=nullptr and _published has no effect at all. Thus no unadvertise is
	 * necessary.
	 * In case of multiple instances, we have at most 1 publisher per instance and
	 * we can signal an instance as 'free' by setting _published to false.
	 * We never really free the DeviceNode, for this we would need reference counting
	 * of subscribers and publishers. But we also do not have a leak since future
	 * publishers reuse the same DeviceNode object.
	 */
	devnode->_advertised = false;

	return PX4_OK;
}

#ifdef ORB_COMMUNICATOR
int16_t uORB::DeviceNode::topic_advertised(const orb_metadata *meta, ORB_PRIO priority)
{
	uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();

	if (ch != nullptr && meta != nullptr) {
		return ch->topic_advertised(meta->o_name);
	}

	return -1;
}

/*
//TODO: Check if we need this since we only unadvertise when things all shutdown and it doesn't actually remove the device
int16_t uORB::DeviceNode::topic_unadvertised(const orb_metadata *meta, ORB_PRIO priority)
{
	uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();
	if (ch != nullptr && meta != nullptr) {
		return ch->topic_unadvertised(meta->o_name);
	}
	return -1;
}
*/
#endif /* ORB_COMMUNICATOR */

bool
uORB::DeviceNode::appears_updated(cdev::file_t *filp)
{
	// check if this topic has been published yet, if not bail out
	if (_data == nullptr) {
		return false;
	}

	SubscriberData *sd = static_cast<SubscriberData *>(filp->f_priv);

	// if subscriber has interval check time since last update
	if (sd->update_interval != nullptr) {
		if (hrt_elapsed_time(&sd->update_interval->last_update) < sd->update_interval->interval) {
			return false;
		}
	}

	// finally, compare the generation
	return (sd->generation != published_message_count());
}

bool
uORB::DeviceNode::print_statistics(int max_topic_length)
{
	if (!_advertised) {
		return false;
	}

	lock();

	const uint8_t instance = get_instance();
	const uint8_t priority = get_priority();
	const int8_t sub_count = subscriber_count();
	const uint8_t queue_size = get_queue_size();

	unlock();

	PX4_INFO_RAW("%-*s %2i %4i %2i %4i %4i %s\n", max_topic_length, get_meta()->o_name, (int)instance, (int)sub_count,
		     queue_size, get_meta()->o_size, priority, get_devname());

	return true;
}

void uORB::DeviceNode::add_internal_subscriber()
{
	lock();
	_subscriber_count++;

#ifdef ORB_COMMUNICATOR
	uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();

	if (ch != nullptr && _subscriber_count > 0) {
		unlock(); //make sure we cannot deadlock if add_subscription calls back into DeviceNode
		ch->add_subscription(_meta->o_name, 1);

	} else
#endif /* ORB_COMMUNICATOR */

	{
		unlock();
	}
}

void uORB::DeviceNode::remove_internal_subscriber()
{
	lock();
	_subscriber_count--;

#ifdef ORB_COMMUNICATOR
	uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();

	if (ch != nullptr && _subscriber_count == 0) {
		unlock(); //make sure we cannot deadlock if remove_subscription calls back into DeviceNode
		ch->remove_subscription(_meta->o_name);

	} else
#endif /* ORB_COMMUNICATOR */
	{
		unlock();
	}
}

#ifdef ORB_COMMUNICATOR
int16_t uORB::DeviceNode::process_add_subscription(int32_t rateInHz)
{
	// if there is already data in the node, send this out to
	// the remote entity.
	// send the data to the remote entity.
	uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();

	if (_data != nullptr && ch != nullptr) { // _data will not be null if there is a publisher.
		ch->send_message(_meta->o_name, _meta->o_size, _data);
	}

	return PX4_OK;
}

int16_t uORB::DeviceNode::process_remove_subscription()
{
	return PX4_OK;
}

int16_t uORB::DeviceNode::process_received_message(int32_t length, uint8_t *data)
{
	int16_t ret = -1;

	if (length != (int32_t)(_meta->o_size)) {
		PX4_ERR("Received '%s' with DataLength[%d] != ExpectedLen[%d]", _meta->o_name, (int)length, (int)_meta->o_size);
		return PX4_ERROR;
	}

	/* call the devnode write method with no file pointer */
	ret = write(nullptr, (const char *)data, _meta->o_size);

	if (ret < 0) {
		return PX4_ERROR;
	}

	if (ret != (int)_meta->o_size) {
		errno = EIO;
		return PX4_ERROR;
	}

	return PX4_OK;
}
#endif /* ORB_COMMUNICATOR */

int uORB::DeviceNode::update_queue_size(unsigned int queue_size)
{
	if (_queue_size == queue_size) {
		return PX4_OK;
	}

	//queue size is limited to 255 for the single reason that we use uint8 to store it
	if (_data || _queue_size > queue_size || queue_size > 255) {
		return PX4_ERROR;
	}

	_queue_size = queue_size;
	return PX4_OK;
}

bool
uORB::DeviceNode::register_callback(uORB::SubscriptionCallback *callback_sub)
{
	if (callback_sub != nullptr) {
		ATOMIC_ENTER;

		// prevent duplicate registrations
		for (auto existing_callbacks : _callbacks) {
			if (callback_sub == existing_callbacks) {
				ATOMIC_LEAVE;
				return true;
			}
		}

		_callbacks.add(callback_sub);
		ATOMIC_LEAVE;
		return true;
	}

	return false;
}

void
uORB::DeviceNode::unregister_callback(uORB::SubscriptionCallback *callback_sub)
{
	ATOMIC_ENTER;
	_callbacks.remove(callback_sub);
	ATOMIC_LEAVE;
}
