#pragma once

#include <lib/cdev/CDev.hpp>

class SyslinkBridge : public cdev::CDev
{
public:
	SyslinkBridge(Syslink *link);
	virtual ~SyslinkBridge() = default;

	virtual int	init();

	virtual ssize_t	read(struct file *filp, char *buffer, size_t buflen);
	virtual ssize_t	write(struct file *filp, const char *buffer, size_t buflen);
	virtual int	ioctl(struct file *filp, int cmd, unsigned long arg);

	// Makes the message available for reading to processes reading from the bridge
	void pipe_message(crtp_message_t *msg);

protected:

	virtual pollevent_t poll_state(struct file *filp);

private:

	Syslink *_link;

	// Stores data that was received from syslink but not yet read by another driver
	ringbuffer::RingBuffer _readbuffer;

	crtp_message_t _msg_to_send;
	int _msg_to_send_size_remaining;

};
