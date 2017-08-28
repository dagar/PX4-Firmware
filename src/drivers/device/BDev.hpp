
#pragma once

#include "device.h"

namespace device {

/**
 * Abstract class for any block device
 */
class __EXPORT BDev : public Device
{
public:
	/**
	 * Constructor
	 *
	 * @param name		Driver name
	 * @param devname	Device node name
	 */
	BDev(const char *name, const char *devname);

	/**
	 * Destructor
	 */
	virtual ~BDev();

	virtual int	init();

	/**
	 * Handle an open of the device.
	 *
	 * This function is called for every open of the device. The default
	 * implementation maintains _open_count and always returns OK.
	 *
	 * @param inode		Pointer to the NuttX inode structure.
	 * @return		OK if the open is allowed, -errno otherwise.
	 */
	virtual int	open(struct inode *inode);

	/**
	 * Handle a close of the device.
	 *
	 * This function is called for every close of the device. The default
	 * implementation maintains _open_count and returns OK as long as it is not zero.
	 *
	 * @param inode		Pointer to the NuttX inode structure.
	 * @return		OK if the close was successful, -errno otherwise.
	 */
	virtual int	close(struct inode *inode);

	/**
	 * Perform a read from the device.
	 *
	 * The default implementation returns -ENOSYS.
	 *
	 * @param inode		Pointer to the NuttX inode structure.
	 * @param buffer	Pointer to the buffer into which data should be placed.
	 * @param start_sector	The first sector to be read.
	 * @param nsectors	The number of sectors to be read.
	 * @return		The number of sectors read or -errno otherwise.
	 */
	virtual ssize_t	read(struct inode *inode, unsigned char *buffer, size_t start_sector, size_t nsectors);

	/**
	 * Perform a write to the device.
	 *
	 * The default implementation returns -ENOSYS.
	 *
	 * @param inode		Pointer to the NuttX inode structure.
	 * @param buffer	Pointer to the buffer from which data should be read.
	 * @param start_sector	The first sector to be written.
	 * @param nsectors	The number of sectors to be written.
	 * @return		The number of sectors written or -errno otherwise.
	 */
	virtual ssize_t	write(struct inode *inode, const unsigned char *buffer, size_t start_sector, size_t nsectors);

	/**
	 * Query the block device geometry.
	 *
	 * The default implementation returns -ENOSYS.
	 *
	 * @param inode		Pointer to the NuttX inode structure.
	 * @param geometry	The geometry structure to be populated.
	 * @return		OK if the geometry argument is non-NULL, or -errno otherwise.
	 */
	virtual int	geometry(struct inode *inode, struct geometry *geometry);

	/**
	 * Perform an ioctl operation on the device.
	 *
	 * The default implementation handles DIOC_GETPRIV, and otherwise
	 * returns -ENOTTY. Subclasses should call the default implementation
	 * for any command they do not handle themselves.
	 *
	 * @param inode		Pointer to the NuttX inode structure.
	 * @param cmd		The ioctl command value.
	 * @param arg		The ioctl argument value.
	 * @return		OK on success, or -errno otherwise.
	 */
	virtual int	ioctl(struct inode *inode, int cmd, unsigned long arg);

	/**
	 * Test whether the device is currently open.
	 *
	 * This can be used to avoid tearing down a device that is still active.
	 * Note - not virtual, cannot be overridden by a subclass.
	 *
	 * @return              True if the device is currently open.
	 */
	bool            is_open() { return _open_count > 0; }

protected:
	/**
	 * Pointer to the default cdev file operations table; useful for
	 * registering clone devices etc.
	 */
	static const struct block_operations	bops;

	/**
	 * Notification of the first open.
	 *
	 * This function is called when the device open count transitions from zero
	 * to one.  The driver lock is held for the duration of the call.
	 *
	 * The default implementation returns OK.
	 *
	 * @param inode		Pointer to the NuttX inode structure.
	 * @return		OK if the open should proceed, -errno otherwise.
	 */
	virtual int	open_first(struct inode *inode);

	/**
	 * Notification of the last close.
	 *
	 * This function is called when the device open count transitions from
	 * one to zero.  The driver lock is held for the duration of the call.
	 *
	 * The default implementation returns OK.
	 *
	 * @param inode		Pointer to the NuttX inode structure.
	 * @return		OK if the open should return OK, -errno otherwise.
	 */
	virtual int	close_last(struct inode *inode);

private:

	const char	*_devname;		/**< device node name */
	bool		_registered;		/**< true if device name was registered */
	unsigned	_open_count;		/**< number of successful opens */
};

}
