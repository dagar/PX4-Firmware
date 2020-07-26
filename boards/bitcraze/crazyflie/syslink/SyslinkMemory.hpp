
#pragma once

#include <lib/cdev/CDev.hpp>

class SyslinkMemory : public cdev::CDev
{

public:
	SyslinkMemory(Syslink *link);
	virtual ~SyslinkMemory() = default;

	virtual int	init();

	virtual ssize_t	read(struct file *filp, char *buffer, size_t buflen);
	virtual ssize_t	write(struct file *filp, const char *buffer, size_t buflen);
	virtual int	ioctl(struct file *filp, int cmd, unsigned long arg);

private:
	friend class Syslink;

	Syslink *_link;

	int _activeI;

	syslink_message_t msgbuf;

	uint8_t scan();
	void getinfo(int i);

	int read(int i, uint16_t addr, char *buf, int length);
	int write(int i, uint16_t addr, const char *buf, int length);

	void sendAndWait();

};
