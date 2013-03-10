/*
 * danielle.cpp
 * Copyright (c) 2013 Kristina Brooks
 *
 * OMAP3 Serial port.
 */

/*
 * I have no idea how the IOSerialFamily included
 * these without defining KERNEL_PRIVATE.
 */
#ifndef KERNEL_PRIVATE
#define KERNEL_PRIVATE 1
#endif

#include <sys/types.h>
#include <kern/kern_types.h>

__BEGIN_DECLS

#include <kern/thread.h>
#include <sys/time.h>

__END_DECLS

#include <sys/proc.h>
#include <sys/errno.h>
#include <sys/dkstat.h>
#include <sys/fcntl.h>
#include <sys/conf.h>
#include <sys/tty.h>
#include <sys/ucred.h>
#include <sys/kernel.h>

#include <miscfs/devfs/devfs.h>
#include <sys/systm.h>
#include <sys/kauth.h>

#undef KERNEL_PRIVATE

#include <IOKit/IOLib.h>
#include "danielle.h"

/*
static inline int bsdld_open(dev_t dev, struct tty *tp)
{ return (*linesw[tp->t_line].l_open)(dev, tp); }

static inline int bsdld_close(struct tty *tp, int flag)
{ return (*linesw[tp->t_line].l_close)(tp, flag); }

static inline int bsdld_read(struct tty *tp, struct uio *uio, int flag)
{ return (*linesw[tp->t_line].l_read)(tp, uio, flag); }

static inline int bsdld_write(struct tty *tp, struct uio *uio, int flag)
{ return (*linesw[tp->t_line].l_write)(tp, uio, flag); }

static inline int
bsdld_ioctl(struct tty *tp, u_long cmd, caddr_t data, int flag,
			struct proc *p)
{ return (*linesw[tp->t_line].l_ioctl)(tp, cmd, data, flag, p); }

static inline int bsdld_rint(int c, struct tty *tp)
{ return (*linesw[tp->t_line].l_rint)(c, tp); }

static inline void  bsdld_start(struct tty *tp)
{ (*linesw[tp->t_line].l_start)(tp); }

static inline int bsdld_modem(struct tty *tp, int flag)
{ return (*linesw[tp->t_line].l_modem)(tp, flag); }
*/

#define kIOTTYBaseNameKey "IOTTYBaseName"
#define kIOTTYSuffixKey "IOTTYSuffix"

bool danielleSerialBsdClient::start(IOService *provider)
{
	OSString* baseName;
	OSString* ttySuffix;
	void* node;
	
	/* get provider */
	_sync = OSDynamicCast(danielleSerialSync, provider);
	assert(sync);
	
	baseName = (OSString*)provider->getProperty(kIOTTYBaseNameKey);
	ttySuffix = (OSString*)provider->getProperty(kIOTTYBaseNameKey);
	
	/* blah */
	assert(baseName != NULL && ttySuffix != NULL);
	
	/* woah dude */
	node = 
	devfs_make_node(_device,
					0,
					0,
					0,
					0666,
					"uart.%s%s",
					baseName,
					ttySuffix);
	
	if (!node) {
		IOLog("danielleSerialBsdClient: couldn't make a devfs node");
		return false;
	}
	
	_node = node;
	
	return true;
}

int danielleSerialBsdClient::open(int flags, int devtype, struct proc * p)
{
	
}

void danielleSerialBsdClient::close(int flags, int devtype, struct proc *p)
{
	
}

int danielleSerialBsdClient::read(struct uio *uio, int ioflag)
{
	
}

int danielleSerialBsdClient::write(struct uio *uio, int ioflag)
{
	
}

int danielleSerialBsdClient::ioctl(u_long cmd, caddr_t data, int fflag, struct proc *p)
{
	
}

/**************************************************************/

unsigned int danielleUartPort::serial_in(int offset)
{
	offset <<= regshift;
	
	if (isByte)
		return __raw_readb(membase + offset);
	else
		return __raw_readw(membase + offset);
}

void danielleUartPort::serial_out(int offset, int value)
{
	offset <<= regshift;
	
	if (isByte)
		__raw_writeb(value, membase + offset);
	else
		__raw_writew(value, membase + offset);
}