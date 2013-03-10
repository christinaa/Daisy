/*
 * danielle.h
 * Copyright (c) 2013 Kristina Brooks
 *
 * OMAP3 Serial port.
 */

#ifndef _DIANA_H
#define _DIANA_H

#include <IOKit/IOInterrupts.h>
#include <IOKit/IOInterruptController.h>

#include "compat.h"

class danielleSerialSync : public IOService
{
	OSDeclareDefaultStructors(danielleSerialSync)	
};

class danielleUartSync : public danielleSerialSync
{
	OSDeclareDefaultStructors(danielleUartSync)
};

class danielleSerialBsdClient : public IOService
{
	OSDeclareDefaultStructors(danielleSerialBsdClient)
	
private:
	dev_t _device;
	void* _node;
	danielleSerialSync* _sync;
	
public:
	virtual bool start(IOService *provider);
	
	int open(int flags, int devtype, struct proc * p);
	void close(int flags, int devtype, struct proc *p);
	int read(struct uio *uio, int ioflag);
    int write(struct uio *uio, int ioflag);
	int ioctl(u_long cmd, caddr_t data, int fflag, struct proc *p);
};

/* based on omap-serial from Linux */
class danielleUartPort : public IOService
{
	OSDeclareDefaultStructors(danielleUartPort)

private:
	uint32_t regshift;
	uint8_t* membase;
	
	bool isByte;
	
	unsigned char		ier;
	unsigned char		lcr;
	unsigned char		mcr;
	unsigned char		efr;
	int			use_dma;
	int			is_buf_dma_alloced;
	
	/*
	 * Some bits in registers are cleared on a read, so they must
	 * be saved whenever the register is read but the bits will not
	 * be immediately processed.
	 */
	unsigned int		lsr_break_flag;
#define MSR_SAVE_FLAGS UART_MSR_ANY_DELTA
	unsigned char		msr_saved_flags;
	char			name[20];
	int			use_console;
	IOLock*		uart_lock;
	
public:
	/* main stuff */
	unsigned int serial_in(int offset);
	void serial_out(int offset, int value);
	
	unsigned int get_divisor(unsigned int baud);
	void clear_fifos();
	void stop_rxdma();
	void enable_ms();
	void start_tx();
	void stop_tx();
	void stop_rx();
	void receive_chars(int *status);
	void transmit_chars();
	unsigned int check_modem_status();
	void set_mctrl(unsigned int mctrl);
};

#endif