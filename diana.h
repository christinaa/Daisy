/*
 * diana.h
 * Copyright (c) 2013 Kristina Brooks
 *
 * OMAP3 Interrupts.
 */

#ifndef _DIANA_H
#define _DIANA_H

#include <IOKit/IOInterrupts.h>
#include <IOKit/IOInterruptController.h>

#include "compat.h"

class diana : public IOInterruptController
{
	OSDeclareDefaultStructors(diana)
	
private:
	IOLogicalAddress picBaseAddress;
	IOMemoryMap *picMemoryMap;
	IOInterruptVector *vectors;
	OSSymbol* interruptControllerName;
	
	IOReturn dispatchInterrupt(void *refCon, IOService *nub, int source, int irq);
	
public:
	virtual bool start(IOService *provider);
	virtual IOReturn getInterruptType(IOService *nub, int source, int *interruptType);
	virtual IOInterruptAction getInterruptHandlerAddress(void);
	virtual IOReturn handleInterrupt(void *refCon, IOService *nub, int source);
	virtual bool vectorCanBeShared(long vectorNumber, IOInterruptVector *vector);
	virtual void initVector(long vectorNumber, IOInterruptVector *vector);
	virtual void disableVectorHard(long vectorNumber, IOInterruptVector *vector);
	virtual void enableVector(long vectorNumber, IOInterruptVector *vector);
	virtual void causeVector(long vectorNumber, IOInterruptVector *vector);
};

#endif