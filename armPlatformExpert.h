/*
 * armPlatformExpert.h
 * Copyright (c) 2013 Kristina Brooks
 *
 * ARM Platform Expert.
 */

#include <IOKit/IOPlatformExpert.h>

#ifndef _ARM_PE_H
#define _ARM_PE_H

class armPlatformExpert : public IOPlatformExpert
{
	OSDeclareDefaultStructors(armPlatformExpert)
	
public:
	virtual bool init(OSDictionary *propTable);

	virtual IOService * probe(IOService *provider, SInt32 *score);

	virtual bool start(IOService * provider);

	virtual bool getMachineName(char *name, int maxLength);
};

#endif
