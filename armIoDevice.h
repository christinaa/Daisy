/*
 * armIoDevice.h
 * Copyright (c) 2013 Kristina Brooks
 *
 * ARM IO Device.
 */

#include <IOKit/IOService.h>

#ifndef _ARM_PE_H
#define _ARM_PE_H

class armIoDevice : public IOService
{
	OSDeclareDefaultStructors(armIoDevice);
	
private:
	struct ExpansionData { };
	ExpansionData *reserved;
	
public:
	virtual bool compareName( OSString * name, OSString ** matched = 0 ) const;
	virtual IOService *matchLocation(IOService *client);
	virtual IOReturn getResources( void );
	
	OSMetaClassDeclareReservedUnused(armIoDevice,  0);
	OSMetaClassDeclareReservedUnused(armIoDevice,  1);
	OSMetaClassDeclareReservedUnused(armIoDevice,  2);
	OSMetaClassDeclareReservedUnused(armIoDevice,  3);
};

#endif
