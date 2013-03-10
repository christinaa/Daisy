/*
 * armIoDevice.cpp
 * Copyright (c) 2013 Kristina Brooks
 *
 * ARM IO Device. Loosely based on the AppleMacIO driver.
 */

#include <IOKit/IOLib.h>
#include <IOKit/IODeviceTreeSupport.h>
#include <IOKit/IODeviceMemory.h>

#include "armIoDevice.h"
#include "armIo.h"

#define super IOService

OSDefineMetaClassAndAbstractStructors(armIoDevice, IOService);
OSMetaClassDefineReservedUnused(armIoDevice,  0);
OSMetaClassDefineReservedUnused(armIoDevice,  1);
OSMetaClassDefineReservedUnused(armIoDevice,  2);
OSMetaClassDefineReservedUnused(armIoDevice,  3);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

bool armIoDevice::compareName(OSString* name, OSString** matched) const
{
	return (IODTCompareNubName(this, name, matched) ||
			IORegistryEntry::compareName(name, matched));
}

IOService *armIoDevice::matchLocation(IOService * /* client */)
{
	return this;
}

IOReturn armIoDevice::getResources( void )
{
	IOService *armIO = this;
	
	if (getDeviceMemory() != 0) return kIOReturnSuccess;
	
	while (armIO && ((armIO = armIO->getProvider()) != 0))
		if (strcmp("arm-io", armIO->getName()) == 0) break;
	
	if (armIO == 0) return kIOReturnError;
	
	IODTResolveAddressing(this, "reg", armIO->getDeviceMemoryWithIndex(0));
	
	return kIOReturnSuccess;
}