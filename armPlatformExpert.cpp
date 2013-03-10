/*
 * armPlatformExpert.cpp
 * Copyright (c) 2013 Kristina Brooks
 *
 * ARM Platform Expert.
 */

#include "armPlatformExpert.h"

#define super IOPlatformExpert

bool armPlatformExpert::init(OSDictionary *propTable)
{
    if (!super::init(propTable))  return false;
    return true;
}

bool armPlatformExpert::start(IOService * provider)
{
    if (!super::start(provider))
        return false;

    registerService();
	
    return true;
}

IOService * armPlatformExpert::probe(IOService* provider, SInt32* score)
{
	/* Pick meeeeeee! */
	*score = 100000;
    return this;
}

bool armPlatformExpert::getMachineName( char * name, int maxLength )
{
    strncpy(name, "arm", maxLength);
    return true;
}
