/*
 * compat.h
 * Copyright (c) 2013 Kristina Brooks
 *
 * Some glue for recycling Linux code.
 */

#ifndef _COMPAT_H
#define _COMPAT_H

#include <stdint.h>
#include <sys/types.h>

typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;

typedef u32 dma_addr_t;

#define __force
#define __iomem

/* Compiler */

#define likely(x)       __builtin_expect((x),1)
#define unlikely(x)     __builtin_expect((x),0)

/* Errno */

#define ENOMEM 12
#define EBUSY 16
#define EINVAL 22
#define ENODEV 19

/* IO stuff */

static inline void __raw_writew(u16 val, volatile void *addr)
{
	asm volatile("strh %1, %0"
				 : "+Qo" (*(volatile u16 __force *)addr)
				 : "r" (val));
}

static inline u16 __raw_readw(const volatile void __iomem *addr)
{
	u16 val;
	asm volatile("ldrh %1, %0"
				 : "+Qo" (*(volatile u16 __force *)addr),
				 "=r" (val));
	
	return val;
}

static inline void __raw_writeb(u8 val, volatile void __iomem *addr)
{
	asm volatile("strb %1, %0"
				 : "+Qo" (*(volatile u8 __force *)addr)
				 : "r" (val));
}

static inline void __raw_writel(u32 val, volatile void *addr)
{
	asm volatile("str %1, %0"
				 : "+Qo" (*(volatile u32 __force *)addr)
				 : "r" (val));
}

static inline u8 __raw_readb(const volatile void __iomem *addr)
{
	u8 val;
	asm volatile("ldrb %1, %0"
				 : "+Qo" (*(volatile u8 __force *)addr),
				 "=r" (val));
	
	return val;
}

static inline u32 __raw_readl(const volatile void __iomem *addr)
{
	u32 val;
	asm volatile("ldr %1, %0"
	: "+Qo" (*(volatile u32 __force *)addr),
	"=r" (val));
	
	return val;
}

#endif