/*
 * diana.cpp
 * Copyright (c) 2013 Kristina Brooks
 *
 * OMAP3 Interrupts. Based on the mach-omap driver for Linux.
 */

#include "diana.h"

#include <IOKit/IOLib.h>
#include <IOKit/IOPlatformExpert.h>

/* selected INTC register offsets */

#define INTC_REVISION		0x0000
#define INTC_SYSCONFIG		0x0010
#define INTC_SYSSTATUS		0x0014
#define INTC_SIR		0x0040
#define INTC_CONTROL		0x0048
#define INTC_PROTECTION		0x004C
#define INTC_IDLE		0x0050
#define INTC_THRESHOLD		0x0068
#define INTC_MIR0		0x0084
#define INTC_MIR_CLEAR0		0x0088
#define INTC_MIR_SET0		0x008c

#define INTC_PENDING_IRQ0	0x0098

/* Number of IRQ state bits in each MIR register */
#define IRQ_BITS_PER_REG	32

#define INTCPS_NR_MIR_REGS      3
#define INTCPS_NR_IRQS          96
#define INTCPS_SIR_IRQ_OFFSET   0x0040  /* omap2/3 active interrupt offset */
#define ACTIVEIRQ_MASK          0x7f    /* omap2/3 active interrupt bits */

static struct omap_irq_bank {
	uint8_t* base_reg;
	unsigned int nr_irqs;
} __attribute__ ((aligned(4))) irq_banks[] = {
	{
		/* MPU INTC */
		.base_reg	= 0,
		.nr_irqs	= 96,
	},
};

#define BASE_ADDR ((uint8_t*)picBaseAddress)
#define NUM_VECTORS INTCPS_NR_IRQS
/* Structure to save interrupt controller context */
struct omap3_intc_regs {
	u32 sysconfig;
	u32 protection;
	u32 idle;
	u32 threshold;
	u32 ilr[INTCPS_NR_IRQS];
	u32 mir[INTCPS_NR_MIR_REGS];
};

static void intc_bank_write_reg(u32 val, struct omap_irq_bank *bank, u16 reg)
{
	__raw_writel(val, bank->base_reg + reg);
}

static u32 intc_bank_read_reg(struct omap_irq_bank *bank, u16 reg)
{
	return __raw_readl(bank->base_reg + reg);
}

static void omap_ack_irq(unsigned int irq)
{
	intc_bank_write_reg(0x1, &irq_banks[0], INTC_CONTROL);
}

static void omap_irq_bank_init_one(struct omap_irq_bank *bank)
{
	unsigned long tmp;
	
	tmp = intc_bank_read_reg(bank, INTC_REVISION) & 0xff;
	
	IOLog("IRQ: Found an INTC at 0x%p "
		   "(revision %ld.%ld) with %d interrupts\n",
		   bank->base_reg, tmp >> 4, tmp & 0xf, bank->nr_irqs);
	
	tmp = intc_bank_read_reg(bank, INTC_SYSCONFIG);
	tmp |= 1 << 1;	/* soft reset */
	intc_bank_write_reg(tmp, bank, INTC_SYSCONFIG);
	
	while (!(intc_bank_read_reg(bank, INTC_SYSSTATUS) & 0x1))
	/* Wait for reset to complete */;
	
	/* Enable autoidle */
	intc_bank_write_reg(1 << 0, bank, INTC_SYSCONFIG);
}

static void omap_mask_irq(unsigned int irq)
{
	int offset = irq & (~(IRQ_BITS_PER_REG - 1));
	
	irq &= (IRQ_BITS_PER_REG - 1);
	
	intc_bank_write_reg(1 << irq, &irq_banks[0], INTC_MIR_SET0 + offset);
}

static void omap_unmask_irq(unsigned int irq)
{
	int offset = irq & (~(IRQ_BITS_PER_REG - 1));
	
	irq &= (IRQ_BITS_PER_REG - 1);
	
	intc_bank_write_reg(1 << irq, &irq_banks[0], INTC_MIR_CLEAR0 + offset);
}

/********************************************************************/

#define super IOInterruptController

bool diana::start(IOService *provider)
{
	long cnt;
	OSObject *tmpObject;
	IOInterruptAction handler;
	
	if (!super::start(provider))
		return false;
	
	if (irq_banks[0].base_reg != 0) {
		panic("diana: IRQ bank 0 is already mapped, wtf?");
	}
	
	/* get parents' name */
	tmpObject = provider->getProperty("InterruptControllerName");
	interruptControllerName = OSDynamicCast(OSSymbol, tmpObject);
	if (interruptControllerName == 0) return false;
	
	/* set it to this one */
	setProperty("InterruptControllerName", interruptControllerName);
	
	
	picMemoryMap = provider->mapDeviceMemoryWithIndex(0);
	
	if (picMemoryMap == 0) {
		return false;
	}
	
	/* base address */
	picBaseAddress = picMemoryMap->getVirtualAddress();
	irq_banks[0].base_reg = (uint8_t*)picBaseAddress;
	
	/* alloc memory for vectors */
	vectors = (IOInterruptVector *)IOMalloc((NUM_VECTORS) * sizeof(IOInterruptVector));
	if (vectors == NULL) return false;
	bzero(vectors, (NUM_VECTORS) * sizeof(IOInterruptVector));
	
	/* alloc locks */
	for (cnt = 0; cnt < (NUM_VECTORS) ; cnt++) {
		vectors[cnt].interruptLock = IOLockAlloc();
		if (vectors[cnt].interruptLock == NULL) {
			for (cnt = 0; cnt < (NUM_VECTORS); cnt++) {
				if (vectors[cnt].interruptLock != NULL)
					IOLockFree(vectors[cnt].interruptLock);
			}
			return false;
		}
	}
	
	/* init irq banks */
	omap_irq_bank_init_one(&irq_banks[0]);
	
	registerService();
	
	/* subscribe to cpu irqs */
	getPlatform()->setCPUInterruptProperties(provider);
	handler = getInterruptHandlerAddress();
	provider->registerInterrupt(cnt, this, handler, 0);
	provider->enableInterrupt(cnt);
	
	/* reg this interrupt handler */
	getPlatform()->registerInterruptController(interruptControllerName, this);
	
	/* done */
	return true;
}

IOReturn diana::getInterruptType(IOService *nub, int source, int *interruptType)
{
	return kIOReturnSuccess;
}

#define assert_vector_number(irq) if (irq > NUM_VECTORS) {panic("diana: Eeek, invalid interrupt vector %d", (int)irq); }

IOReturn diana::dispatchInterrupt(void *refCon, IOService *nub, int source, int irq)
{
	IOInterruptVector *vector;
	
	assert_vector_number(irq);
	
	vector = &vectors[irq];
	vector->interruptActive = 1;
	
	if (!vector->interruptDisabledSoft) {
		/* call the handler if we have one */
		if (vector->interruptRegistered) {
			vector->handler(vector->target, vector->refCon,
							vector->nub, vector->source);
		}
	}
	
	vector->interruptActive = 0;
	
	/* acknowledge the irq */
	omap_ack_irq(irq);
	
	return kIOReturnSuccess;
}

IOReturn diana::handleInterrupt(void *refCon, IOService *nub, int source)
{
	u32 irqnr;
	
	do {
		irqnr = __raw_readl(BASE_ADDR + INTC_PENDING_IRQ0);
		if (irqnr) {
			goto out;
		}
		
		irqnr = __raw_readl(BASE_ADDR + INTC_PENDING_IRQ0 + IRQ_BITS_PER_REG);
		if (irqnr) {
			goto out;
		}
		
		irqnr = __raw_readl(BASE_ADDR + INTC_PENDING_IRQ0 + IRQ_BITS_PER_REG*2);

#if 0
		if (irqnr) {
			goto out;
		}
		irqnr = __raw_readl(BASE_ADDR + INTC_PENDING_IRQ0 + IRQ_BITS_PER_REG*3);
#endif
		
	out:
		if (!irqnr) {
			break;
		}
		
		irqnr = __raw_readl(BASE_ADDR + INTCPS_SIR_IRQ_OFFSET);
		irqnr &= ACTIVEIRQ_MASK;
		
		if (irqnr) {
			return dispatchInterrupt(refCon, nub, source, irqnr);
		}
	} while (irqnr);
	
	return kIOReturnError;
}

IOInterruptAction diana::getInterruptHandlerAddress(void)
{
	/* lol */
	return OSMemberFunctionCast(IOInterruptAction,
								this,
								&diana::handleInterrupt);
}

bool diana::vectorCanBeShared(long vectorNumber, IOInterruptVector *vector)
{
	return false;
}

void diana::initVector(long vectorNumber, IOInterruptVector *vector)
{
	assert_vector_number(vectorNumber);
	
	IOLockLock(vector->interruptLock);
	omap_unmask_irq(vectorNumber);
	IOLockUnlock(vector->interruptLock);
}

void diana::disableVectorHard(long vectorNumber, IOInterruptVector *vector)
{
	
}

void diana::enableVector(long vectorNumber, IOInterruptVector *vector)
{
	
}

void diana::causeVector(long vectorNumber, IOInterruptVector *vector)
{
	
}
