#ifndef _PIRADIO_UTIL_H
#define _PIRADIO_UTIL_H

#include <linux/types.h>
#include <linux/io.h>
#include "piradio0.h"

static inline void piradio_dma_bdout(struct piradio_dma *q,
				     off_t reg, dma_addr_t value)
{
#if defined(CONFIG_PHYS_ADDR_T_64BIT)
	writeq(value, (q->dma_regs + reg));
#else
	writel(value, (q->dma_regs + reg));
#endif
}

static inline void write_reg(__u32 value, void __iomem *base)
{
	writel(value, base);
}

static inline __u32 read_reg(void __iomem *base)
{
	return readl(base);
}

static inline u32 piradio_dma_read32(struct piradio_dma *dma_obj, off_t reg){
	return ioread32(dma_obj->dma_regs + reg);
}

static inline void piradio_dma_write32(struct piradio_dma *dma_obj,
				     off_t reg, u32 value)
{

	iowrite32(value, dma_obj->dma_regs + reg);
}

#endif /*_PIRADIO_UTIL_H*/