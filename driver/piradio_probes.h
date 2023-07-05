#ifndef _PIRADIO_PROBES_H
#define _PIRADIO_PROBES_H

#include "piradio0.h"

int piradio_dma_probe(struct platform_device *pdev,
		struct piradio_dma* dma_obj, int dev_tree_idx);

int piradio_reg_probe(struct platform_device *pdev,
	void __iomem **reg_base, const char * dev_tree_name, int dev_tree_idx);

// int piradio_mem_reg_probe(struct platform_device *pdev,
// 	void __iomem **reg_base);

// int piradio_memtx_reg_probe(struct platform_device *pdev,
// 	void __iomem **reg_base);

// int piradio_ifft_reg_probe(struct platform_device *pdev,
// 	void __iomem **reg_base);

// int piradio_dma_rx_probe(struct platform_device *pdev,
// 		struct piradio_dma* dma_obj, int dev_tree_idx);

#endif /*_PIRADIO_PROBES_H*/