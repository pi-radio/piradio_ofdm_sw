#ifndef _PIRADIO_CONFIG_H
#define _PIRADIO_CONFIG_H

#include "piradio0.h"

int piradio_dma_alloc_tx_bds(struct piradio_dma *dma_obj, u32 num_bds,
		struct net_device *ndev);

int piradio_dma_alloc_rx_bds(struct piradio_dma *dma_obj,
		struct net_device *ndev);

int piradio_dma_dealloc_tx_bds(struct piradio_dma *dma_obj, u32 num_bds,
		struct net_device *ndev);

int piradio_dma_dealloc_rx_bds(struct piradio_dma *dma_obj,
		struct net_device *ndev);

void config_log(struct net_device *dev);

irqreturn_t tx_config_cmplt_callback(int irq, void *_dev);

int piradio_config_core(char *data, uint32_t len, struct net_device *netdev,
                         conf_t conf_type);

int piradio_soft_reset_dma(struct piradio_dma *dma_obj,
		struct net_device *ndev);


#endif /*_PIRADIO_CONFIG_H*/