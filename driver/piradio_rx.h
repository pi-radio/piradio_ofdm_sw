#ifndef _PIRADIO_RX_H
#define _PIRADIO_RX_H

#include "piradio0.h"

int piradio_rx(struct net_device *dev, int budget);

int piradio_rx_poll(struct napi_struct *napi, int quota);

irqreturn_t rx_cmplt_callback0(int irq, void *_dev);

irqreturn_t rx_log_cmplt_callback(int irq, void *_dev);

#endif /*_PIRADIO_RX_H*/