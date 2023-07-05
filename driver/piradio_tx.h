#ifndef _PIRADIO_TX_H
#define _PIRADIO_TX_H

#include "piradio0.h"

int piradio_prep_tx(struct sk_buff *skb, struct net_device *netdev);

netdev_tx_t piradio0_tx(struct sk_buff *skb, struct net_device *dev);

irqreturn_t tx_cmplt_callback(int irq, void *_dev);

irqreturn_t framer_ctrl_cmplt_callback(int irq, void *_dev);

int piradio_framer_ctrl(struct net_device *netdev, size_t reps);

irqreturn_t tx_log_cmplt_callback(int irq, void *_dev);

void tx_workqueue_fn(struct work_struct *work);

#endif