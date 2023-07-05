#include "piradio_config.h"
#include "piradio_util.h"
#include <linux/module.h>

MODULE_LICENSE("Dual BSD/GPL");

int piradio_dma_alloc_tx_bds(struct piradio_dma *dma_obj, u32 num_bds,
		struct net_device *ndev){
	struct piradio_priv *priv = netdev_priv(ndev);
	u32 cr;
	int i;
	dma_obj->bd_curr = 0;
	dma_obj->bd_tail = 0;
	printk(KERN_ALERT "Allocating TX BDs\n");
	dma_obj->bd_list = dma_alloc_coherent(ndev->dev.parent,
			sizeof(*dma_obj->bd_list) * num_bds, &dma_obj->bd_list_base, GFP_KERNEL);
	if (!dma_obj->bd_list)
		goto out;

	for (i = 0; i < num_bds; i++) {
		dma_obj->bd_list[i].next = dma_obj->bd_list_base
				+ sizeof(*dma_obj->bd_list) * ((i + 1) % num_bds);
	}
	printk(KERN_ALERT "Allocating TX %d\n", read_reg(dma_obj->dma_regs + XAXIDMA_TX_CR_OFFSET));
	cr = piradio_dma_read32(dma_obj, XAXIDMA_TX_CR_OFFSET);
	
	cr = (((cr & ~XAXIDMA_COALESCE_MASK))
			| ((priv->coalesce_count_tx) << XAXIDMA_COALESCE_SHIFT));

	cr = (((cr & ~XAXIDMA_DELAY_MASK))
			| (XAXIDMA_DFT_TX_WAITBOUND << XAXIDMA_DELAY_SHIFT));
	/* Enable coalesce, delay timer and error interrupts */
	cr |= XAXIDMA_IRQ_ALL_MASK;
	piradio_dma_write32(dma_obj, XAXIDMA_TX_CR_OFFSET, cr);
	printk(KERN_ALERT "Allocating TX BDs\n");
	piradio_dma_bdout(dma_obj, XAXIDMA_TX_CDESC_OFFSET, dma_obj->bd_list_base);
	dma_rmb();
	cr = piradio_dma_read32(dma_obj, XAXIDMA_TX_CR_OFFSET);
	wmb();
	piradio_dma_write32(dma_obj, XAXIDMA_TX_CR_OFFSET, cr | XAXIDMA_CR_RUNSTOP_MASK);
	return 0;
out:
	printk(KERN_ALERT "Error in dma_alloc_coherent\n");
	return -ENOMEM;
}

int piradio_dma_dealloc_tx_bds(struct piradio_dma *dma_obj, u32 num_bds,
	struct net_device *ndev){
	struct piradio_priv *priv;
	priv = netdev_priv(ndev);
	u32 cr;
	dma_free_coherent(ndev->dev.parent,sizeof(*dma_obj->bd_list) * num_bds,
					dma_obj->bd_list, dma_obj->bd_list_base);
	dma_rmb();
	cr = piradio_dma_read32(dma_obj, XAXIDMA_TX_CR_OFFSET);
	cr &= ~(XAXIDMA_CR_RUNSTOP_MASK | XAXIDMA_IRQ_ALL_MASK);
	wmb();
	piradio_dma_write32(dma_obj, XAXIDMA_TX_CR_OFFSET, cr);
	return 0;
}

int piradio_dma_alloc_rx_bds(struct piradio_dma *dma_obj,
		struct net_device *ndev){
	struct sk_buff *skb;
	struct piradio_priv *priv = netdev_priv(ndev);
	u32 cr;
	int i;
	dma_obj->bd_curr = 0;
	dma_obj->bd_tail = priv->rx_bd_num - 1;

	dma_obj->bd_list = dma_alloc_coherent(ndev->dev.parent,
			sizeof(*dma_obj->bd_list) * priv->rx_bd_num, &dma_obj->bd_list_base, GFP_KERNEL);
	if (!dma_obj->bd_list){
		netdev_dbg(ndev, "Error in dma alloc coherent");
		goto out;

	}

	for (i = 0; i < priv->rx_bd_num; i++) {
		dma_obj->bd_list[i].next = dma_obj->bd_list_base
				+ sizeof(*dma_obj->bd_list) * ((i + 1) % priv->rx_bd_num);
		skb = netdev_alloc_skb(ndev,
							   priv->rx_transfer_length);
		if (!skb){
			netdev_dbg(ndev, "Could not alloc skb");
			goto out;
		}
		wmb();
		dma_obj->bd_list[i].phys = dma_map_single(priv->dev,
			    					skb->data,
									priv->rx_transfer_length,
									DMA_FROM_DEVICE);
		if(dma_mapping_error(priv->dev, dma_obj->bd_list[i].phys)){
			netdev_dbg(ndev, "dma_mapping_error");
			goto out;
		}

		dma_obj->bd_list[i].sw_id_offset = (phys_addr_t)skb;
		dma_obj->bd_list[i].cntrl = priv->rx_transfer_length;
		if(i % 2 == 0)
			dma_obj->bd_list[i].cntrl |= XAXIDMA_BD_CTRL_TXSOF_MASK;
		else	
			dma_obj->bd_list[i].cntrl |= XAXIDMA_BD_CTRL_TXEOF_MASK;
	}
	dma_rmb();
	cr = piradio_dma_read32(dma_obj, XAXIDMA_RX_CR_OFFSET);

	cr = (((cr & ~XAXIDMA_COALESCE_MASK))
			| ((priv->coalesce_count_rx) << XAXIDMA_COALESCE_SHIFT));

	cr = (((cr & ~XAXIDMA_DELAY_MASK))
			| (XAXIDMA_DFT_TX_WAITBOUND << XAXIDMA_DELAY_SHIFT));
	/* Enable coalesce, delay timer and error interrupts */
	cr |= XAXIDMA_IRQ_ALL_MASK;
	dma_rmb();
	piradio_dma_write32(dma_obj, XAXIDMA_RX_CR_OFFSET, cr);

	piradio_dma_bdout(dma_obj, XAXIDMA_RX_CDESC_OFFSET, dma_obj->bd_list_base);
	cr = piradio_dma_read32(dma_obj, XAXIDMA_RX_CR_OFFSET);
	piradio_dma_write32(dma_obj, XAXIDMA_RX_CR_OFFSET, cr | XAXIDMA_CR_RUNSTOP_MASK);
	piradio_dma_write32(dma_obj, XAXIDMA_RX_TDESC_OFFSET,
			 dma_obj->bd_list_base + sizeof(*dma_obj->bd_list) * dma_obj->bd_tail);
	return 0;
out:
	return -ENOMEM;

}

int piradio_dma_dealloc_rx_bds(struct piradio_dma *dma_obj,
		struct net_device *ndev){
	struct piradio_priv *priv;
	priv = netdev_priv(ndev);
	u32 cr;
	int i;
	for (i = 0; i < priv->rx_bd_num; i++) {
		kfree_skb((struct sk_buff*) dma_obj->bd_list[i].sw_id_offset);
		dma_unmap_single(priv->dev, dma_obj->bd_list[i].phys,
				priv->rx_transfer_length,
				DMA_FROM_DEVICE);
	}

	dma_free_coherent(ndev->dev.parent,sizeof(*dma_obj->bd_list) * priv->rx_bd_num,
					dma_obj->bd_list, dma_obj->bd_list_base);
	dma_rmb();
	size_t status = piradio_dma_read32(dma_obj, XAXIDMA_RX_CR_OFFSET);
	cr = piradio_dma_read32(dma_obj, XAXIDMA_RX_CR_OFFSET);
	cr &= ~(XAXIDMA_CR_RUNSTOP_MASK | XAXIDMA_IRQ_ALL_MASK);
	wmb();
	piradio_dma_write32(dma_obj, XAXIDMA_RX_CR_OFFSET, cr);
	status = piradio_dma_read32(dma_obj, XAXIDMA_RX_CR_OFFSET);
	return 0;
}

int piradio_soft_reset_dma(struct piradio_dma *dma_obj,
		struct net_device *ndev){
	u32 cr;
	cr = XAXIDMA_CR_RESET_MASK;
	piradio_dma_write32(dma_obj, XAXIDMA_TX_CR_OFFSET, cr);
	piradio_dma_write32(dma_obj, XAXIDMA_RX_CR_OFFSET, cr);
	return 0;
}

void config_log(struct net_device *dev){
	struct piradio_priv *priv = netdev_priv(dev);
	// priv->phys_tx = dma_map_single(priv->dev, priv->tx_log_buff,
	// 					priv->log_length,
	// 				   DMA_FROM_DEVICE);
	priv->phys_rx = dma_map_single(priv->dev, priv->rx_log_buff,
						priv->log_length,
					   DMA_FROM_DEVICE);
	// if (dma_mapping_error(priv->dev, priv->phys_tx)) {
	// 		netdev_dbg(dev, "Error with mapping TX");
	// 		netdev_dbg(dev, "Error with mapping TX");
	// }
	if (dma_mapping_error(priv->dev, priv->phys_rx)) {
			netdev_dbg(dev, "Error with mapping RX");
			netdev_dbg(dev, "Error with mapping RX");
	}

	// uint32_t cr = piradio_dma_read32(&priv->tx_log_dma, XAXIDMA_RX_CR_OFFSET);
	// cr = (uint32_t)(cr | XAXIDMA_CR_RUNSTOP_MASK);
	// piradio_dma_write32(&priv->tx_log_dma, XAXIDMA_RX_CR_OFFSET, cr); // Start engine

	// piradio_dma_write32(&priv->tx_log_dma, 0x30 + 0x18, priv->phys_tx); // Destination address
	// piradio_dma_write32(&priv->tx_log_dma, 0x30 + 0x28, priv->log_length);

	uint32_t cr = piradio_dma_read32(&priv->rx_log_dma, XAXIDMA_RX_CR_OFFSET);
	cr = (uint32_t)(cr | XAXIDMA_CR_RUNSTOP_MASK);
	piradio_dma_write32(&priv->rx_log_dma, XAXIDMA_RX_CR_OFFSET, cr); // Start engine

	piradio_dma_write32(&priv->rx_log_dma, 0x30 + 0x18, priv->phys_rx); // Destination address
	piradio_dma_write32(&priv->rx_log_dma, 0x30 + 0x28, priv->log_length);
	netdev_dbg(dev, "Log configured");
	netdev_dbg(dev, "Log configured");
}

irqreturn_t tx_config_cmplt_callback(int irq, void *_dev){
	printk(KERN_ALERT "tx_config_cmplt_callback called with %d", irq);
	printk(KERN_ALERT "tx_config_cmplt_callback called with %d", irq);
	u32 status;
	struct axidma_bd *cur_p;
	struct net_device *dev = (struct net_device *)_dev;
	struct piradio_priv *priv = netdev_priv(dev);
	struct piradio_dma* dma_obj;
	if(irq == priv->config_framer_dma.irq)
	 	dma_obj = &priv->config_framer_dma;
	else if(irq == priv->config_correlator_dma.irq)
		dma_obj = &priv->config_correlator_dma;
	else{
		printk(KERN_ALERT "Invalid IRQ int ");
		return IRQ_NONE;
	}
	status = piradio_dma_read32(dma_obj, XAXIDMA_TX_SR_OFFSET);
	if (status & (XAXIDMA_IRQ_IOC_MASK | XAXIDMA_IRQ_DELAY_MASK)) {
		piradio_dma_write32(dma_obj, XAXIDMA_TX_SR_OFFSET, status);
		cur_p = &dma_obj->bd_list[dma_obj->bd_curr];
		status = cur_p->status;
		while (status & XAXIDMA_BD_STS_COMPLETE_MASK) {
			dma_unmap_single(dev->dev.parent, cur_p->phys,
					 cur_p->cntrl &
					 XAXIDMA_BD_CTRL_LENGTH_MASK,
					 DMA_TO_DEVICE);
			cur_p->status = 0;
			cur_p->cntrl = 0;
			if (++dma_obj->bd_curr >= priv->tx_bd_config_num)
				dma_obj->bd_curr = 0;
			cur_p = &dma_obj->bd_list[dma_obj->bd_curr];
			status = cur_p->status;
		}
		smp_mb();
		printk(KERN_INFO "Handled ");
		printk(KERN_INFO "handled ");
		return IRQ_HANDLED;
	}
	if (!(status & XAXIDMA_IRQ_ALL_MASK)){
		printk(KERN_ALERT "TX Interrupt received no 2. %d ", status
						);
		return IRQ_NONE;
	}
	if (status & XAXIDMA_IRQ_ERROR_MASK) {
		printk(KERN_ALERT "TX Interrupt received error %d ", status
								);
		return IRQ_NONE;
	}
	printk(KERN_INFO "Handled 2");
	printk(KERN_INFO "handled 2");
	return IRQ_HANDLED;
}	

int piradio_config_core(char *data, uint32_t len, struct net_device *netdev, conf_t conf_type){
		struct piradio_priv *priv = netdev_priv(netdev);
		struct piradio_dma *pdma;
		if(conf_type == CONFIG_FRAMER)
			pdma = &priv->config_framer_dma;
		else if(conf_type == CONFIG_CORRELATOR)
			pdma = &priv->config_correlator_dma;
		else {
			printk(KERN_ALERT "conf_type error ");
			return -1;
		}
		struct axidma_bd *cur_p;
		dma_addr_t tail_base;
		cur_p = &pdma->bd_list[pdma->bd_tail];

		cur_p->phys = dma_map_single(priv->dev, data,
				len, DMA_TO_DEVICE);
		cur_p->cntrl = len | XAXIDMA_BD_CTRL_TXSOF_MASK | XAXIDMA_BD_CTRL_TXEOF_MASK;
		cur_p->tx_desc_mapping = 0;

		tail_base = pdma->bd_list_base + sizeof(*pdma->bd_list) * pdma->bd_tail;
		wmb();
		piradio_dma_bdout(pdma, XAXIDMA_TX_TDESC_OFFSET, tail_base);

		if (++pdma->bd_tail >= priv->tx_bd_config_num)
				pdma->bd_tail = 0;
		return 0;
}