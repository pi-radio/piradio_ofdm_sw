#include "piradio_tx.h"
#include "piradio_util.h"
#include <linux/ip.h>
#include <linux/crc32.h>
#include <linux/kernel.h>
#include <linux/etherdevice.h>
#include <linux/module.h>


MODULE_LICENSE("Dual BSD/GPL");


int piradio_setup_dma_tx(uint8_t* tx_buffer, struct net_device *netdev){
	struct piradio_priv *priv;
    priv = netdev_priv(netdev);
	dma_addr_t tail_base;
	struct piradio_dma *pdma = &priv->tx_dma;
	struct axidma_bd *cur_p, *next_p;
	unsigned long l_flags;
	u32 nxt_p_offset;

	spin_lock_irqsave(&pdma->lock, l_flags);
	cur_p = &pdma->bd_list[pdma->bd_tail];
	
	cur_p->phys = dma_map_single(priv->dev, tx_buffer, //skb->data,
			priv->tx_transfer_length, DMA_TO_DEVICE);
	cur_p->cntrl = (priv->tx_transfer_length)| XAXIDMA_BD_CTRL_TXSOF_MASK | XAXIDMA_BD_CTRL_TXEOF_MASK;
	cur_p->tx_desc_mapping = 0;

	tail_base = pdma->bd_list_base + sizeof(*pdma->bd_list) * pdma->bd_tail;
	wmb();
	piradio_dma_bdout(pdma, XAXIDMA_TX_TDESC_OFFSET, tail_base);

	if (++pdma->bd_tail >= priv->tx_bd_num)
			pdma->bd_tail = 0;

	nxt_p_offset = (pdma->bd_tail + 1) % priv->tx_bd_num;
	next_p = &pdma->bd_list[nxt_p_offset];
	if(next_p->cntrl != 0){
		//netif_stop_queue(netdev); // TODO Check for outstanding packets if we ever get here
		smp_mb();
		spin_unlock_irqrestore(&pdma->lock, l_flags);
		return NETDEV_TX_BUSY;
	}
	spin_unlock_irqrestore(&pdma->lock, l_flags);
	return 0;
}

int breakup_packet(struct net_device *netdev){
	struct piradio_priv *priv;
    priv = netdev_priv(netdev);
	struct piradio_hdr *pir;
	uint32_t num_frames;
	int i, ret;
	uint32_t bytes_sent;
	unsigned long l_flags;
	struct tx_state *tx_state = &priv->state_tx;
	struct sk_buff *skb = tx_state->pending_tx_skb;
	num_frames = (skb->len % (priv->tx_transfer_length - PIRADIO_HDR_LEN) == 0) ? 
							skb->len / (priv->tx_transfer_length - PIRADIO_HDR_LEN) :
							skb->len / (priv->tx_transfer_length - PIRADIO_HDR_LEN) + 1;
		//netdev_dbg(netdev, "Size = %d will send %d frames", skb->len, num_frames);
	bytes_sent = skb->len - tx_state->remaining_bytes;
	int start_index = bytes_sent / (priv->tx_transfer_length - PIRADIO_HDR_LEN);
	for(i = start_index; i < num_frames; i++){
		pir = (struct piradio_hdr *)priv->buff_tmp;
		if(i == 0){ // First Frame
			pir->p_type = PIR_T_DATA | PIR_T_FRAME_START | (0xAE23B8EE << 5);
			pir->p_seq_num = htonl(priv->sequence_num++) | (0x39CDAA71 >> 16);
			pir->p_len = (priv->tx_transfer_length - PIRADIO_HDR_LEN);
			memcpy(&priv->buff_tmp[PIRADIO_HDR_LEN], &skb->data[i * (priv->tx_transfer_length - PIRADIO_HDR_LEN)],
																pir->p_len);
			pir->p_csum = ~crc32_le(~0, priv->buff_tmp + PIRADIO_HDR_LEN, pir->p_len);
			
		}
		else if(i == num_frames - 1){ // Last Frame
			memcpy(priv->buff_tmp, priv->buff, priv->tx_transfer_length );
			pir->p_type = PIR_T_DATA | PIR_T_FRAME_END | (0xAE23B8EE << 5);
			pir->p_seq_num = htonl(priv->sequence_num++) | (0x39CDAA71 >> 16);
			pir->p_len = (skb->len % (priv->tx_transfer_length - PIRADIO_HDR_LEN)) == 0 ? (priv->tx_transfer_length - PIRADIO_HDR_LEN)
																: skb->len % (priv->tx_transfer_length - PIRADIO_HDR_LEN);
			memcpy(&priv->buff_tmp[PIRADIO_HDR_LEN], &skb->data[i * (priv->tx_transfer_length - PIRADIO_HDR_LEN)],
																pir->p_len);
			pir->p_csum = ~crc32_le(~0, priv->buff_tmp + PIRADIO_HDR_LEN, pir->p_len);
		}
		else{
			pir->p_type = PIR_T_DATA | PIR_T_FRAME_CONT | (0xAE23B8EE << 5);
			pir->p_len = (priv->tx_transfer_length - PIRADIO_HDR_LEN);
			pir->p_seq_num = htonl(priv->sequence_num++) | (0x39CDAA71 >> 16);
			memcpy(&priv->buff_tmp[PIRADIO_HDR_LEN], &skb->data[i * (priv->tx_transfer_length - PIRADIO_HDR_LEN)],
																pir->p_len );
			pir->p_csum = ~crc32_le(~0, priv->buff_tmp + PIRADIO_HDR_LEN, priv->tx_transfer_length - PIRADIO_HDR_LEN);
		}
		pir->p_rand = 0x39CDAA71;
		piradio_framer_ctrl(netdev, 1); // This should change depending on mumber of frames per transmission
		netdev_dbg(netdev, "TX len = %d CSUM = %d", pir->p_len, pir->p_csum);
		pir->p_len = htonl(pir->p_len);
		spin_lock_irqsave(&priv->tx_byte_cnt_lock, l_flags);
		tx_state->remaining_bytes -= ntohl(pir->p_len);
		ret = piradio_setup_dma_tx(priv->buff_tmp, netdev);
		if(tx_state->remaining_bytes == 0){
			dev_kfree_skb_irq((struct sk_buff *)skb);
			if(netif_queue_stopped(netdev)){
				netif_wake_queue(netdev);
			}
		}
		if(ret == NETDEV_TX_BUSY){
			if(!netif_queue_stopped(netdev)){
				netif_stop_queue(netdev);
			}
			spin_unlock_irqrestore(&priv->tx_byte_cnt_lock, l_flags);
			return 0;
		}
		spin_unlock_irqrestore(&priv->tx_byte_cnt_lock, l_flags);
	}
	return 0;
}

void tx_workqueue_fn(struct work_struct *work){
	
	struct tx_wq_item *my_work = container_of(work, struct tx_wq_item, work);
	breakup_packet(my_work->netdev);
	return;
}

int piradio_prep_tx(struct sk_buff *skb, struct net_device *netdev)
{
	struct piradio_priv *priv;
    priv = netdev_priv(netdev);
	if(priv->driver_state != DRIVER_READY)
		return NETDEV_TX_BUSY;
	int ret;
	struct piradio_hdr *pir;
	int i;
	if(skb->len == 0)
		return -1;
	priv->state_tx.pending_tx_skb = skb;
	priv->state_tx.remaining_bytes = skb->len;
	if(skb->len > (priv->tx_transfer_length - PIRADIO_HDR_LEN)){ // Larger packet than one frame
		breakup_packet(netdev);
	}
	else{
		memcpy(priv->buff_tmp, priv->buff, priv->tx_transfer_length );
		pir = (struct piradio_hdr *)priv->buff_tmp;
		pir->p_type = PIR_T_DATA | PIR_T_FRAME_START | PIR_T_FRAME_END | (0xAE23B8EE << 5);
		pir->p_seq_num = htonl(priv->sequence_num++) | (0x39CDAA71 >> 16);
		pir->p_rand = 0x39CDAA71;
		pir->p_len = (skb->len % (priv->tx_transfer_length - PIRADIO_HDR_LEN)) == 0 ? (priv->tx_transfer_length - PIRADIO_HDR_LEN)
															: skb->len % (priv->tx_transfer_length - PIRADIO_HDR_LEN);
		memcpy(&priv->buff_tmp[PIRADIO_HDR_LEN], skb->data, pir->p_len);
		pir->p_csum = ~crc32_le(~0, priv->buff_tmp + PIRADIO_HDR_LEN, pir->p_len);
		piradio_framer_ctrl(netdev, 1);
		pir->p_len = htonl(pir->p_len);
		//pir->p_len = (0x39CDAA71BC4C197F);
		ret = piradio_setup_dma_tx(priv->buff_tmp, netdev);
		priv->state_tx.remaining_bytes = 0;
		//priv->state_tx.remaining_bytes -= ntohl(pir->p_len) & 0xFF;
		if(priv->state_tx.remaining_bytes == 0){
			dev_kfree_skb_irq((struct sk_buff *)skb);
		}
		if(ret == NETDEV_TX_BUSY){
			netif_stop_queue(netdev);
		}
		//netdev_dbg(netdev, "Size = %d will send %d frames", skb->len, 1);
	}
	
	//netdev_dbg(netdev, "Len to TX %d and seq num %d", skb->len, ntohl(pir->p_seq_num));
	return ret;
}

netdev_tx_t piradio0_tx(struct sk_buff *skb, struct net_device *dev)
{
	struct piradio_priv* priv = netdev_priv(dev);
	priv->link_stats.tx_packets++;
	piradio_prep_tx(skb, dev);
	return 0;
}

irqreturn_t tx_cmplt_callback(int irq, void *_dev){
	
	u32 status;
	unsigned long l_flags;
	struct axidma_bd *cur_p;
	struct net_device *dev = (struct net_device *)_dev;
	struct piradio_priv *priv = netdev_priv(dev);
	struct piradio_dma* dma_obj = &priv->tx_dma;
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
			if (++dma_obj->bd_curr >= priv->tx_bd_num)
				dma_obj->bd_curr = 0;
			cur_p = &dma_obj->bd_list[dma_obj->bd_curr];
			status = cur_p->status;
		}
		smp_mb();
		spin_lock_irqsave(&priv->tx_byte_cnt_lock, l_flags);
		if((priv->state_tx.remaining_bytes > 0) && netif_queue_stopped(dev)){
			spin_unlock_irqrestore(&priv->tx_byte_cnt_lock, l_flags);
			priv->tx_workqueue_item->netdev = dev;
			queue_work(priv->tx_workqueue, &priv->tx_workqueue_item->work);
		}
		else{
			spin_unlock_irqrestore(&priv->tx_byte_cnt_lock, l_flags);
			if(netif_queue_stopped(dev)){
				netif_wake_queue(dev);
			}
		}
		return IRQ_HANDLED;
	}
	if (!(status & XAXIDMA_IRQ_ALL_MASK)){
		printk(KERN_ALERT "TX Interrupt received no 2. %d ", status
						);
		return IRQ_NONE;
	}
	if (status & XAXIDMA_IRQ_ERROR_MASK) {
		cur_p = &dma_obj->bd_list[dma_obj->bd_curr];
		printk(KERN_ALERT "TX Interrupt received error %d %d %d", status, cur_p->cntrl, cur_p->status
								);
		return IRQ_NONE;
	}

	return IRQ_HANDLED;
}

int piradio_framer_ctrl(struct net_device *netdev, size_t reps)
{
	struct piradio_priv *priv;
    priv = netdev_priv(netdev);
	// if(priv->driver_state != DRIVER_READY)
	// 	return NETDEV_TX_BUSY;
    if(reps < 1)
		return -1;
	int i;
	unsigned long l_flags;
	int pad;
	int ret;
    uint32_t mod;
	dma_addr_t tail_base;
	struct piradio_dma *pdma = &priv->framer_ctrl_dma;
	struct axidma_bd *cur_p, *next_p;
	u32 nxt_p_offset;
	mod = (priv->modulation == 1) ? 1 :
		((priv->modulation == 2) ? 2 :
		((priv->modulation == 4) ? 3 :
		((priv->modulation == 6) ? 4 : 0)));
	for(i = 0; i < reps ; i++){
		cur_p = &pdma->bd_list[pdma->bd_tail];

		cur_p->phys = dma_map_single(priv->dev, priv->modulation_buf, //skb->data,
				sizeof(uint32_t), DMA_TO_DEVICE);
		cur_p->cntrl = (sizeof(uint32_t))| XAXIDMA_BD_CTRL_TXSOF_MASK | XAXIDMA_BD_CTRL_TXEOF_MASK;
		cur_p->tx_desc_mapping = 0;

		tail_base = pdma->bd_list_base + sizeof(*pdma->bd_list) * pdma->bd_tail;
		wmb();
		piradio_dma_bdout(pdma, XAXIDMA_TX_TDESC_OFFSET, tail_base);

		if (++pdma->bd_tail >= priv->ctrl_bd_num)
				pdma->bd_tail = 0;

		nxt_p_offset = (pdma->bd_tail + 1) % priv->ctrl_bd_num;
		next_p = &pdma->bd_list[nxt_p_offset];
		if(next_p->cntrl != 0){
			netdev_dbg(netdev, "Framer ctrl error");
			spin_unlock_irqrestore(&pdma->lock, l_flags);
			return -2;
		}
	}
	spin_unlock_irqrestore(&pdma->lock, l_flags);
	return 0;
error1:
	spin_unlock_irqrestore(&pdma->lock, l_flags);
	return -1;
}

irqreturn_t framer_ctrl_cmplt_callback(int irq, void *_dev){
	u32 status;
	struct axidma_bd *cur_p;
	struct net_device *dev = (struct net_device *)_dev;
	struct piradio_priv *priv = netdev_priv(dev);
	struct piradio_dma* dma_obj = &priv->framer_ctrl_dma;
	struct piradio_hdr *pir;
	struct sk_buff *skb;
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
			if (++dma_obj->bd_curr >= priv->ctrl_bd_num)
				dma_obj->bd_curr = 0;
			cur_p = &dma_obj->bd_list[dma_obj->bd_curr];
			status = cur_p->status;
		}
		smp_mb();
		priv->int_unack = false;
		return IRQ_HANDLED;
	}
	if (!(status & XAXIDMA_IRQ_ALL_MASK) && !priv->int_unack){
		printk(KERN_ALERT "TX Interrupt received no 2. %d ", status
						);
		priv->int_unack = true;
		return IRQ_NONE;
	}
	if ((status & XAXIDMA_IRQ_ERROR_MASK)) {
		printk(KERN_ALERT "TX Interrupt received error %d ", status
								);
		piradio_dma_write32(dma_obj, XAXIDMA_TX_SR_OFFSET, status);
		return IRQ_NONE;
	}

	return IRQ_HANDLED;
}

irqreturn_t tx_log_cmplt_callback(int irq, void *_dev){
	struct net_device *dev = (struct net_device *)_dev;
	struct piradio_priv *priv = netdev_priv(dev);
	dma_unmap_single(priv->dev, priv->phys_tx,
				priv->log_length,
				 DMA_FROM_DEVICE);
	netdev_dbg(dev, "Log TX completed");
	netdev_dbg(dev, "Log TX completed");
	return IRQ_HANDLED;
}