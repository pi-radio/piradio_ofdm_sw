#include "piradio_rx.h"
#include "piradio_tx.h"
#include "piradio_util.h"
#include <linux/ip.h>
#include <linux/crc32.h>
#include <linux/etherdevice.h>
#include <linux/module.h>

MODULE_LICENSE("Dual BSD/GPL");

int prepare_skb(uint8_t *rx_buffer, size_t len,
							struct net_device *netdev){
	struct piradio_priv *priv;
    priv = netdev_priv(netdev);
	struct sk_buff* skb;
	int ret;
	skb = netdev_alloc_skb(netdev, len);
	skb_put(skb, len);
	
	struct iphdr *ip_header;
	ip_header = (struct iphdr *)&rx_buffer[ETH_HLEN];
	if (ip_header->ihl == 5 && ip_header->version == 4) {
	ip_header->daddr ^=
		0x00010000; /*Flip last bit of network address to fool the OS*/
	ip_header->saddr ^=
		0x00010000; /*Flip last bit of host address to fool the OS*/
	ip_header->check = 0;
	ip_header->check =
		ip_fast_csum((u8 *)ip_header, ip_header->ihl);
	memcpy(skb->data, rx_buffer, len);
	skb->protocol = eth_type_trans(skb, netdev);
	skb->ip_summed = CHECKSUM_UNNECESSARY;
	
	// netdev_dbg(dev, "Sent upstream %d %d %d %d %d",
	//  (skb->pkt_type == PACKET_OTHERHOST), 
	//  (ip_fast_csum((u8 *)ip_header, ip_header->ihl), skb->len, ntohs(ip_header->tot_len),
		//ip_header->ihl*4));
	ret = netif_receive_skb(skb);
	//netdev_dbg(dev, "Sent upstream %d",ret);
	if(ret == NET_RX_DROP){
		priv->link_stats.rx_dropped++;
		netdev_dbg(netdev, "RX drop");
	}
	else if(ret == NET_RX_SUCCESS)
		priv->link_stats.rx_packets++;
	}
	else{
		printk(KERN_ALERT "Invalid header ");
	}
	return ret;
}

int piradio_rx(struct net_device *dev, int budget){
	dma_addr_t tail_p = 0;
	u32 status;
	int ret;
	struct piradio_priv *priv = netdev_priv(dev);
	struct axidma_bd *cur_p;
	u32 csum = 0;
	struct piradio_dma* dma_obj = &priv->rx_dma;
	struct piradio_hdr *pir;
	struct sk_buff *skb, *new_skb;
	struct iphdr *ip_header;
	size_t rx_len;
	int pkts_processed = 0;
	rmb();
	cur_p = &dma_obj->bd_list[dma_obj->bd_curr];
	status = piradio_dma_read32(dma_obj, XAXIDMA_RX_SR_OFFSET);
	piradio_dma_write32(dma_obj, XAXIDMA_RX_SR_OFFSET, status);
	while ( (pkts_processed < budget) &&
			(cur_p->status & XAXIDMA_BD_STS_COMPLETE_MASK)) {
//		netdev_dbg(dev, "Received = %d", cur_p->status & 0x03ffffff);
				
		tail_p = dma_obj->bd_list_base + sizeof(*dma_obj->bd_list) * dma_obj->bd_curr;
		dma_unmap_single(priv->dev, cur_p->phys,
				priv->rx_transfer_length,
				 DMA_FROM_DEVICE);

		new_skb = netdev_alloc_skb(dev, priv->rx_transfer_length );
		if (!new_skb) {
			printk(KERN_ALERT "No mem for skb ");
			break;
		}

		skb = (struct sk_buff *)(cur_p->sw_id_offset);
		pir = (struct piradio_hdr *)(skb->data);
		
		rx_len = ntohl(pir->p_len);
		if( rx_len <= (priv->rx_transfer_length - PIRADIO_HDR_LEN)){
			skb_put(skb, rx_len + PIRADIO_HDR_LEN);
			//skb_put(skb, ntohl(pir->p_len) + PIRADIO_HDR_LEN); TODO
			skb_pull(skb, PIRADIO_HDR_LEN);
			csum = ~crc32_le(~0, skb->data,
					skb->len);
			if(pir->p_csum != csum){
				priv->link_stats.crc_rx_errors++;
				netdev_dbg(dev, "RX CSUM error %d", rx_len /*ntohl(pir->p_len)*/);
			}
			switch(priv->state_rx.state){
				case PIR_RX_IDLE :
					if(pir->p_type & PIR_T_FRAME_START){
						memcpy(priv->rx_buffer, skb->data, skb->len);
						if(pir->p_type & PIR_T_FRAME_END){
							prepare_skb(priv->rx_buffer, skb->len, dev);
							priv->state_rx.state = PIR_RX_IDLE;
							priv->state_rx.storred_bytes = 0;
						}
						else{
							priv->state_rx.state = PIR_RX_OPEN;
							priv->state_rx.storred_bytes = skb->len;
						}
					}
					else{
						priv->state_rx.storred_bytes = 0;
					}
					break;
				case PIR_RX_OPEN:
					if((pir->p_type & PIR_T_FRAME_START) && 
											(pir->p_type & PIR_T_FRAME_END)){
						memcpy(priv->rx_buffer, skb->data, skb->len);
						prepare_skb(priv->rx_buffer, priv->state_rx.storred_bytes, dev);
						priv->state_rx.state = PIR_RX_IDLE;
						priv->state_rx.storred_bytes = 0;
					}
					else{
						if(pir->p_type & PIR_T_FRAME_START){
							memcpy(priv->rx_buffer, skb->data, skb->len);
							priv->state_rx.storred_bytes = skb->len;
						}
						if(pir->p_type & PIR_T_FRAME_CONT){
							memcpy(&priv->rx_buffer[priv->state_rx.storred_bytes],
										 skb->data, skb->len);
							priv->state_rx.storred_bytes += skb->len;
							if(priv->state_rx.storred_bytes >= PIRADIO_MAX_JUMBO_FRAME){
								priv->state_rx.state = PIR_RX_IDLE;
								priv->state_rx.storred_bytes = 0;
							}
						}
						if(pir->p_type & PIR_T_FRAME_END){
							if(priv->state_rx.storred_bytes + skb->len > PIRADIO_MAX_JUMBO_FRAME){
								priv->state_rx.state = PIR_RX_IDLE;
								priv->state_rx.storred_bytes = 0;
								break;
							}
							memcpy(&priv->rx_buffer[priv->state_rx.storred_bytes],
										 skb->data, skb->len);
							priv->state_rx.storred_bytes += skb->len;
							prepare_skb(priv->rx_buffer, priv->state_rx.storred_bytes, dev);
							priv->state_rx.state = PIR_RX_IDLE;
							priv->state_rx.storred_bytes = 0;
						}
					}
					break;
				default:
					break;
			}
		}
		else{
			netdev_dbg(dev, "RX Invalid length %u", ntohl(pir->p_len));
		}
		dev_kfree_skb_irq((struct sk_buff *)skb);
		wmb();
		cur_p->phys = dma_map_single(priv->dev, new_skb->data,
						priv->rx_transfer_length,
						DMA_FROM_DEVICE);
		cur_p->cntrl = priv->rx_transfer_length;
		cur_p->status = 0;
		cur_p->sw_id_offset = (phys_addr_t)new_skb;
		if (++dma_obj->bd_curr >= priv->rx_bd_num)
			dma_obj->bd_curr = 0;

		rmb();
		cur_p = &dma_obj->bd_list[dma_obj->bd_curr];
		pkts_processed++;
	}
	if (tail_p) {
		piradio_dma_bdout(dma_obj, XAXIDMA_RX_TDESC_OFFSET, tail_p);
	}
	return pkts_processed;
}

int piradio_rx_poll(struct napi_struct *napi, int quota)
{
	u32 status,control, pkts = 0;
	struct net_device *dev = napi->dev;
	struct piradio_priv *priv = netdev_priv(dev);
	struct piradio_dma* dma_obj = &priv->rx_dma;
	//spin_lock(&dma_obj->lock);
	status = piradio_dma_read32(dma_obj, XAXIDMA_RX_SR_OFFSET);
	while ((status & (XAXIDMA_IRQ_IOC_MASK | XAXIDMA_IRQ_DELAY_MASK)) &&
		       (pkts < quota)) {
		piradio_dma_write32(dma_obj, XAXIDMA_RX_SR_OFFSET, status);
		pkts += piradio_rx(dev, quota - pkts);
		status = piradio_dma_read32(dma_obj, XAXIDMA_RX_SR_OFFSET);
	}
	if(pkts < quota){
		napi_complete(napi);
		control = piradio_dma_read32(dma_obj, XAXIDMA_RX_CR_OFFSET);
		control |= (XAXIDMA_IRQ_IOC_MASK | XAXIDMA_IRQ_DELAY_MASK);
		piradio_dma_write32(dma_obj, XAXIDMA_RX_CR_OFFSET, control);
		control = piradio_dma_read32(dma_obj, XAXIDMA_RX_SR_OFFSET);
		if((status & (XAXIDMA_IRQ_IOC_MASK | XAXIDMA_IRQ_DELAY_MASK))){
			napi_reschedule(napi);
			control = piradio_dma_read32(dma_obj, XAXIDMA_RX_CR_OFFSET);
			control &= ~(XAXIDMA_IRQ_IOC_MASK | XAXIDMA_IRQ_DELAY_MASK);
			piradio_dma_write32(dma_obj, XAXIDMA_RX_CR_OFFSET, control);
		}
	}
	//spin_unlock(&dma_obj->lock);
	return pkts;
}

irqreturn_t rx_cmplt_callback0(int irq, void *_dev){
	u32 status,control;
	struct net_device *dev = (struct net_device *)_dev;
	struct piradio_priv *priv = netdev_priv(dev);
	struct piradio_dma* dma_obj = &priv->rx_dma;
	status = piradio_dma_read32(dma_obj, XAXIDMA_RX_SR_OFFSET);
	if (status & (XAXIDMA_IRQ_IOC_MASK | XAXIDMA_IRQ_DELAY_MASK)) {
		control = piradio_dma_read32(dma_obj, XAXIDMA_RX_CR_OFFSET);
		control &= ~(XAXIDMA_IRQ_IOC_MASK | XAXIDMA_IRQ_DELAY_MASK);
		piradio_dma_write32(dma_obj, XAXIDMA_RX_CR_OFFSET, control);
		napi_schedule(&priv->napi);
	}
	if (!(status & XAXIDMA_IRQ_ALL_MASK)){
		netdev_dbg(priv->netdev,"Error 1");
		return IRQ_NONE;
	}

	if (status & XAXIDMA_IRQ_ERROR_MASK) {
		netdev_dbg(priv->netdev,"Error 2");
		return IRQ_HANDLED;
	}
	return IRQ_HANDLED;
}
irqreturn_t rx_log_cmplt_callback(int irq, void *_dev){
	struct net_device *dev = (struct net_device *)_dev;
	struct piradio_priv *priv = netdev_priv(dev);
	dma_unmap_single(priv->dev, priv->phys_rx,
				priv->log_length,
				 DMA_FROM_DEVICE);
	netdev_dbg(dev, "Log RX completed");
	netdev_dbg(dev, "Log RX completed");
	return IRQ_HANDLED;
}