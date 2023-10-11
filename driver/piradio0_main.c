#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/socket.h>
#include <linux/errno.h>
#include <linux/fcntl.h>
#include <linux/in.h>
#include <linux/netdevice.h>
#include <linux/of_dma.h>
#include <linux/crc32.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/inet.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/ethtool.h>
#include <net/sock.h>
#include <net/checksum.h>
#include <linux/if_ether.h> /* For the statistics structure. */
#include <linux/if_arp.h> /* For ARPHRD_ETHER */
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/percpu.h>
#include <linux/net_tstamp.h>
#include <net/net_namespace.h>
#include <linux/u64_stats_sync.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/random.h>
#include <net/inet_ecn.h>
#include "piradio0.h"
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/kfifo.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>

#include <linux/module.h>

#include "piradio_util.h"
#include "piradio_tx.h"
#include "piradio_rx.h"
#include "piradio_config.h"
#include "piradio_probes.h"


MODULE_LICENSE("Dual BSD/GPL");

struct net_device * netdevg;
int project_type;

module_param(project_type, int, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(project_type, "Indicate type of project: 0 fir standalone, 1 for loopback");

static int piradio0_change_mtu(struct net_device *dev, int new_mtu){
	if (netif_running(dev))
		return -EBUSY;
	if(new_mtu <= PIRADIO_MAX_JUMBO_FRAME)
		dev->mtu = new_mtu;
	else return -EINVAL;

	return 0;
}

char test[8192];
static int piradio0_ioctl(struct net_device *dev, struct ifreq *rq, void __user *data,int cmd)
{
	netdev_dbg(dev, "IOCTL called");
	struct piradio_priv* priv = netdev_priv(dev);
	struct config_data conf;

	if(copy_from_user(&conf, (void*)rq->ifr_data, sizeof(struct config_data))){
		netdev_dbg(dev, "IOCTL failed");
		return 0;
	}
	netdev_dbg(dev, "Config mod %d %d", cmd, conf.length);
	switch (cmd) {
	case PIRADIO_CONFIG_MOD:
		if (conf.length) {
			priv->modulation = (conf.length == 1) ? MOD_BPSK :
								(conf.length == 2) ? MOD_QPSK :
								(conf.length == 3) ? MOD_QAM16 : MOD_BPSK;
			priv->tx_transfer_length = (priv->fec_struct.block_data_len / 8) * priv->modulation ;//: OFDM_FRAME_DATA_CARRIERS / (8 / priv->modulation); // TODO change when modulation changes
			priv->rx_transfer_length = (priv->fec_struct.block_data_len / 8) * priv->modulation;
			if(conf.length >= 1 && conf.length <= 3){
				write_reg(conf.length, priv->mod_rx_base);
				netdev_dbg(dev, "Wrote mod");
			}
			else
				write_reg(MOD_BPSK, priv->mod_rx_base);
			memcpy(priv->modulation_buf, &priv->modulation, sizeof(uint32_t));
			
			write_reg((priv->fec_struct.block_data_len * priv->modulation) / 128, 
						priv->rx_tlast_frame_len);
			
			piradio_dma_dealloc_tx_bds(&priv->tx_dma, priv->tx_bd_num, 
			dev);
			piradio_dma_dealloc_rx_bds(&priv->rx_dma,
					dev);
			piradio_soft_reset_dma(&priv->tx_dma, dev);
			piradio_soft_reset_dma(&priv->rx_dma, dev);
			piradio_dma_alloc_tx_bds(&priv->tx_dma, priv->tx_bd_num, 
					dev);
			piradio_dma_alloc_rx_bds(&priv->rx_dma,
					dev);
		}
		break;
	case PIRADIO_CONFIG_CORRELATOR:
		if (conf.length) {
			if(copy_from_user(test, (void*)conf.data, conf.length)){
				netdev_dbg(dev, "IOCTL failed");
				return 0;
			}
			printk(KERN_ALERT "IOCTL called PIRADIO_CONFIG_CORRELATOR\n", conf.length);
			piradio_config_core(test, conf.length, dev, CONFIG_CORRELATOR); // Locking?
		}
		break;
	case PIRADIO_CONFIG_FRAMER:
		if (conf.length) {
			if(copy_from_user(test, (void*)conf.data, conf.length)){
				netdev_dbg(dev, "IOCTL failed");
				return 0;
			}
			printk(KERN_ALERT "IOCTL called PIRADIO_CONFIG_FRAMER\n", conf.length);
			piradio_config_core(test, conf.length, dev, CONFIG_FRAMER); // Locking?
		}
		if(conf.length > 1024)
			priv->driver_state = DRIVER_READY;
		break;
	case PIRADIO_CONFIG_FFT_TX:
		if (conf.length) {
			memcpy(conf.data, (uint8_t*)priv->memtx_reg_base, priv->log_length);
		}
		break;
	case PIRADIO_CONFIG_FFT_RX:
		if (conf.length) {
			copy_to_user(conf.data, (uint8_t*)priv->mem_reg_base, priv->log_length);
			//memcpy(conf.data, (uint8_t*)priv->mem_reg_base, priv->log_length);
		}
		break;
	case PIRADIO_CONFIG_FIR:
		config_log(dev);
		break;
	case PIRADIO_CONFIG_FREQ_OFF:
		write_reg(conf.length, priv->ifft_reg_base);
		break;
	case PIRADIO_CONFIG_STHRESH:
		priv->driver_state = DRIVER_READY;
//		write_reg(80000, piradio_p->sync_thresh_base);
		return 0;
	case PIRADIO_CONFIG_CSMA:
//		if (conf->length) {
//			write_reg(conf->length, piradio_p->csma_delay_base);
//		}
		break;
	case PIRADIO_CONFIG_ACK:
		printk(KERN_ALERT "Ack enabled %d", conf.length);
		if (conf.length) {
			priv->ack_enabled = 1;
		}
		else{
			priv->ack_enabled = 0;
		}
		break;
	case CONFIG_TRANSFER_LEN:
		priv->tx_transfer_length = conf.length;
		break;
	}
	return 0;
}

static void piradio0_stats(struct net_device *dev, struct rtnl_link_stats64 *stats)
{
	struct piradio_priv* priv = netdev_priv(dev);
	stats->collisions = priv->link_stats.retransmissions;
	stats->tx_packets = priv->link_stats.tx_packets;
	stats->tx_bytes = priv->link_stats.tx_bytes;
	stats->tx_errors = priv->link_stats.tx_errors;
	stats->tx_dropped = priv->link_stats.tx_dropped;
	stats->rx_bytes = priv->link_stats.rx_bytes;
	stats->rx_errors = priv->link_stats.rx_errors;
	stats->rx_packets = priv->link_stats.rx_packets;
	stats->rx_dropped = priv->link_stats.rx_dropped;
	stats->rx_crc_errors = priv->link_stats.crc_rx_errors;
	stats->rx_missed_errors = priv->link_stats.rx_missed_errors;
}

int piradio0_rebuild_header(struct net_device *dev)
{
	return 0;
}

int piradio0_set_mac(struct net_device *dev, void *addr)
{
	return 0;
}

int piradio0_dev_init(struct net_device *dev)
{
	printk(KERN_ALERT "PIRADIO0 INIT");
	printk(KERN_ALERT "PIRADIO0 INIT");
	return 0;
}

int piradio0_open(struct net_device *dev)
{
	struct piradio_priv *priv = netdev_priv(dev);
	int ret;
	uint32_t fec_ctrl_reg;
	netif_start_queue(dev);
	
	
	printk(KERN_ALERT "PIRADIO0 OPEN");
	printk(KERN_ALERT "PIRADIO0 OPEN");
	priv->link_stats.retransmissions = 0;
	priv->link_stats.rx_bytes = 0;
	priv->link_stats.rx_dropped = 0;
	priv->link_stats.rx_errors = 0;
	priv->link_stats.rx_packets = 0;
	priv->link_stats.tx_bytes = 0;
	priv->link_stats.tx_dropped = 0;
	priv->link_stats.tx_errors = 0;
	priv->link_stats.tx_packets = 0;
	priv->link_stats.rx_missed_errors = 0;
	priv->link_stats.crc_rx_errors = 0;
	priv->tx_tries = 0;
	priv->csma_tries = 0;
	priv->sequence_num = 0;
	priv->ack_enabled = 1;
	priv->tx_bd_num = 64;
	priv->ctrl_bd_num = 64;
	priv->tx_bd_config_num = 3;
	priv->rx_bd_num = 64 * 2;
	priv->coalesce_count_tx = 24;
	priv->coalesce_count_rx = 1;
	priv->modulation = MOD_BPSK;
	priv->log_length = 10 * 10 * 1280 * 4;

	priv->fec_struct.fec_hdr.z_j = 6;
	priv->fec_struct.fec_hdr.z_set = 2;
	priv->fec_struct.fec_hdr.hard_op = 1;
	priv->fec_struct.fec_hdr.bg = 0;
	priv->fec_struct.fec_hdr.mb = 8;
	priv->fec_struct.fec_hdr.max_it = 8;
	priv->fec_struct.block_data_len = 7040;
	priv->fec_struct.block_len = 9600;

	priv->tx_transfer_length = (priv->fec_struct.block_data_len / 8) * priv->modulation ;//: OFDM_FRAME_DATA_CARRIERS / (8 / priv->modulation); // TODO change when modulation changes
	priv->rx_transfer_length = (priv->fec_struct.block_data_len / 8) * priv->modulation;// OFDM_FRAME_DATA_CARRIERS / (8 / priv->modulation)  : 528 ;
	priv->driver_state = DRIVER_IDLE;
	priv->int_unack = false;

	napi_enable(&priv->napi);
	ret = request_irq(priv->tx_dma.irq, tx_cmplt_callback,
						  0, dev->name, dev);
	ret = request_irq(priv->rx_dma.irq, rx_cmplt_callback0,
						  0, dev->name, dev);
	ret = request_irq(priv->framer_ctrl_dma.irq, framer_ctrl_cmplt_callback,
						  0, dev->name, dev);
	netdev_dbg(dev, "Will Allocate BDs");
	piradio_dma_alloc_tx_bds(&priv->tx_dma, priv->tx_bd_num, 
			dev);
	netdev_dbg(dev, "Will Allocate RX BDs");
	piradio_dma_alloc_rx_bds(&priv->rx_dma,
			dev);
	if(priv->main_iface){ 
		memcpy(dev->dev_addr, "PIRAD0", PIRADIO_HW_ALEN);
		ret = request_irq(priv->config_framer_dma.irq, tx_config_cmplt_callback, // TODO
						0, dev->name, dev);
		ret = request_irq(priv->config_correlator_dma.irq, tx_config_cmplt_callback, // TODO
						0, dev->name, dev);
		netdev_dbg(dev, "Will Allocate TX BDs");
		piradio_dma_alloc_tx_bds(&priv->config_framer_dma, priv->tx_bd_config_num,
			dev);
		netdev_dbg(dev, "Will Allocate TX BDs");
		piradio_dma_alloc_tx_bds(&priv->config_correlator_dma, priv->tx_bd_config_num,
			dev);
		
	}
	else{
		memcpy(dev->dev_addr, "PIRAD0", PIRADIO_HW_ALEN);
		
	}
	netdev_dbg(dev, "Will Allocate BDs");
	piradio_dma_alloc_tx_bds(&priv->framer_ctrl_dma, priv->ctrl_bd_num, 
			dev);
	netdev_dbg(dev, "Complete");
	priv->buff = (__u8*)kmalloc(PIRADIO_MAX_DMA_TRANSF_LEN, GFP_KERNEL);
	priv->buff_tmp = (__u8*)kmalloc(PIRADIO_MAX_DMA_TRANSF_LEN, GFP_KERNEL);
	priv->tx_log_buff = (__u8*)kmalloc(priv->log_length, GFP_KERNEL);
	priv->rx_log_buff = (__u8*)kmalloc(priv->log_length, GFP_KERNEL);
	priv->modulation_buf = (__u8*)kmalloc(sizeof(uint32_t), GFP_KERNEL);
	priv->rx_buffer = (__u8*)kmalloc(PIRADIO_MAX_JUMBO_FRAME, GFP_KERNEL);
	priv->state_rx.state = PIR_RX_IDLE;
	priv->state_rx.storred_bytes = 0;
	memcpy(priv->modulation_buf, &priv->modulation, sizeof(uint32_t));
	get_random_bytes(priv->buff, PIRADIO_MAX_DMA_TRANSF_LEN);

	// memcpy(priv->fec_tx_ctrl, &priv->fec_struct.fec_hdr, 2 * sizeof(uint32_t));
	// memcpy(priv->fec_rx_ctrl, &priv->fec_struct.fec_hdr, 2 * sizeof(uint32_t));

	write_reg(priv->modulation, priv->mod_rx_base);
	write_reg(0x01, priv->fec_tx_base + 0x14);
	write_reg(0x01, priv->fec_rx_base + 0x14);

	write_reg(0x3F, priv->fec_tx_base + 0x10);
	write_reg(0x3F, priv->fec_rx_base + 0x10);
	fec_ctrl_reg = priv->fec_struct.fec_hdr.z_j |
					(priv->fec_struct.fec_hdr.z_set << 3) |
					(priv->fec_struct.fec_hdr.bg << 6) |
					(priv->fec_struct.fec_hdr.norm << 9) |
					(priv->fec_struct.fec_hdr.rsvd << 13) |
					(priv->fec_struct.fec_hdr.hard_op << 14) |
					(priv->fec_struct.fec_hdr.inc_par << 15) |
					(priv->fec_struct.fec_hdr.top << 16) |
					(priv->fec_struct.fec_hdr.tnc << 17) |
					(priv->fec_struct.fec_hdr.max_it << 18) |
					(priv->fec_struct.fec_hdr.id << 24) ;
	write_reg(fec_ctrl_reg, priv->fec_tx_ctrl_base);
	write_reg(fec_ctrl_reg, priv->fec_rx_ctrl_base);
	fec_ctrl_reg = priv->fec_struct.fec_hdr.mb |
					(priv->fec_struct.fec_hdr.msch << 6) |
					(priv->fec_struct.fec_hdr.rsvd2 << 8);
	write_reg(fec_ctrl_reg, priv->fec_tx_ctrl_base + 0x4);
	write_reg(fec_ctrl_reg, priv->fec_rx_ctrl_base + 0x4);

	write_reg(priv->fec_struct.block_data_len, priv->fec_tx_ctrl_base + 0x8);
	write_reg(priv->fec_struct.block_len, priv->fec_rx_ctrl_base + 0x8);
	write_reg(0x1, priv->fec_tx_ctrl_base + 0xC);
	write_reg(0x1, priv->fec_rx_ctrl_base + 0xC);
	write_reg((priv->fec_struct.block_data_len * priv->modulation) / 128, 
						priv->rx_tlast_frame_len); // CHANGE IF RX DATA WIDTH CHANGES !TODO!

	priv->tx_workqueue = create_workqueue("tx_workqueue");
	priv->tx_workqueue_item = kmalloc(sizeof(struct tx_wq_item), GFP_ATOMIC);
	INIT_WORK(&priv->tx_workqueue_item->work, tx_workqueue_fn);
	priv->project_type = (project_t)project_type;

	return 0;
}



static int piradio0_down(struct net_device *dev)
{
	struct piradio_priv *priv; 
	priv = netdev_priv(dev);
	netdev_dbg(dev, "In down");
	netif_stop_queue(dev);
	napi_disable(&priv->napi);

	free_irq(priv->tx_dma.irq, dev);
	free_irq(priv->rx_dma.irq, dev);
	free_irq(priv->framer_ctrl_dma.irq, dev);
		
	kfree(priv->buff);
	kfree(priv->tx_log_buff);
	kfree(priv->rx_log_buff);
	kfree(priv->modulation_buf);
	// kfree(priv->fec_rx_ctrl);
	// kfree(priv->fec_tx_ctrl);
	piradio_dma_dealloc_tx_bds(&priv->tx_dma, priv->tx_bd_num, 
			dev);
	piradio_dma_dealloc_tx_bds(&priv->framer_ctrl_dma, priv->ctrl_bd_num, 
			dev);
	piradio_dma_dealloc_rx_bds(&priv->rx_dma,
			dev);
	piradio_soft_reset_dma(&priv->tx_dma, dev);
	piradio_soft_reset_dma(&priv->framer_ctrl_dma, dev);
	piradio_soft_reset_dma(&priv->rx_dma, dev);
	if(priv->main_iface){
		free_irq(priv->config_framer_dma.irq, dev);
		free_irq(priv->config_correlator_dma.irq, dev);
		piradio_dma_dealloc_tx_bds(&priv->config_framer_dma, priv->tx_bd_config_num,
				dev);
		piradio_dma_dealloc_tx_bds(&priv->config_correlator_dma, priv->tx_bd_config_num,
				dev);
		piradio_soft_reset_dma(&priv->config_framer_dma, dev);
		piradio_soft_reset_dma(&priv->config_correlator_dma, dev);
	}
	return 0;
}

void ret_0(struct net_device *dev)
{
	return;
}

int piradio0_header(struct sk_buff *skb, struct net_device *dev,
		   unsigned short type, const void *daddr, const void *saddr,
		   unsigned int len)
{
	struct ethhdr *eth = skb_push(skb, ETH_HLEN);
	if (type != ETH_P_802_3 && type != ETH_P_802_2)
		eth->h_proto = htons(type);
	else
		eth->h_proto = htons(len);

	if (!saddr)
		saddr = dev->dev_addr;
	memcpy(eth->h_source, saddr, ETH_ALEN);

	if (daddr) {
			memcpy(eth->h_dest, dev->dev_addr,
			       PIRADIO_HW_ALEN);
	}
	//netdev_dbg(dev, "Size = %d ", skb->len);
	return ETH_HLEN;
}

const struct header_ops header_ops ____cacheline_aligned = {
	.create = piradio0_header,
	//	.parse		= ret_0,
	//	.cache		= ret_0,
	//	.cache_update	= ret_0,
	//	.parse_protocol	= ret_0,
};

static const struct net_device_ops piradio0_ops = {
	.ndo_open = piradio0_open,
	.ndo_stop = piradio0_down,
	.ndo_init = piradio0_dev_init,
	.ndo_start_xmit = piradio0_tx,
	.ndo_siocdevprivate = piradio0_ioctl,
	.ndo_get_stats64 = piradio0_stats,
	.ndo_change_mtu = piradio0_change_mtu,
};

void piradio0_setup(struct net_device *dev)
{
	ether_setup(dev);
	dev->mtu = 1000;
	dev->max_mtu = 64000;
	dev->hard_header_len = PIRADIO_HDR_LEN + ETH_HLEN; /* 14	*/
	dev->min_header_len = PIRADIO_HDR_LEN + ETH_HLEN; /* 14	*/
	dev->addr_len = PIRADIO_HW_ALEN; /* 6	*/
	dev->flags = IFF_NOARP | IFF_DEBUG | IFF_BROADCAST | IFF_MULTICAST;
	//	dev->priv_flags |= IFF_LIVE_ADDR_CHANGE | IFF_NO_QUEUE ;
	dev->priv_flags |= IFF_TX_SKB_SHARING;
	//	netif_keep_dst(dev);
	//	dev->hw_features = NETIF_F_GSO_SOFTWARE;
	//	dev->features = NETIF_F_SG | NETIF_F_FRAGLIST | NETIF_F_GSO_SOFTWARE |
	//			NETIF_F_NO_CSUM | NETIF_F_RXCSUM | NETIF_F_SCTP_CRC |
	//			NETIF_F_HIGHDMA | NETIF_F_LLTX | NETIF_F_NETNS_LOCAL |
	//			NETIF_F_VLAN_CHALLENGED;
	//	dev->ethtool_ops	= &piradio_ethtool_ops;
	dev->header_ops = &header_ops;
	dev->netdev_ops = &piradio0_ops;
	dev->needs_free_netdev = 1;
	dev->priv_destructor = ret_0;
}

static int piradio0_probe(struct platform_device *pdev)
{
	printk(KERN_ALERT "piradio0 probed\n");
	int err;
	bool iface_main;
	struct net_device *netdev = alloc_netdev(sizeof(struct piradio_priv), "piradio0%d", NET_NAME_UNKNOWN, piradio0_setup);
	printk(KERN_ALERT "%s\n", pdev->name);
	printk(KERN_ALERT "%s\n", pdev->name);
	SET_NETDEV_DEV(netdev, &pdev->dev);
	
	platform_set_drvdata(pdev, netdev);

	struct piradio_priv *priv = netdev_priv(netdev);

	iface_main = of_property_read_bool(pdev->dev.of_node, "iface-main");
	if(iface_main){
		priv->main_iface = true;
		piradio_dma_probe(pdev,
			&priv->tx_dma, 0);
		piradio_dma_probe(pdev,
			&priv->config_framer_dma, 1);
		piradio_dma_probe(pdev,
			&priv->config_correlator_dma, 2);
		piradio_dma_probe(pdev,
				&priv->rx_dma, 3);
		piradio_dma_probe(pdev,
			&priv->framer_ctrl_dma, 4);
	}
	else{
		priv->main_iface = false;
		piradio_dma_probe(pdev,
			&priv->tx_dma, 0);
		piradio_dma_probe(pdev,
			&priv->rx_dma, 1);
		piradio_dma_probe(pdev,
			&priv->framer_ctrl_dma, 2);
	}

	//piradio_mod_reg_probe(pdev, &priv->mod_rx_base);
	piradio_reg_probe(pdev, &priv->mod_rx_base, "mod_reg", 0);
	piradio_reg_probe(pdev, &priv->fec_tx_base, "fec_tx_reg", 0);
	piradio_reg_probe(pdev, &priv->fec_tx_ctrl_base, "fec_tx_reg", 1);
	piradio_reg_probe(pdev, &priv->fec_rx_base, "fec_rx_reg", 0);
	piradio_reg_probe(pdev, &priv->fec_rx_ctrl_base, "fec_rx_reg", 1);
	piradio_reg_probe(pdev, &priv->rx_tlast_frame_len, "tlast_reg", 0);
	
	//piradio_reg_probe(pdev, &priv->mem_reg_base,"log_reg", 0);
	// piradio_ifft_reg_probe(pdev, &priv->ifft_reg_base);
	priv->dev = &pdev->dev;
	priv->pdev = pdev;
	netif_napi_add(netdev, &priv->napi, piradio_rx_poll,
				       PIRADIO_NAPI_WEIGHT);
	err = register_netdev(netdev);
	
	printk(KERN_INFO "dma_proxy module 0 init ok\n");
	return 0;
}

static int piradio0_remove(struct platform_device *pdev)
{
	struct net_device *ndev;
	struct piradio_priv *priv;
	ndev = platform_get_drvdata(pdev);

	if(ndev){
		netdev_dbg(ndev, "In remove");
		priv = netdev_priv(ndev);
		netif_napi_del(&priv->napi);
		netif_tx_stop_all_queues(ndev);
		unregister_netdev(ndev);
		//free_netdev(ndev);
	}
	else
		netdev_dbg(ndev, "In remove but failed");
	return 0;
}

static const struct of_device_id piradio0_of_ids[] = {
	{
		.compatible = "xlnx,piradio0",
	},
	{}
};

static struct platform_driver piradio0_driver =
  { .driver =
    { .name = "piradio0_driver", .owner = THIS_MODULE, .of_match_table =
	piradio0_of_ids, }, .probe = piradio0_probe, .remove = piradio0_remove, };

static int __init piradio0_init(void)
{
	if(project_type)
		printk(KERN_ALERT "Hello World for loopback project\n");
	else	
		printk(KERN_ALERT "Hello World for standalone project\n");
	
	return platform_driver_register(&piradio0_driver);
}

static void piradio0_exit(void)
{

	platform_driver_unregister(&piradio0_driver);
	// struct net_device *dev;
	// struct piradio_priv *priv;
	// dev = platform_get_drvdata(pdev);
	// priv = netdev_priv(dev);

	// if (dev) {
	// 	printk(KERN_ALERT "DEV removed OK\n");
	// 	netif_stop_queue(dev);
	// 	unregister_netdev(dev);
	// 	free_netdev(dev);
	// 	napi_disable(&priv->napi);
	// }
	// else{
	// 	printk(KERN_ALERT "DEV NOT removed OK\n");
	// }	
	printk(KERN_ALERT "Bye bye World1\n");
}

module_init(piradio0_init);
module_exit(piradio0_exit);
