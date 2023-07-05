/*
 * piradio.h
 *
 *  Created on: Sep 22, 2021
 *      Author: george
 */

#ifndef PIRADIO0_H_
#define PIRADIO0_H_

#include <linux/spinlock.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/of_dma.h>
#include <linux/types.h>
#include <linux/sched/task.h>
#include <linux/sockios.h>
#include <linux/kfifo.h>
#include <linux/netdevice.h>
#include <linux/workqueue.h>

#define XIL_CHANN_EMULATOR0          0xA0060000
#define XIL_FIR_FILTER0              0xA0050000
#define XIL_MODULATOR0               0xA0030000
#define XIL_CSMA_DELAY0              0xA00A0000
#define XIL_CSMA_STATUS0             0xA00B0000
#define XIL_SYNC_THRESH0             0xB0080000

#define XIL_CHANN_EMULATOR1          0xB0000000
#define XIL_FIR_FILTER1              0xB0010000
#define XIL_MODULATOR1               0xA00E0000
#define XIL_CSMA_DELAY1              0xB0050000
#define XIL_CSMA_STATUS1             0xB0060000
#define XIL_SYNC_THRESH1             0xA0180000

#define XIL_RESET                    0xFF0A0054


#define PIRADIO_SENT_QUEUE_LENGTH            128
#define PIRADIO_PENDING_QUEUE_LENGTH         128
#define PIRADIO_WAIT_QUEUE_LENGTH            2
#define PIRADIO_MAX_CSMA_TRIES               10   /*Maximum number of attempts to restart CSMA timer if channel always occupied*/
#define PIRADIO_MAX_TX_TRIES                 10   /*Maximum number of attempts to transmit if no ACK was received*/
#define PIRADIO_CSMA_THRESHOLD               1000000
#define PIRADIO_CSMA_BACKOFF_MAX             100000 /* Maximum nanoseconds to wait for CSMA engine recheck*/
#define PIRADIO_ACK_HOLDOFF                  700000 /* Maximum nanoseconds to wait for ACK arrival */

#define PIRADIO_HW_ALEN                      6
#define PIRADIO_HDR_LEN                      sizeof(struct piradio_hdr)

#define PIRADIO_ACK_TYPE                     0x2000
#define PIRADIO_SYNC_THRESHOLD               0x61a8 //25000

#define XAXIDMA_TX_CR_OFFSET	0x00000000 /* Channel control */
#define XAXIDMA_TX_SR_OFFSET	0x00000004 /* Status */
#define XAXIDMA_TX_CDESC_OFFSET	0x00000008 /* Current descriptor pointer */
#define XAXIDMA_TX_TDESC_OFFSET	0x00000010 /* Tail descriptor pointer */

#define XAXIDMA_RX_CR_OFFSET	0x00000030 /* Channel control */
#define XAXIDMA_RX_SR_OFFSET	0x00000034 /* Status */
#define XAXIDMA_RX_CDESC_OFFSET	0x00000038 /* Current descriptor pointer */
#define XAXIDMA_RX_TDESC_OFFSET	0x00000040 /* Tail descriptor pointer */

#define XAXIDMA_CR_RUNSTOP_MASK	0x00000001 /* Start/stop DMA channel */
#define XAXIDMA_CR_RESET_MASK	0x00000004 /* Reset DMA engine */

#define XAXIDMA_SR_HALT_MASK	0x00000001 /* Indicates DMA channel halted */

#define XAXIDMA_BD_NDESC_OFFSET		0x00 /* Next descriptor pointer */
#define XAXIDMA_BD_BUFA_OFFSET		0x08 /* Buffer address */
#define XAXIDMA_BD_CTRL_LEN_OFFSET	0x18 /* Control/buffer length */
#define XAXIDMA_BD_STS_OFFSET		0x1C /* Status */
#define XAXIDMA_BD_USR0_OFFSET		0x20 /* User IP specific word0 */
#define XAXIDMA_BD_USR1_OFFSET		0x24 /* User IP specific word1 */
#define XAXIDMA_BD_USR2_OFFSET		0x28 /* User IP specific word2 */
#define XAXIDMA_BD_USR3_OFFSET		0x2C /* User IP specific word3 */
#define XAXIDMA_BD_USR4_OFFSET		0x30 /* User IP specific word4 */
#define XAXIDMA_BD_ID_OFFSET		0x34 /* Sw ID */
#define XAXIDMA_BD_HAS_STSCNTRL_OFFSET	0x38 /* Whether has stscntrl strm */
#define XAXIDMA_BD_HAS_DRE_OFFSET	0x3C /* Whether has DRE */

#define XAXIDMA_BD_HAS_DRE_SHIFT	8 /* Whether has DRE shift */
#define XAXIDMA_BD_HAS_DRE_MASK		0xF00 /* Whether has DRE mask */
#define XAXIDMA_BD_WORDLEN_MASK		0xFF /* Whether has DRE mask */

#define XAXIDMA_BD_CTRL_LENGTH_MASK	0x007FFFFF /* Requested len */
#define XAXIDMA_BD_CTRL_TXSOF_MASK	0x08000000 /* First tx packet */
#define XAXIDMA_BD_CTRL_TXEOF_MASK	0x04000000 /* Last tx packet */
#define XAXIDMA_BD_CTRL_ALL_MASK	0x0C000000 /* All control bits */

#define XAXIDMA_DELAY_MASK		0xFF000000 /* Delay timeout counter */
#define XAXIDMA_COALESCE_MASK		0x00FF0000 /* Coalesce counter */

#define XAXIDMA_DELAY_SHIFT		24
#define XAXIDMA_COALESCE_SHIFT		16

#define XAXIDMA_IRQ_IOC_MASK		0x00001000 /* Completion intr */
#define XAXIDMA_IRQ_DELAY_MASK		0x00002000 /* Delay interrupt */
#define XAXIDMA_IRQ_ERROR_MASK		0x00004000 /* Error interrupt */
#define XAXIDMA_IRQ_ALL_MASK		0x00007000 /* All interrupts */

/* Default TX/RX Threshold and waitbound values for SGDMA mode */
#define XAXIDMA_DFT_TX_THRESHOLD	24
#define XAXIDMA_DFT_TX_WAITBOUND	254
#define XAXIDMA_DFT_RX_THRESHOLD	1
#define XAXIDMA_DFT_RX_WAITBOUND	254

#define XAXIDMA_BD_CTRL_TXSOF_MASK	0x08000000 /* First tx packet */
#define XAXIDMA_BD_CTRL_TXEOF_MASK	0x04000000 /* Last tx packet */
#define XAXIDMA_BD_CTRL_ALL_MASK	0x0C000000 /* All control bits */

#define XAXIDMA_BD_STS_ACTUAL_LEN_MASK	0x007FFFFF /* Actual len */
#define XAXIDMA_BD_STS_COMPLETE_MASK	0x80000000 /* Completed */
#define XAXIDMA_BD_STS_DEC_ERR_MASK	0x40000000 /* Decode error */
#define XAXIDMA_BD_STS_SLV_ERR_MASK	0x20000000 /* Slave error */
#define XAXIDMA_BD_STS_INT_ERR_MASK	0x10000000 /* Internal err */
#define XAXIDMA_BD_STS_ALL_ERR_MASK	0x70000000 /* All errors */
#define XAXIDMA_BD_STS_RXSOF_MASK	0x08000000 /* First rx pkt */
#define XAXIDMA_BD_STS_RXEOF_MASK	0x04000000 /* Last rx pkt */
#define XAXIDMA_BD_STS_ALL_MASK		0xFC000000 /* All status bits */

#define XAXIDMA_BD_MINIMUM_ALIGNMENT	0x40
#define PIRADIO_NAPI_WEIGHT         64

#define OFDM_SYMBOL_SIZE 80
#define OFDM_FRAME_DATA_CARRIERS   (9 * 640)
#define PIRADIO_MAX_DMA_TRANSF_LEN (OFDM_FRAME_DATA_CARRIERS / (8 / MOD_QPSK))
#define PIRADIO_MAX_JUMBO_FRAME    (6 * PIRADIO_MAX_DMA_TRANSF_LEN)




struct axidma_bd {
	phys_addr_t next;	/* Physical address of next buffer descriptor */
#ifndef CONFIG_PHYS_ADDR_T_64BIT
	u32 reserved1;
#endif
	phys_addr_t phys;
#ifndef CONFIG_PHYS_ADDR_T_64BIT
	u32 reserved2;
#endif
	u32 reserved3;
	u32 reserved4;
	u32 cntrl;
	u32 status;
	u32 app0;
	u32 app1;	/* TX start << 16 | insert */
	u32 app2;	/* TX csum seed */
	u32 app3;
	u32 app4;
	phys_addr_t sw_id_offset; /* first unused field by h/w */
	phys_addr_t ptp_tx_skb;
	u32 ptp_tx_ts_tag;
	phys_addr_t tx_skb;
	u32 tx_desc_mapping;
} __aligned(XAXIDMA_BD_MINIMUM_ALIGNMENT);

struct piradio_dma {
	void __iomem *dma_regs;
	int irq;
	spinlock_t lock;
	struct axidma_bd *bd_list;
	u32 bd_curr;
	u32 bd_tail;
	dma_addr_t bd_list_base;
};

typedef enum{
	PIR_T_ACK   		= 1 << 0,
	PIR_T_DATA  		= 1 << 1,
	PIR_T_FRAME_START   = 1 << 2,
	PIR_T_FRAME_CONT    = 1 << 3,
	PIR_T_FRAME_END     = 1 << 4 
}piradio_hdr_t;

typedef enum{
	PIR_RX_IDLE         = 0,
	PIR_RX_OPEN         = 1
}piradio_rx_state_t;

struct tx_wq_item{
	struct work_struct work;
	struct net_device *netdev;
};

struct piradio_hdr {
	__u32           p_seq_num;              /* sequence number      */
	piradio_hdr_t   p_type;                 /* piradio packet type  */
	__u32             p_len;
	__u32             p_rand;
	__u32             p_csum;
} __attribute__((packed));

struct fec_ctrl_hdr{
	__u32 z_j     : 3,
		  z_set   : 3,
		  bg      : 3,
		  norm    : 4,
		  rsvd    : 1,
		  hard_op : 1,
		  inc_par : 1,
		  top     : 1,
		  tnc     : 1,
		  max_it  : 6,
		  id      : 8;
	__u32 mb      : 6,
	      msch    : 2,
		  rsvd2   : 24;
}__attribute__((packed));

struct fec_ctrl{
	struct fec_ctrl_hdr fec_hdr;
	__u32 block_data_len;
	__u32 block_len;
};

struct config_data{
	__u8 *data;
	__u16 length;
};

struct rx_state{
	piradio_rx_state_t state;
	__u32              storred_bytes;
};

struct tx_state{
	__u32              remaining_bytes;
	struct sk_buff*  pending_tx_skb;
};

typedef enum{
	PIRADIO_CONFIG_MOD        = SIOCDEVPRIVATE + 1,
	PIRADIO_CONFIG_FRAMER     = SIOCDEVPRIVATE + 2,
	PIRADIO_CONFIG_CORRELATOR = SIOCDEVPRIVATE + 3,
	PIRADIO_CONFIG_FIR        = SIOCDEVPRIVATE + 4,
	PIRADIO_CONFIG_FFT_TX     = SIOCDEVPRIVATE + 5,
	PIRADIO_CONFIG_FFT_RX     = SIOCDEVPRIVATE + 6,
	PIRADIO_CONFIG_CAPT_SYST  = SIOCDEVPRIVATE + 7,
	PIRADIO_CONFIG_FREQ_OFF   = SIOCDEVPRIVATE + 8,
	PIRADIO_CONFIG_ACK        = SIOCDEVPRIVATE + 9,
	PIRADIO_CONFIG_CSMA       = SIOCDEVPRIVATE + 10,
	PIRADIO_CONFIG_STHRESH    = SIOCDEVPRIVATE + 11,
	CONFIG_TRANSFER_LEN       = SIOCDEVPRIVATE + 12
}cmd_t;

typedef enum{
	MOD_BPSK				= 1,
	MOD_QPSK				= 2,
	MOD_QAM16				= 4,
	MOD_QAM64				= 6,
	MOD_QAM256				= 8
}mod_t;

typedef enum{
	CONFIG_FRAMER           = 0,
	CONFIG_CORRELATOR       =1
}conf_t;

typedef enum{
	DRIVER_IDLE				= 0,
	DRIVER_READY 			= 1
}driver_state_t;

struct stats{
	__u64	rx_packets;
	__u64	tx_packets;
	__u64	rx_bytes;
	__u64	tx_bytes;
	__u64	rx_errors;
	__u64	tx_errors;
	__u64	rx_dropped;
	__u64	tx_dropped;
	__u64	retransmissions;
	__u64   crc_rx_errors;
	__u64   rx_missed_errors;
};

struct piradio_dma_chann {

	struct dma_chan *channel_p;
	struct completion cmp;
	dma_cookie_t cookie;
	dma_addr_t mapping;
	__u8 *rx_buffer;
	//__u8 **tx_ring;
	//__u8 **rx_ring;
};

struct piradio_priv {
	spinlock_t lock;
	spinlock_t timer_lock;
	spinlock_t tx_byte_cnt_lock;
	struct net_device *netdev;
	struct device * dev;
	struct piradio_dma tx_dma;
	struct piradio_dma config_framer_dma;
	struct piradio_dma config_correlator_dma;
	struct piradio_dma rx_dma;
	struct piradio_dma tx_log_dma;
	struct piradio_dma rx_log_dma;
	struct piradio_dma framer_ctrl_dma;
	struct piradio_dma fec_tx_ctrl_dma;
	struct piradio_dma fec_rx_ctrl_dma;
	struct piradio_dma_chann framer_config_dma;
	struct piradio_dma_chann fft_tx_dma;
	struct piradio_dma_chann correlator_dma;
	struct piradio_dma_chann correlator2_dma;
	struct piradio_dma_chann fft_rx_dma;
	struct piradio_dma_chann fft_rx2_dma;
	struct platform_device *pdev;
	struct stats link_stats;
	struct sk_buff *last_skb;
	mod_t modulation;
	struct dma_async_tx_descriptor *chan_desc;
	__u32 sequence_num;

	size_t log_length;
	__u8 *ack_buffer;
	struct piradio_hdr *ack_hdr;

	__u16 csma_tries;
	__u16 tx_tries;
	struct hrtimer csma_timer;
	struct hrtimer ack_timer;
	__u8 ack_enabled;
	__u32 tx_transfer_length;
	__u32 rx_transfer_length;
	u32 tx_bd_num;
	u32 ctrl_bd_num;
	u32 tx_bd_config_num;
	u32 rx_bd_num;
	u32 coalesce_count_tx;
	u32 coalesce_count_rx;
	struct napi_struct napi;
	__u8* buff;
	__u8* buff_tmp;
	__u8* tx_log_buff;
	__u8* rx_log_buff;
	__u8* modulation_buf;
	__u8* fec_tx_ctrl;
	__u8* fec_rx_ctrl;
	__u8* rx_buffer;
	
	
	struct tx_state state_tx;
	struct tx_wq_item* tx_workqueue_item;
	struct workqueue_struct *tx_workqueue;
	void __iomem *mod_rx_base;
	void __iomem *mem_reg_base;
	void __iomem *memtx_reg_base;
	void __iomem *ifft_reg_base;
	void __iomem *fec_tx_base;
	void __iomem *fec_rx_base;
	void __iomem *fec_tx_ctrl_base;
	void __iomem *fec_rx_ctrl_base;
	void __iomem *rx_tlast_frame_len;
	phys_addr_t phys_tx;
	phys_addr_t phys_rx;
	driver_state_t driver_state;
	struct rx_state state_rx;
	bool main_iface;
	bool int_unack;

	struct fec_ctrl fec_struct;
};

int
piradio_prep_tx(struct sk_buff *skb, struct net_device *dev);

#endif /* PIRADIO_H_ */
