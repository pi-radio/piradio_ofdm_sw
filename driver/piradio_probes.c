#include "piradio_probes.h"
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/module.h>

MODULE_LICENSE("Dual BSD/GPL");

int piradio_dma_probe(struct platform_device *pdev,
		struct piradio_dma* dma_obj, int dev_tree_idx){
	struct device_node* np = NULL;
	struct resource dmares;
	int ret;

	np = of_parse_phandle(pdev->dev.of_node, "mydms",
			dev_tree_idx);
	ret = of_address_to_resource(np, 0, &dmares);
	if (ret >= 0) {
		dma_obj->dma_regs = devm_ioremap_resource(&pdev->dev,
							&dmares);
		if(!dma_obj->dma_regs)
			return -1;
	}
	else{
		printk(KERN_ALERT "DMA probe problem %d %d %s", dev_tree_idx, ret, np->name);
		return -1;
	}
	dma_obj->irq = irq_of_parse_and_map(np, 0);
	of_node_put(np);
	spin_lock_init(&dma_obj->lock);
	printk(KERN_ALERT "DMA TX probe successful");
	return 0;
}


// int piradio_dma_rx_probe(struct platform_device *pdev,
// 		struct piradio_dma* dma_obj, int dev_tree_idx){
// 	struct device_node* np = NULL;
// 	struct resource dmares;
// 	int ret;

// 	np = of_parse_phandle(pdev->dev.of_node, "mydms",
// 						dev_tree_idx);
// 	ret = of_address_to_resource(np, 0, &dmares);
// 	if (ret >= 0) {
// 		dma_obj->dma_regs = devm_ioremap_resource(&pdev->dev,
// 							&dmares);
// 		if(!dma_obj->dma_regs)
// 			return -1;
// 	}
// 	else{
// 		printk(KERN_ALERT "DMA probe problem");
// 		return -1;
// 	}
// 	dma_obj->irq = irq_of_parse_and_map(np, 0);
// 	of_node_put(np);
// 	spin_lock_init(&dma_obj->lock);
// 	printk(KERN_ALERT "DMA RX probe successful");
// 	return 0;
// }

int piradio_reg_probe(struct platform_device *pdev,
	void __iomem **reg_base, const char* dev_tree_name, int dev_tree_idx){

	struct device_node* np = NULL;
	struct resource dmares;
	int ret;
	np = of_parse_phandle(pdev->dev.of_node, dev_tree_name,
						dev_tree_idx);
	ret = of_address_to_resource(np, 0, &dmares);
	if (ret >= 0) {
		*reg_base = devm_ioremap_resource(&pdev->dev,
							&dmares);
		if(!*reg_base)
			return -1;
	}
	else{
		printk(KERN_ALERT "REG probe problem");
		return -1;
	}
	of_node_put(np);
	printk(KERN_ALERT "REG probe successful");
	return 0;	
}

// int piradio_mem_reg_probe(struct platform_device *pdev,
// 	void __iomem **reg_base){

// 	struct device_node* np = NULL;
// 	struct resource dmares;
// 	int ret;
// 	np = of_parse_phandle(pdev->dev.of_node, "mem_reg",
// 						0);
// 	ret = of_address_to_resource(np, 0, &dmares);
// 	if (ret >= 0) {
// 		*reg_base = devm_ioremap_resource(&pdev->dev,
// 							&dmares);
// 		if(!*reg_base)
// 			return -1;
// 	}
// 	else{
// 		printk(KERN_ALERT "Log memory probe problem");
// 		return -1;
// 	}
// 	of_node_put(np);
// 	printk(KERN_ALERT "Log memory probe successful");
// 	return 0;	
// }
// int piradio_memtx_reg_probe(struct platform_device *pdev,
// 	void __iomem **reg_base){
// 	printk(KERN_ALERT "Log memory attempt");
// 	struct device_node* np = NULL;
// 	struct resource dmares;
// 	int ret;
// 	np = of_parse_phandle(pdev->dev.of_node, "memtx_reg",
// 						0);
// 	ret = of_address_to_resource(np, 1, &dmares);
// 	if (ret >= 0) {
// 		*reg_base = devm_ioremap_resource(&pdev->dev,
// 							&dmares);
// 		if(!*reg_base){
// 			printk(KERN_ALERT "Log memory probe problem");
// 			return -1;
// 		}
			
// 	}
// 	else{
// 		printk(KERN_ALERT "Log memory probe problem");
// 		return -1;
// 	}
// 	of_node_put(np);
// 	printk(KERN_ALERT "Log memory probe successful");
// 	return 0;	
// }

// int piradio_ifft_reg_probe(struct platform_device *pdev,
// 	void __iomem **reg_base){
// 	printk(KERN_ALERT "IFFT memory attempt");
// 	struct device_node* np = NULL;
// 	struct resource dmares;
// 	int ret;
// 	np = of_parse_phandle(pdev->dev.of_node, "ifft_reg",
// 						0);
// 	ret = of_address_to_resource(np, 0, &dmares);
// 	if (ret >= 0) {
// 		*reg_base = devm_ioremap_resource(&pdev->dev,
// 							&dmares);
// 		if(!*reg_base){
// 			printk(KERN_ALERT "IFFT memory probe problem");
// 			return -1;
// 		}
			
// 	}
// 	else{
// 		printk(KERN_ALERT "IFFT memory probe problem");
// 		return -1;
// 	}
// 	of_node_put(np);
// 	printk(KERN_ALERT "IFFT memory probe successful");
// 	return 0;	
// }

