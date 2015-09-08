/*
 * Raspberry Pi GPIO - FPGA Interface Driver
 *
 * Authored by Robert Bartle, May 2015
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
 
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/circ_buf.h>
#include <linux/spi/spi.h>
#include <linux/device.h>
#include <linux/err.h>

#ifdef CONFIG_ARCH_BCM2708
#include <mach/platform.h>
#endif

#include "ax-fpga.h"
#include "ax-spi.h"
#include "ax-char.h"

static bool realtime = 0;
module_param(realtime, bool, 0);
MODULE_PARM_DESC(realtime, "Run the driver with realtime priority");

static struct cdev character_device;
struct device *chardrv_device;
static struct class *chardrv_class;
	
struct fgpa_data *ax_driver = NULL;
EXPORT_SYMBOL_GPL(ax_driver);

static int __init ax_fpga_module_init(void);
static void __exit ax_fpga_module_exit(void);	
module_init(ax_fpga_module_init);
module_exit(ax_fpga_module_exit);

static int fpga_transfer_spi(struct ring_item *item, u32 index);
static void fpga_transfer_complete(void *context);

static bool end_thread = 0;
static struct task_struct *thread;
struct semaphore ax_thread_cond;
EXPORT_SYMBOL_GPL(ax_thread_cond);
 
struct fgpa_data *fpga_buffers_alloc(struct device *dev)
{
	struct spi_transfer *t;
	struct spi_message *m;
	int i, j;
	struct fgpa_data *data;
	struct output_data *tx;
	
	i = 0;
	data = kzalloc(sizeof(struct fgpa_data), GFP_KERNEL);
	if (!data) {
		printk(KERN_INFO "failed to allocate fgpa_data\n");
		return NULL;
	}
 				 		
 	dev->coherent_dma_mask = 0xFFFFFFFF;
	for (; i<RING_BUFFER_COUNT; i++) {
		tx = &data->tx[i];
		init_waitqueue_head(&tx->wait_queue);
		//tx->head = 0; //producer/writer/userspace
		tx->tail = 0; //consumer/reader/kernel
		//spin_lock_init(&tx->consumer_lock);
		//INIT_LIST_HEAD(&tx->linked_list);		
				
		if ((tx->mem = dma_alloc_coherent(dev, RING_BUFFER_SIZE, &tx->dma_handle, GFP_DMA)) == NULL)
 			goto alloc_dma_fail;
 			
 		//Set the ring on the first page aligned boundary in mem.
		tx->ring = tx->mem + (((size_t)tx->mem) & (PAGE_SIZE-1));
						
 		//pre-allocate spi_transfer and spi_message structs for each ring_item
 		for (j = 0; j<RING_BUFFER_ITEM_COUNT; j++) {
 			//SPI Message
 			m = &tx->spi_message[j];
			spi_message_init(m);
			m->complete = fpga_transfer_complete;
			m->is_dma_mapped = 1;
			
			//Enable S/PDIF output setting transfer
 			t = &tx->enable_transfer[j];
 			t->cs_change = 1;
 			t->len = 2;
 			
			//Sample word size settings transfer
 			t = &tx->word_size_transfer[j];
 			t->cs_change = 1;
 			t->len = 2;
 			
 			//Sample rate settings transfer
 			t = &tx->sample_rate_transfer[j];
 			t->cs_change = 1;
 			t->len = 2;
 							 			
 			//SPI PCM data transfer
 			t = &tx->pcm_transfer[j];
 		}
	}
		
	return data;
alloc_dma_fail:
	printk(KERN_INFO "failed to allocate dma buffers\n");
	//free any successful allocations
	for (; i>=0; i--) {
		tx = &data->tx[i];
		dma_free_coherent(dev, RING_BUFFER_SIZE, (void*)tx->mem, tx->dma_handle);
	}
	kfree(data);
	return NULL;
}

static int process_ring(struct output_data *tx)
{
	int i;
	//u32 head;
	u32 status;
	struct ring_item *item;
	
	//spin_lock(&tx->consumer_lock);

	//head = smp_load_acquire(&tx->head);
	
	//dma_rmb(); - dma_wmb();
	//if (CIRC_CNT(head, tx->tail, RING_BUFFER_ITEM_COUNT) >= 1) {
	
	//must only perform a maximum of the ring size iterations at a time so that other rings
	//have processing time too.
	for(i = 0; i < RING_BUFFER_ITEM_COUNT; i++) {
	    item = &tx->ring[tx->tail];
		status = ACCESS_ONCE(item->status);
		if (status == MMAP_STATUS_SEND_REQUEST) {
			dma_rmb(); /* do not process data until we own buffer */
			
			//printk(KERN_INFO "MMAP_STATUS_SEND_REQUEST :%d \n", tx->tail);
			
			item->status = MMAP_STATUS_SENDING;
			fpga_transfer_spi(item, tx->tail);
			ACCESS_ONCE(tx->tail) = (tx->tail + 1) & (RING_BUFFER_ITEM_COUNT - 1);
		} else if (status == MMAP_STATUS_SENDING) {
			//printk(KERN_INFO "MMAP_STATUS_SENDING :%d \n", tx->tail);
			
			//caught up to the producer which still has pending data to go
			break;
		} else { //MMAP_STATUS_AVAILABLE
			//printk(KERN_INFO "MMAP_STATUS_AVAILABLE :%d \n", tx->tail);
			
			//item is available - we are waiting on the producer to give us more data
			break;
		}
	}
	
	//spin_unlock(&tx->consumer_lock);	
    return 0;
}

int ax_thread_function(void *userdata)
{
	int i;
	struct fgpa_data *data = userdata;
	while (!end_thread) { //!kthread_should_stop()
		//wait for signal to process the ring buffers
		if (down_interruptible(&ax_thread_cond) == 0)
			for (i = 0; i < RING_BUFFER_COUNT; i++)
				process_ring(&data->tx[i]);
	}
	return 0;
}
	
static int fpga_driver_probe_spi(struct spi_device *spi)
{
	printk(KERN_INFO "%s: \n", __FUNCTION__);
	 		
	ax_driver = fpga_buffers_alloc(&spi->dev);
	if (ax_driver == NULL)
		return -1;
	ax_driver->spi = spi;
	spi_set_drvdata(spi, ax_driver);
	
	sema_init(&ax_thread_cond, 0);
	thread = kthread_run(&ax_thread_function, (void *)ax_driver, "ax_thread");
	
	return 0;
}

static int fpga_driver_remove_spi(struct spi_device *spi)
{
	int i;
	struct fgpa_data *data = spi->dev.driver_data;
	printk(KERN_INFO "%s\n", __FUNCTION__);
	
	end_thread = 1;
	up(&ax_thread_cond);
	kthread_stop(thread);
	
	if (data)  {
		struct output_data *tx;	
		for (i=0; i<RING_BUFFER_COUNT; i++) {
			tx = &data->tx[i];
			dma_free_coherent(&spi->dev, RING_BUFFER_SIZE, (void*)tx->mem, tx->dma_handle);
		}
		spi_set_drvdata(spi, NULL);
		kfree(data);
	}
	
	return 0;
}

static struct spi_driver fpga_spi_driver = {
	.driver = {
		.name   = "AX-FPGAv1",
		.owner  = THIS_MODULE
	},
	.probe  = fpga_driver_probe_spi,
	.remove = fpga_driver_remove_spi
};
	
static void fpga_transfer_complete(void *context)
{
	//int rl;
	struct output_data *tx;
	struct ring_item *item = context;

	//convert output from register address back into index
	tx = &ax_driver->tx[(item->output & ~0x41) >> 1];	
	item->status = MMAP_STATUS_AVAILABLE;
	
	//wake up any polling process
	wmb(); /* force memory to sync */
	wake_up_interruptible_sync_poll(&tx->wait_queue, POLLOUT);
	//up(&ax_thread_cond);
}

static int fpga_transfer_spi(struct ring_item *item, u32 index)
{
	struct spi_transfer *t, *t2;
	struct spi_message *m;
	struct output_data *tx;
	u8 output;
	bool first = true;
	//int rl;
		
	output = item->output;
	tx = &ax_driver->tx[output];
	m = &tx->spi_message[index];
	m->context = item;	
	
	//Check if settings have changed.  Add extra SPI transfers to set them.
	if (tx->enabled != item->enable) {
		//Enable output transfer (bitfield of all S/PDIF outputs.  bit set if on, clear if off)
		item->enable_register = (0x01 << 1) | 0x01;
		t = &tx->enable_transfer[index];
		item->enable = (ax_driver->tx[0].enabled | (ax_driver->tx[1].enabled << 1) |
							(ax_driver->tx[2].enabled << 2) | (ax_driver->tx[3].enabled << 3)) ^ 
								(tx->enabled << output);
		t->tx_buf = &item->enable_register;
	
		INIT_LIST_HEAD(&t->transfer_list);
		INIT_LIST_HEAD(&m->transfers);
		spi_message_add_tail(t, m);
		first = false;
		t2 = t;
	}
	if (tx->word_size != item->word_size) {
		//Word size transfer (bitfield of all S/PDIF outputs.  bit set if 16bit, clear if 24bit)
		item->word_size_register = (0x02 << 1) | 0x01;
		t = &tx->word_size_transfer[index];
		item->word_size = (ax_driver->tx[0].word_size | (ax_driver->tx[1].word_size << 1) |
							(ax_driver->tx[2].word_size << 2) | (ax_driver->tx[3].word_size << 3)) ^ 
								(tx->word_size << output);
		t->tx_buf = &item->word_size_register;
		
		if (first) {
			INIT_LIST_HEAD(&t->transfer_list);
			INIT_LIST_HEAD(&m->transfers);
			spi_message_add_tail(t, m);
			first = false;
		} else {
			list_add_tail(&t->transfer_list, &t2->transfer_list);
		}
		t2 = t;
	}
	if (tx->sample_rate != item->sample_rate) {
		//Sample rate transfer
		item->sample_rate_register = ((0x10 + output) << 1) | 0x01;
		t = &tx->sample_rate_transfer[index];
		t->tx_buf = &item->sample_rate_register;
		
		if (first) {
			INIT_LIST_HEAD(&t->transfer_list);
			INIT_LIST_HEAD(&m->transfers);
			spi_message_add_tail(t, m);
			first = false;
		} else {
			list_add_tail(&t->transfer_list, &t2->transfer_list);
		}
		t2 = t;
	}
		
	//Output select transfer
	item->output = ((0x20 + output) << 1) | 0x01;
	t = &tx->pcm_transfer[index];
	t->tx_buf = &item->output; //right before the &item->data memory region..
	t->len = item->data_len+1;
	if (first) {
		INIT_LIST_HEAD(&t->transfer_list);
		INIT_LIST_HEAD(&m->transfers);
		spi_message_add_tail(t, m);
		first = false;
	} else {
		list_add_tail(&t->transfer_list, &t2->transfer_list);
	}
		
	//these messages spam the log and cause big problems!
	//rl = printk_ratelimit();
	//if (rl)
	//	//printk(KERN_INFO "%s: ", __FUNCTION__);
	//printk(KERN_INFO "%s: Output:%d Length:%d", __FUNCTION__, output, item->data_len);

//why?
//AX-FPGAv1 spi0.0: Unaligned spi tx-transfer bridging page

	return !end_thread && spi_async(ax_driver->spi, m);
}
     
static int __init ax_fpga_module_init(void)
{
	int ret;
	dev_t device_number = MKDEV(DEV_MAJOR, DEV_MINOR);
	if (realtime) {
		struct task_struct *kernel_task;
		struct sched_param param;
		kernel_task = current;

		param.sched_priority = 80; //high priority
		sched_setscheduler(kernel_task, SCHED_FIFO, &param);
	}

	printk(KERN_INFO "ax_fpga module initializing\n");
	
	ret = ax_register_spi_driver(&fpga_spi_driver);
	if (ret != 0) {
		printk ("Failed to register spi driver\n");
		return ret;
	}
	
	if (IS_ERR(chardrv_class = class_create(THIS_MODULE, DEV_NAME))) {
		spi_unregister_driver(&fpga_spi_driver);
		return PTR_ERR(chardrv_class);
	}
	if (IS_ERR(chardrv_device = device_create(chardrv_class, NULL, device_number, NULL, DEV_NAME))) {
		spi_unregister_driver(&fpga_spi_driver);
		class_destroy(chardrv_class);
		return PTR_ERR(chardrv_device);
	}
	
	ret = ax_register_character_device(&character_device);
	if (ret != 0) {
		printk("Character device registration failed\n");	
		spi_unregister_driver(&fpga_spi_driver);
		device_destroy(chardrv_class, device_number);
		class_destroy(chardrv_class);
		return ret;
	}
	
	return ret;
}

static void __exit ax_fpga_module_exit(void)
{
	printk ("ax_fpga module unloaded\n");
	spi_unregister_driver(&fpga_spi_driver);
	device_destroy(chardrv_class, MKDEV(DEV_MAJOR, DEV_MINOR));
	class_destroy(chardrv_class);
	cdev_del(&character_device);
}

MODULE_ALIAS("AX-FPGAv1");
MODULE_DESCRIPTION("Axium FPGA driver for Raspberry Pi");
MODULE_AUTHOR("Robert Bartle <robert@axium.co.nz>");
MODULE_LICENSE("GPL");



//struct dma_memory_region {
//	int mem_offset; //offset from the base
//	int mem_size;   //size of this region
//	struct list_head list;
//};
////Searches through currently used dma region to find a region with enough space for this transaction
//static struct dma_memory_region *assign_dma_memory_region(u32 output, u32 required_len) {
//	struct dma_memory_region *reg, *reg2;
//	int offset = 0;
//	struct list_head linked_list = &data->tx[output].linked_list;
//
//	if (!list_empty(linked_list)) 
//	list_for_each_entry (reg, linked_list, list) { 
//		offset = reg->mem_offset+reg->mem_size;
//		
//		//make sure the region fits the max size of the dma allocated memory
//		if (required_len < RING_BUFFER_SIZE - offset)
//			goto CONTINUE_MAIN_ITERATION;
//		
//		//make sure this region does not overlap any other region
//		list_for_each_entry (reg2, linked_list, list)
//			if (reg != reg2 && (reg2->mem_size+reg2->mem_offset > offset || offset+required_len > reg2->mem_offset))
//    			goto CONTINUE_MAIN_ITERATION;
//    	
//    	//if execution reaches here, then the offset is valid..
//    	break;
//    	CONTINUE_MAIN_ITERATION:;
//    	offset = -1;
//    }
//    
//    if (offset != -1)  {
//    	reg = kmalloc(sizeof(struct dma_memory_region), GFP_KERNEL);
//    	if (reg != NULL) {
//	    	reg->mem_offset = offset;
//	    	reg->mem_size = required_len;
//	    	INIT_LIST_HEAD(&reg->list);
//	    	list_add(&reg->list, linked_list);
//	    	return reg;
//    	}
//    }
//    return NULL;
//}
