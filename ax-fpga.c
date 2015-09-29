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
		tx->tail = 0; //consumer/reader/kernel
				
		if ((tx->mem = dma_alloc_coherent(dev, RING_BUFFER_SIZE, &tx->dma_handle, GFP_DMA)) == NULL)
 			goto alloc_dma_fail;
 									
 		//pre-allocate spi_transfer and spi_message structs for each ring_item
 		for (j = 0; j<RING_BUFFER_ITEM_COUNT; j++) {			
			//Sample word size settings transfer
 			t = &tx->word_size_transfer[j];
 			t->len = 2;
 			t->cs_change = 1;
 			
 			//Sample rate settings transfer
 			t = &tx->sample_rate_transfer[j];
 			t->cs_change = 1;
 			t->len = 2;
 			
 			//Buffer state transfer
			t = &tx->buffer_state_transfer[j];
 			t->len = 2;
 			t->cs_change = 1;
 							 			
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
	u32 status;
	struct ring_item *item;
	
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
	int rl;
	struct output_data *tx;
	struct ring_item *item = context;

	//convert output from register address back into index
	tx = &ax_driver->tx[(item->output & ~0x41) >> 1];	
	
	rl = printk_ratelimit();
	if (rl)
		printk(KERN_INFO "%s:BufferState:%.2x/%.2x FL:%d AL:%d", __FUNCTION__, item->buffer_state, item->buffer_state_register, tx->spi_message[(item - tx->ring)].frame_length, tx->spi_message[(item - tx->ring)].actual_length);
		
	item->status = MMAP_STATUS_AVAILABLE;
	
	//wake up any polling process
	wmb(); /* force memory to sync */
	wake_up_interruptible_sync_poll(&tx->wait_queue, POLLOUT);
}

static int fpga_transfer_spi(struct ring_item *item, u32 index)
{
	struct spi_transfer *t, *t2 = NULL;
	struct spi_message *m;
	struct output_data *tx;
	u8 output, word_size;
	//int rl;
		
	output = item->output;
	tx = &ax_driver->tx[output];
	m = &tx->spi_message[index];
	spi_message_init(m);
	m->complete = fpga_transfer_complete;
	m->context = item;	
	m->is_dma_mapped = 1;
	
	
	//Output select transfer
	item->output = ((0x20 + output) << 1) | 0x01;
	t = &tx->pcm_transfer[index];
	t->tx_buf = &item->output; //right before the &item->data memory region..
	t->tx_dma = tx->dma_handle + offsetof(struct ring_item, output);
	t->len = item->data_len+1;
	INIT_LIST_HEAD(&t->transfer_list);
	spi_message_add_tail(t, m);
	t2 = t;
	
	//Buffer state transfer
	item->buffer_state_register = 0x00;//((0x18 + output) << 1) | 0x00;
	t = &tx->buffer_state_transfer[index];
	t->tx_buf = &item->buffer_state_register;
	t->rx_buf = &item->buffer_state_register; //first byte read is garbage, second is the data we want
	list_add_tail(&t->transfer_list, &t2->transfer_list);
	t2 = t;
	
	//Check if settings have changed.  Add extra SPI transfers to set them.
	if (tx->sample_rate != item->sample_rate) {
		tx->sample_rate = item->sample_rate;
		//Sample rate transfer
		item->sample_rate_register = ((0x10 + output) << 1) | 0x01;
		t = &tx->sample_rate_transfer[index];
		t->tx_buf = &item->sample_rate_register;
		//t->rx_buf = &item->sample_rate_register;	
		list_add_tail(&t->transfer_list, &t2->transfer_list);
		t2 = t;
	}	
	
	if (tx->word_size != item->word_size) {
		//Word size transfer (bitfield of all S/PDIF outputs.  bit set if 16bit, clear if 24bit)
		word_size =  item->word_size;
		item->word_size_register = (0x02 << 1) | 0x01;
		t = &tx->word_size_transfer[index];
		item->word_size = (ax_driver->tx[0].word_size | (ax_driver->tx[1].word_size << 1) |
							(ax_driver->tx[2].word_size << 2) | (ax_driver->tx[3].word_size << 3)) ^ 
								(word_size << output);
		tx->word_size = word_size;					
								
		t->tx_buf = &item->word_size_register;
		//t->rx_buf = &item->word_size_register;	
		list_add_tail(&t->transfer_list, &t2->transfer_list);
	}
		
	//these messages spam the log and cause big problems!
	//rl = printk_ratelimit();
	//if (rl)
	//	//printk(KERN_INFO "%s: ", __FUNCTION__);
	//printk(KERN_INFO "%s: Output:%d Length:%d", __FUNCTION__, output, item->data_len);

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
