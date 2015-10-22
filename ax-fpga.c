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
static int fpga_query_transfer_spi(struct output_data *tx);
static void fpga_transfer_complete(void *context);
static void fpga_query_transfer_complete(void *context);
static void fpga_pause_transfer_complete(void *context);
static void fpga_flush_transfer_complete(void *context);

static bool end_thread = 0;
static struct task_struct *thread;
struct semaphore ax_thread_cond;
EXPORT_SYMBOL_GPL(ax_thread_cond);
 
struct fgpa_data *fpga_buffers_alloc(struct device *dev)
{
	int i;
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
		spin_lock_init(&tx->ring_lock);
		
		if ((tx->mem = dma_alloc_coherent(dev, RING_BUFFER_SIZE, &tx->dma_handle, GFP_DMA)) == NULL)
 			goto alloc_dma_fail;
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

static void process_ring(struct output_data *tx)
{
	int i;
	u32 status;
	struct ring_item *item;
		
	if (tx->buffer_mode) {
		//The SDRAM buffer is currently draining so instead of writing more data,
		//'poll' the buffer state until it reaches a set level when data should be filled again.  
				
		//If there is not currently a transfer in progress, and that at least 10ms has gone by
		//since the last buffer state query..
		getnstimeofday(&tx->ts_now);
		if (!tx->pause_state && tx->query_spi_message.context == NULL && (tx->ts_now.tv_sec * MSEC_PER_SEC) +
			 (tx->ts_now.tv_nsec / NSEC_PER_MSEC) > (tx->ts_start.tv_sec * MSEC_PER_SEC) +
			  (tx->ts_start.tv_nsec / NSEC_PER_MSEC) + 10)
			fpga_query_transfer_spi(tx);
		return;
	}
	
	spin_lock(&tx->ring_lock);
	//must only perform a maximum of the ring size iterations at a time so that other rings
	//have processing time too.
	for(i = 0; i < RING_BUFFER_ITEM_COUNT; i++) {
	    item = &tx->ring[tx->tail];
		status = ACCESS_ONCE(item->status);
		if (status == MMAP_STATUS_SEND_REQUEST) {
			dma_rmb(); /* do not process data until we own buffer */
	
			if (!fpga_transfer_spi(item, tx->tail)) {
				//Is not ready to receive data
				spin_unlock(&tx->ring_lock);	
				return;
			}
				
			item->status = MMAP_STATUS_SENDING;
			ACCESS_ONCE(tx->tail) = (tx->tail + 1) & (RING_BUFFER_ITEM_COUNT - 1);
		} else if (status == MMAP_STATUS_SENDING) {
			//caught up to the producer which still has pending data to go
			break;
		} else { //MMAP_STATUS_AVAILABLE
			//item is available - we are waiting on the producer to give us more data
			break;
		}
	}
	spin_unlock(&tx->ring_lock);	
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
	
	if (!end_thread) {
		end_thread = 1;
		up(&ax_thread_cond);
		kthread_stop(thread);
	}
	
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
		.name   = "ax-fpga",
		.owner  = THIS_MODULE
	},
	.probe  = fpga_driver_probe_spi,
	.remove = fpga_driver_remove_spi
};
	
static void fpga_transfer_complete(void *context)
{
	struct output_data *tx;
	struct ring_item *item = context;

	//if (!end_thread) {
	tx = &ax_driver->tx[(item->output & ~0x41) >> 1]; //convert output from register address back into index	 

	tx->buffer_state = item->buffer_state;	
	item->status = MMAP_STATUS_AVAILABLE;
			
	//wake up any polling process
	wake_up_poll(&tx->wait_queue, POLLOUT);
	//}	
}

static void fpga_query_transfer_complete(void *context)
{
	struct output_data *tx = context;
	if (tx->buffer_state <= 0x1C)
		//buffer is approaching about 10% capacity filled.  It is time to resume normal(filling) buffer mode
		tx->buffer_mode = 0;
	
	tx->query_spi_message.context = NULL;	
	getnstimeofday(&tx->ts_start);
}

static void fpga_pause_transfer_complete(void *context)
{
	struct output_data *tx = context;
	tx->pausing = 0;
}

static void fpga_flush_transfer_complete(void *context)
{
	struct output_data *tx = context;
	tx->flushing = 0;
	tx->buffer_state = 0;
	tx->buffer_mode = 0;
}

static int fpga_transfer_spi(struct ring_item *item, u32 index)
{
	struct spi_transfer *t;
	struct spi_message *m;
	struct output_data *tx;
	u8 output, word_size;
		
	output = item->output;
	
	tx = &ax_driver->tx[output];
	
	//If the buffer state reaches about 95% full (0xF0) then no more pcm transfers will be sent until the
	//buffer state drops back down to about 10% (0x1C).  The userspace code simply polls until the ring buffer
	//is ready, which is when the buffer has cleared up and the pending transfers on the ring have been sent.
	
	//Test the SDRAM buffer state
	if (tx->buffer_state >= 0xF0) {		
		//buffer is approaching 95% full (~500 milliseconds of buffering left).
		//Do not queue the message but leave the item in limbo until the buffer has drained somewhat.		
		tx->buffer_mode = 1;
		getnstimeofday(&tx->ts_start);
		return 0;
	}
	
	//Prepare SPI message
	m = &tx->spi_message[index];
	spi_message_init(m);
	m->complete = fpga_transfer_complete;
	m->context = item;	
	m->is_dma_mapped = 1;
	
	if (tx->word_size != item->word_size) {
		//Word size transfer (bitfield of all S/PDIF outputs.  bit set if 16bit, clear if 24bit)
		word_size =  item->word_size;
		item->word_size_register = (0x02 << 1) | 0x01;
		t = &tx->word_size_transfer[index];
		memset(t, 0, sizeof(struct spi_transfer));
		item->word_size = (ax_driver->tx[0].word_size | (ax_driver->tx[1].word_size << 1) |
							(ax_driver->tx[2].word_size << 2) | (ax_driver->tx[3].word_size << 3)) ^ 
								(word_size << output);
		tx->word_size = word_size;	
		t->tx_buf = &item->word_size_register;
		t->cs_change = 1;
 		t->len = 2;
		spi_message_add_tail(t, m);
	}
	
	if (tx->sample_rate != item->sample_rate) {
		tx->sample_rate = item->sample_rate;
		//Sample rate transfer
		item->sample_rate_register = ((0x10 + output) << 1) | 0x01;
		t = &tx->sample_rate_transfer[index];
		memset(t, 0, sizeof(struct spi_transfer));
		t->tx_buf = &item->sample_rate_register;
		t->cs_change = 1;
 		t->len = 2;
		spi_message_add_tail(t, m);
	}
	
	//Buffer state transfer
	item->buffer_state_register = (0x18 + output) << 1;
	t = &tx->buffer_state_transfer[index];
	memset(t, 0, sizeof(struct spi_transfer));
	t->tx_buf = &item->buffer_state_register;
	t->rx_buf = &item->buffer_state_register; //first byte read is garbage, second is the data we want
	t->cs_change = 1;
 	t->len = 2;
	spi_message_add_tail(t, m);
	
	//Finally the main data transfer
	item->output = ((0x20 + output) << 1) | 0x01;
	t = &tx->pcm_transfer[index];
	memset(t, 0, sizeof(struct spi_transfer));
	t->tx_buf = &item->output; //right before the &item->data memory region..
	t->tx_dma = tx->dma_handle + offsetof(struct ring_item, output);
	t->len = item->data_len+1;
	spi_message_add_tail(t, m);
	
	return spi_async(ax_driver->spi, m) == 0;
}

static int fpga_query_transfer_spi(struct output_data *tx)
{
	u8 output = tx - ax_driver->tx;
	
	//Prepare SPI message
	spi_message_init(&tx->query_spi_message);
	memset(&tx->query_buffer_state_transfer, 0, sizeof(struct spi_transfer));
	tx->query_spi_message.complete = fpga_query_transfer_complete;
	tx->query_spi_message.context = tx;
	tx->buffer_state_query = (0x18 + output) << 1;
	tx->buffer_state = 0;
	tx->query_buffer_state_transfer.tx_buf = &tx->buffer_state_query;
	tx->query_buffer_state_transfer.rx_buf = &tx->buffer_state_query;
	tx->query_buffer_state_transfer.len = 2;
	spi_message_add_tail(&tx->query_buffer_state_transfer, &tx->query_spi_message);
	
	return spi_async(ax_driver->spi, &tx->query_spi_message);
}

int do_pause_transfer(struct output_data *tx, u8 param)
{		
	u8 output = tx - ax_driver->tx;
	
	if (tx->pausing)
		return 0;
				
	//Prepare SPI message
	spi_message_init(&tx->pause_spi_message);
	memset(&tx->pause_transfer, 0, sizeof(struct spi_transfer));
	tx->pause_spi_message.complete = fpga_pause_transfer_complete;
	tx->pause_spi_message.context = tx;
	tx->pausing = ((0x04 + output) << 1) | 0x01;
	tx->pause_state = param;	
	tx->pause_transfer.tx_buf = &tx->pausing;
	tx->pause_transfer.len = 2;
	spi_message_add_tail(&tx->pause_transfer, &tx->pause_spi_message);

	return spi_async(ax_driver->spi, &tx->pause_spi_message);
}

int do_flush_transfer(struct output_data *tx)
{
	u8 output = tx - ax_driver->tx;
		
	if (tx->flushing)
		return 0;
				
	//Prepare SPI message
	spi_message_init(&tx->flush_spi_message);
	memset(&tx->flush_transfer, 0, sizeof(struct spi_transfer));
	tx->flush_spi_message.complete = fpga_flush_transfer_complete;
	tx->flush_spi_message.context = tx;
	tx->flushing = ((0x08 + output) << 1) | 0x01;
	tx->flush_state = 1;
	tx->flush_transfer.tx_buf = &tx->flushing;
	tx->flush_transfer.len = 2;	
	spi_message_add_tail(&tx->flush_transfer, &tx->flush_spi_message);
	
	return spi_async(ax_driver->spi, &tx->flush_spi_message);
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
		printk(KERN_INFO "Failed to register spi driver\n");
		return ret;
	}
	
	if (IS_ERR(chardrv_class = class_create(THIS_MODULE, DEV_NAME))) {
		printk(KERN_INFO "Failed to class_create\n");
		spi_unregister_driver(&fpga_spi_driver);
		return PTR_ERR(chardrv_class);
	}
	if (IS_ERR(chardrv_device = device_create(chardrv_class, NULL, device_number, NULL, DEV_NAME))) {
		printk(KERN_INFO "Failed to device_create\n");
		spi_unregister_driver(&fpga_spi_driver);
		class_destroy(chardrv_class);
		return PTR_ERR(chardrv_device);
	}
	
	ret = ax_register_character_device(&character_device);
	if (ret != 0) {
		printk(KERN_INFO "Character device registration failed\n");	
		spi_unregister_driver(&fpga_spi_driver);
		device_destroy(chardrv_class, device_number);
		class_destroy(chardrv_class);
		return ret;
	}
	
	return ret;
}

static void __exit ax_fpga_module_exit(void)
{
	printk(KERN_INFO "ax_fpga module unloaded\n");
	spi_unregister_driver(&fpga_spi_driver);
	device_destroy(chardrv_class, MKDEV(DEV_MAJOR, DEV_MINOR));
	class_destroy(chardrv_class);
	cdev_del(&character_device);
}

//MODULE_ALIAS_CHARDEV(DEV_MAJOR, DEV_MINOR);
MODULE_ALIAS("ax-fpga");
MODULE_DESCRIPTION("Axium FPGA driver for Raspberry Pi");
MODULE_AUTHOR("Robert Bartle <robert@axium.co.nz>");
MODULE_LICENSE("GPL");
