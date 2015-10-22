/*
 * Raspberry Pi GPIO - FPGA Interface Driver - Character device registration
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

#include <linux/types.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <asm/uaccess.h>

#include "ax-char.h"
#include "ax-fpga.h"

static int dev_open(struct inode *, struct file *);
static int dev_close(struct inode *, struct file *);
static ssize_t dev_read(struct file *, char __user *, size_t, loff_t *);
static ssize_t dev_write(struct file *, const char __user *, size_t, loff_t *);
static int dev_mmap(struct file *, struct vm_area_struct *);
static unsigned int dev_poll(struct file *, struct poll_table_struct *);

static struct file_operations fops =
{
	.open = dev_open,
	.release = dev_close,
	.read = dev_read,
	.write = dev_write,
	.mmap = dev_mmap,
	.poll = dev_poll
};

static int dev_ref_count = 0;
	
int ax_register_character_device(struct cdev *character_device)
{	
	cdev_init(character_device, &fops);
	character_device->owner = THIS_MODULE;
	return cdev_add(character_device, MKDEV(DEV_MAJOR, DEV_MINOR), 1);
}
EXPORT_SYMBOL_GPL(ax_register_character_device);

/**
 * Called when an 'open' system call is made on the device file.
 */
static int dev_open(struct inode * inod, struct file * filp)
{
	if (dev_ref_count < RING_BUFFER_COUNT && ax_driver != NULL) {
		filp->private_data = &ax_driver->tx[dev_ref_count++];
		return 0;
	}
	return -EBUSY;
}

/**
 * Called when a 'close' system call is made on the device file.
 */
static int dev_close(struct inode * inod, struct file * filp)
{
	struct output_data *tx = filp->private_data;
	printk(KERN_INFO "dev_close\n");
	tx->pause_state = 0;
	tx->buffer_state = 0;
	tx->buffer_mode = 0;
	filp->private_data = NULL;
	dev_ref_count--;
	return 0;
}

/**
 * Called when a 'read' system call is made on the device file.
 * Currently unused.
 */
static ssize_t dev_read(struct file * filp, char __user * buff, size_t len, loff_t * off)
{
	printk(KERN_INFO "dev_read\n");
	//put_user(1, buff);
	return 0;
}

/**
 * Called when a 'write' system call is made on the device file.
 * This is the userspace informing that the circular buffer should start processing entries
 */
static ssize_t dev_write(struct file * filp, const char __user * buff, size_t len, loff_t * off)
{
	int i;
	struct ring_item *item;
	u8 action[len];
	struct output_data *tx = filp->private_data;
	
	if (copy_from_user(&action, buff, len) == 0) {
		switch (action[0]) {
			case 0: //standard ring progression
				//wake up the kernel ring buffer consumer thread (if not already woken)
				up(&ax_thread_cond);
				break;
			case 1: //pause
				do_pause_transfer(tx, action[1]);
				break;
			case 2: //flush
				spin_lock(&tx->ring_lock);
				for(i = 0; i < RING_BUFFER_ITEM_COUNT; i++) {
				    item = &tx->ring[i];
					if (ACCESS_ONCE(item->status) == MMAP_STATUS_SEND_REQUEST)
						item->status = MMAP_STATUS_AVAILABLE;
				}
				tx->tail = 0; //reset buffer index
				wmb(); /* force memory to sync */
				spin_unlock(&tx->ring_lock);
				do_flush_transfer(tx);
				break;
			default:
				break;
		}
		return len;
	}
	return -1;
}

/**
 * Called when an 'mmap' system call is made on the device file.
 */
static int dev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	int i;
	struct output_data *tx = filp->private_data;
	
	//reset buffer index
	tx->tail = 0; 
	
	//clear the statuses
	for(i = 0; i < RING_BUFFER_ITEM_COUNT; i++)
	    tx->ring[i].status = MMAP_STATUS_AVAILABLE;
	  
	//ensure the output is flushed and not paused
	do_flush_transfer(tx);
	do_pause_transfer(tx, 0);
	
	return dma_mmap_coherent(&ax_driver->spi->dev, vma, tx->mem, tx->dma_handle, RING_BUFFER_SIZE);
}

/**
 * Called when a 'poll' system call is made on the device file.
 */
static unsigned int dev_poll(struct file *filp, struct poll_table_struct *poll_table) {
	int i;
	struct output_data *tx;
	unsigned int mask = 0;
	
	tx = filp->private_data;
	poll_wait(filp, &tx->wait_queue, poll_table);	
	
	//the circular buffer is always used in order so the first free index is where the head (producer) is waiting
	for(i = 0; i < RING_BUFFER_ITEM_COUNT; i++) {
		if (ACCESS_ONCE(tx->ring[i].status) == MMAP_STATUS_AVAILABLE) {
			mask |= POLLOUT | POLLWRNORM; /* writable */
			break;
		}
	}
	
	up(&ax_thread_cond);
	
	return mask;
}
