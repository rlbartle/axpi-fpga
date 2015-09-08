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
 
#ifndef __AX_FPGA_H
#define __AX_FPGA_H

#include <linux/spi/spi.h>
#include <linux/dma-mapping.h>
#include <linux/semaphore.h>
#include <linux/wait.h>
#include <asm/atomic.h>

#define RING_BUFFER_COUNT 4	 //4 S/PDIF outputs
#define RING_BUFFER_ITEM_COUNT 16 //16 pages of memory for transactions
#define RING_BUFFER_SIZE (PAGE_SIZE*RING_BUFFER_ITEM_COUNT)

#define RING_ITEM_SIZE PAGE_SIZE
#define RING_ITEM_HEADER_SIZE (12)
#define RING_ITEM_DATA_SIZE (RING_ITEM_SIZE - RING_ITEM_HEADER_SIZE)

#define MMAP_STATUS_AVAILABLE 0		//userspace can write into
#define MMAP_STATUS_SEND_REQUEST 1	//kernel can read
#define MMAP_STATUS_SENDING 2		//kernel is processing

#define dma_rmb()       dmb(osh)
#define dma_wmb()       dmb(oshst)
 
struct ring_item {
	u16 data_len; //bytes not used for DMA transfer
	u8 enable_register;
	u8 enable;
	u8 word_size_register;
	u8 word_size;
	u8 sample_rate_register;
	u8 sample_rate;
	u8 status; //byte not used for DMA transfer
	u8 padding1; //temporary until SPI hardware driver bug fixed which causes the
	u8 padding2; //output and data need to be word aligned for DMA to work..
	//! the output and data are contiguous in memory so the DMA transfer will use the output pointer to begin the DMA
	//transfer for the actual PCM data.
//	union {
//		struct {
			u8 output;
			char data[RING_ITEM_DATA_SIZE];
//		}
//		char dma_data[RING_ITEM_DATA_SIZE+1];
//	}

} __attribute__((packed));

struct output_data {
	void *mem; 						//CPU virtual address mapping
	struct ring_item *ring; 		//Page align ring buffer memory region using mem
	dma_addr_t dma_handle; 			//DMA address mapping
	//struct list_head linked_list; //list of dma_memory_region
	//u32 head; 					//ring buffer head
	u32 tail;						//ring buffer tail
	wait_queue_head_t wait_queue;
	//spinlock_t consumer_lock;
	
	struct spi_transfer enable_transfer[RING_BUFFER_ITEM_COUNT];
	struct spi_transfer word_size_transfer[RING_BUFFER_ITEM_COUNT];
	struct spi_transfer sample_rate_transfer[RING_BUFFER_ITEM_COUNT];
	struct spi_transfer pcm_transfer[RING_BUFFER_ITEM_COUNT];
	struct spi_message spi_message[RING_BUFFER_ITEM_COUNT];
	u8 enabled;
	u8 word_size;
	u8 sample_rate;
};

struct fgpa_data {
	struct spi_device *spi;
	struct output_data tx[RING_BUFFER_COUNT];
};

extern struct fgpa_data *ax_driver;
extern struct semaphore ax_thread_cond;

#endif /* __AX_FPGA_H */
