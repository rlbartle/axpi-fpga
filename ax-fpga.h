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
#include <linux/time.h>
#include <asm/atomic.h>

#define RING_BUFFER_COUNT 4	 //4 S/PDIF outputs
#define RING_BUFFER_ITEM_COUNT 16 //16 pages of memory for transactions
#define RING_BUFFER_SIZE (PAGE_SIZE*RING_BUFFER_ITEM_COUNT)

#define RING_ITEM_SIZE PAGE_SIZE
#define RING_ITEM_HEADER_SIZE (10)
#define RING_ITEM_DATA_SIZE (RING_ITEM_SIZE - RING_ITEM_HEADER_SIZE)

#define MMAP_STATUS_AVAILABLE 0		//userspace can write into
#define MMAP_STATUS_SEND_REQUEST 1	//kernel can read
#define MMAP_STATUS_SENDING 2		//kernel is processing

#define dma_rmb()       dmb(osh)
#define dma_wmb()       dmb(oshst)
 
struct ring_item {
	//These two bytes are not used for SPI transfer
	u16 data_len; 
	
	//Bytes used in SPI transfers
	u8 buffer_state_register;
	u8 buffer_state;
	u8 word_size_register;
	u8 word_size;
	u8 sample_rate_register;
	u8 sample_rate;
	u8 output;
	char data[RING_ITEM_DATA_SIZE];
	//
	
	//byte not used for SPI transfer.  At the end so that output is word aligned
	u8 status; 
} __attribute__((packed));

struct output_data {
	union {
		void *mem; 						//CPU virtual address mapping
		struct ring_item *ring;
	};
	dma_addr_t dma_handle; 			//DMA address mapping
	u32 tail;						//ring buffer tail
	wait_queue_head_t wait_queue;

	//State variables for the output
	u8 word_size;
	u8 sample_rate;
	u8 buffer_mode; //0 - ready to fill, 1 - is draining
	struct timespec ts_start;
	struct timespec ts_now;	
	
	u8 buffer_state_query;
	u8 buffer_state;
	u8 pausing;
	u8 pause_state;
	u8 flushing;
	u8 flush_state;
	
	//Transfers and messages for each ring item
	struct spi_transfer word_size_transfer[RING_BUFFER_ITEM_COUNT];
	struct spi_transfer sample_rate_transfer[RING_BUFFER_ITEM_COUNT];
	struct spi_transfer buffer_state_transfer[RING_BUFFER_ITEM_COUNT];
	struct spi_transfer pcm_transfer[RING_BUFFER_ITEM_COUNT];
	struct spi_message spi_message[RING_BUFFER_ITEM_COUNT];
	
	//Transfer and message for querying the buffer state
	struct spi_transfer query_buffer_state_transfer;
	struct spi_message query_spi_message;	
	
	//Transfer and message for triggering pause 
	struct spi_transfer pause_transfer;
	struct spi_message pause_spi_message;
	
	//Transfer and message for triggering flush 
	struct spi_transfer flush_transfer;
	struct spi_message flush_spi_message;
};

struct fgpa_data {
	struct spi_device *spi;
	struct output_data tx[RING_BUFFER_COUNT];
};

extern struct fgpa_data *ax_driver;
extern struct semaphore ax_thread_cond;

int do_pause_transfer(struct output_data *tx, u8 param);
int do_flush_transfer(struct output_data *tx);

#endif /* __AX_FPGA_H */
