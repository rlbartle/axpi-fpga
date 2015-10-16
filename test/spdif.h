/*
 * spdif.h
 *
 *  Created on: 11/06/2015
 *      Author: Robert Bartle
 */

#ifndef SPDIF_SPDIF_H_
#define SPDIF_SPDIF_H_

#include <sys/time.h>
#include <poll.h>
#include <sys/types.h>
#ifndef bool
#include <stdbool.h>
#endif

#define _AX_LOG_LATENCY_

#define FPGA_RING_ITEM_SIZE 4096
#define FPGA_RING_ITEM_COUNT 16
#define FPGA_RING_ITEM_HDRLEN (12)

extern u_int8_t finish;

typedef enum {
	AX_PCM_24 = 0,
	AX_PCM_16 = 1
} __attribute__ ((packed)) ax_word_size;

typedef enum {
	AX_UNKNOWN_RATE = 0x00,
	AX_32KHZ_RATE = 0x01,
	AX_44_1KHZ_RATE = 0x02,
	AX_48KHZ_RATE = 0x03,
	AX_88_2KHZ_RATE = 0x04,
	AX_96KHZ_RATE = 0x05
} __attribute__ ((packed)) ax_sample_rate;

typedef struct fpga_ring_item {
	//These two bytes are not used for SPI transfer
	u_int16_t data_len;

	//Bytes used in SPI transfers
	u_int8_t buffer_state_register;
	u_int8_t buffer_state;
	u_int8_t word_size_register;
	ax_word_size word_size;
	u_int8_t sample_rate_register;
	ax_sample_rate sample_rate;
	u_int8_t output;
	char data[FPGA_RING_ITEM_SIZE-FPGA_RING_ITEM_HDRLEN];
	u_int16_t padding; //used to make sure in 16bit mode the left/right channels are paired
	//////////////////////////////

	//byte not used for SPI transfer.  At the end so that output is word aligned
	u_int8_t status;
} __attribute__ ((packed)) fpga_ring_item;

struct spdif_t {
#ifdef _AX_LOG_LATENCY_
	struct timeval t;
	u_int64_t rt;
	u_int32_t wc, bna_c;
#endif

	int fpga_fd;
	struct iovec *ring;
	unsigned int ring_offset;

	u_int8_t output;

	bool do_buffer;
	unsigned short buffered_len;
	unsigned char buffered_data[256];


	/* Byte 0 is the mode:
	 * 0 - ring item
	 * 1 - pause
	 * 2 - flush
	 *
	 * Byte 1 is the parameter:
	 * Pause:
	 * 	0 - unpause
	 * 	1 - pause
	 *
	 */
	unsigned char write_data_mode[2];

	ax_word_size word_size;
	ax_sample_rate sample_rate;
	u_int8_t channels;
};

void init();
struct spdif_t *spdif_initialise(u_int8_t output);
void spdif_destroy(struct spdif_t *state);
void prepare_spdif(struct spdif_t *state, ax_word_size word_size, ax_sample_rate sample_rate, unsigned int sample_channels);
int write_fpga_msg(struct spdif_t *state, const char *data, size_t data_len);
int pause_fpga_msg(struct spdif_t *state, u_int8_t param);
int flush_fpga_msg(struct spdif_t *state);

#endif /* SPDIF_SPDIF_H_ */
