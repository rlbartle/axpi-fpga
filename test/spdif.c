/*
 * spdif.c
 *
 *  Created on: 11/06/2015
 *      Author: Robert Bartle
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <linux/if.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <unistd.h>
#include <sys/un.h>
#include <sys/mman.h>
#include <errno.h>
#include <sys/time.h>

#include "constants.h"
#include "spdif/spdif.h"

#define MMAP_STATUS_AVAILABLE 0		//userspace can write into
#define MMAP_STATUS_SEND_REQUEST 1	//kernel can read
#define MMAP_STATUS_SENDING 2		//kernel is processing

#define dmb(option) __asm__ __volatile__ ("dmb " #option : : : "memory")
#define dsb(option) __asm__ __volatile__ ("dsb " #option : : : "memory")

static int read_fd(int unix_domain_socket) {
	struct msghdr msg;
	ssize_t n;

	struct iovec iov;
	struct cmsghdr *cmptr = NULL;
	char ctrl_buf[CMSG_SPACE(sizeof(int))];
	char data[1];

	memset(&msg, 0, sizeof(struct msghdr));
	memset(ctrl_buf, 0, CMSG_SPACE(sizeof(int)));

	iov.iov_base = data;
	iov.iov_len = sizeof(data);

	msg.msg_name = NULL;
	msg.msg_namelen = 0;
	msg.msg_control = ctrl_buf;
	msg.msg_controllen = CMSG_LEN(sizeof(int));
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;

	if ((n = recvmsg(unix_domain_socket, &msg, 0)) > 0) {
		if ((cmptr = CMSG_FIRSTHDR(&msg)) != NULL
				&& cmptr->cmsg_len == CMSG_LEN(sizeof(int))) {
			if (cmptr->cmsg_level != SOL_SOCKET) {
				printf("control level != SOL_SOCKET\n");
				return -1;
			}
			if (cmptr->cmsg_type != SCM_RIGHTS) {
				printf("control type != SCM_RIGHTS\n");
				return -1;
			}
			printf("AX_FPGA FD obtained\n");
			return *CMSG_DATA(cmptr);// for cpp : *((int *) CMSG_DATA(cmptr));
		}
	}
	return -1;
}

static int write_output(int unix_domain_client_socket, int output) {
	return write(unix_domain_client_socket, &output, sizeof(int));
}

struct spdif_t *spdif_initialise(u_char output) {
	struct sockaddr_un addr;
	int unix_domain_socket;

	struct spdif_t *state = malloc(sizeof(struct spdif_t));
	state->output = output;
	state->ring_offset = 0;

	if ((unix_domain_socket = socket(AF_UNIX, SOCK_STREAM, 0)) == -1) {
		perror("socket error");
		goto ERROR;
	} else {
		memset(&addr, 0, sizeof(addr));
		addr.sun_family = AF_UNIX;
		addr.sun_path[0] = 0;//make it an abstract path
		strncpy(addr.sun_path+1, GPIO_UDS_PATH, sizeof(addr.sun_path) - 2);

		if (connect(unix_domain_socket, (struct sockaddr*) &addr, sizeof(addr)) == -1) {
			perror("connect error");
			goto ERROR;
		} else {
			//connected to the gpio helper process.  Tell the GPIO process what output we want
			if (write_output(unix_domain_socket, output) == -1) {
				perror("write_output");
				close(unix_domain_socket);
				goto ERROR;
			} else {
				//read the result from the unix domain socket
				state->fpga_fd = read_fd(unix_domain_socket);
				close(unix_domain_socket);
				char *map;

				/* Map circular ring. */
				if ((map = mmap(NULL, FPGA_RING_ITEM_SIZE*FPGA_RING_ITEM_COUNT, PROT_READ | PROT_WRITE,
						MAP_SHARED | MAP_LOCKED | MAP_POPULATE, state->fpga_fd, 0)) == MAP_FAILED &&
					//if attempting to 'lock' the memory fails, then try again without lock
					(map = mmap(NULL, FPGA_RING_ITEM_SIZE*FPGA_RING_ITEM_COUNT, PROT_READ | PROT_WRITE,
						MAP_SHARED, state->fpga_fd, 0)) == MAP_FAILED) {
					perror("mmap ring buffer");
					state->ring = NULL;
					goto ERROR;
				}

				// Allocate and initialise the ring buffers
				state->ring = (struct iovec *) malloc(FPGA_RING_ITEM_COUNT * sizeof(struct iovec));
				for(unsigned i = 0; i < FPGA_RING_ITEM_COUNT; i++) {
					state->ring[i].iov_base = map + (i * FPGA_RING_ITEM_SIZE);
					state->ring[i].iov_len  = FPGA_RING_ITEM_SIZE;
				}
			}
		}
	}

#ifdef _AX_LOG_LATENCY_
	gettimeofday(&state->t, NULL);
	state->rt = 0;
	state->wc = 0;
	state->bna_c = 0;
#endif

	return state;
ERROR:
	free(state);
	return NULL;
}

void spdif_destroy(struct spdif_t *state) {
	if (state != NULL) {
		if (state->fpga_fd != -1)
			close(state->fpga_fd);
		free(state);
	}
}

void prepare_spdif(struct spdif_t *state, ax_word_size word_size, ax_sample_rate sample_rate, unsigned int sample_channels) {
	state->word_size = word_size;
	state->sample_rate = sample_rate;
	state->channels = sample_channels;
	state->buffered_len = 0;
	__atomic_store_n(&state->do_buffer, 1, __ATOMIC_RELEASE);
}

int write_fpga_msg(struct spdif_t *state, const char *data, size_t data_len) {

	//write into the appropriate shared memory region according to the spdif output specified.
	//Find any free blocks within that region and mark it as ready to send.  Then send.

	struct fpga_ring_item *header;

	size_t sent_data = 0;
	u_int16_t size;
	int p;
	for (size_t remains = data_len; remains > 0 && !finish;) {

		header = state->ring[state->ring_offset].iov_base;

		//dsb(osh);
		volatile u_int status;
		while ((status = (volatile u_int)(header->status)) != MMAP_STATUS_AVAILABLE) {
			// MMAP_STATUS_AVAILABLE 0		//userspace can write into
			// MMAP_STATUS_SEND_REQUEST 1	//kernel can read
			// MMAP_STATUS_SENDING 2		//kernel is processing

			//The kernel has started processing the data, now waiting on the DMA transfer to complete.
			struct pollfd pollset;
			pollset.fd = state->fpga_fd;
			pollset.events = POLLOUT;
			pollset.revents = 0;
			if (errno == EINTR || finish) {
				errno = 0;
				return -1;
			} else if ((p = poll(&pollset, 1, 100)) < 0) {
				if (errno != EINTR)
					perror("poll");
				errno = 0;
				return -1;
			} else {
				++state->bna_c;
//				printf("BNA: ring_offset = %d status:%d poll_events:%d src_data_len:%d remains:%d\n",
//						state->ring_offset, status, p, data_len, remains);
//				dsb(osh);
//				errno = 0;
//				return 0;
			}
		}

		if (state->buffered_len > 0) {
			//first copy the previously buffered data..
			memcpy(header->data, state->buffered_data, state->buffered_len);

			size = (remains > FPGA_RING_ITEM_SIZE-FPGA_RING_ITEM_HDRLEN ?
					FPGA_RING_ITEM_SIZE-FPGA_RING_ITEM_HDRLEN : remains) - state->buffered_len;
			remains -= size;

			memcpy(header->data+state->buffered_len, data+sent_data, size);
			sent_data += size;

			size += state->buffered_len;
			state->buffered_len = 0;
		} else {
			size = remains > FPGA_RING_ITEM_SIZE-FPGA_RING_ITEM_HDRLEN ?
					FPGA_RING_ITEM_SIZE-FPGA_RING_ITEM_HDRLEN : remains;
			remains -= size;

			memcpy(header->data, data+sent_data, size);
			sent_data += size;
		}

		// fill header
		header->data_len = size;
		header->output = state->output;
		header->sample_rate = state->sample_rate;
		header->word_size = state->word_size;

		//make sure all the data is written out before setting the status.  This way the kernel module can guarantee that
		//when encountering MMAP_STATUS_SEND_REQUEST that it can be processed without keeping record of the producer state
		dmb(oshst);
		header->status = MMAP_STATUS_SEND_REQUEST;

		//increment ring buffer offset
		state->ring_offset = (state->ring_offset + 1) & (FPGA_RING_ITEM_COUNT - 1);


		if (__atomic_load_n(&state->do_buffer, __ATOMIC_ACQUIRE) && state->buffered_len == 0
						&& remains <= sizeof(state->buffered_data)) {
			//buffer any small data segments for the next time to save resources.
			//Obviously do not buffer if this is the final audio message.
			state->buffered_len = remains;
			memcpy(state->buffered_data, data+sent_data, remains);
			break;
		}

		//XXX! if the number of iterations is equal to the size of the ring, then a blocking write must be made!!
		//I don't think it is possible for the caller (spotify_integration) to buffer that much though so just watch it..
		//16 * 4076 = 65216 bytes.  Max buffer in one-shot is 32768
	}

	//XXX! sometimes the latencies logged are really big and kernel module is starved of data.  where is the latency coming
	//from??  Test the time to execute various functions especially system calls like the write below..

	//MSG_DONTWAIT
	state->write_data_mode[0] = 0;
	if (write(state->fpga_fd, state->write_data_mode, 1) < 0) {
		perror("fpga write");
		errno = 0;
		return -1;
	}


#ifdef _AX_LOG_LATENCY_
	gettimeofday(&state->t, NULL);
	++state->wc;
	if (((state->t.tv_sec*1000000) + state->t.tv_usec) - state->rt > 110000) {
		printf("%u write(s) / %u bna(s) / %lu us / %u bytes\n", state->wc, state->bna_c, ((state->t.tv_sec*1000000) + state->t.tv_usec) - state->rt, data_len);
		state->wc = 0;
		state->bna_c = 0;
	}
	state->rt = (state->t.tv_sec*1000000) + state->t.tv_usec;
#endif

	return 0;
}

int pause_fpga_msg(struct spdif_t *state, u_char param) {
	state->write_data_mode[0] = 1;
	state->write_data_mode[1] = param;
	if (write(state->fpga_fd, state->write_data_mode, 2) < 0) {
		perror("fpga pause");
		return -1;
	}
	return 0;
}

int flush_fpga_msg(struct spdif_t *state) {
	state->write_data_mode[0] = 2;
	if (write(state->fpga_fd, state->write_data_mode, 1) < 0) {
		perror("fpga flush");
		return -1;
	}
	return 0;
}
