#define _POSIX_C_SOURCE 200809L
#define _BSD_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include "spdif.h"
int main(int argc, char *argv[]) {

	size_t data_len = 4096;
	char data[data_len];
	int f = open("/dev/urandom", O_RDONLY);
	read(f, data, data_len);
	close(f);

	if (argc > 1) {
		init();

		struct spdif_t *state = spdif_initialise(strtol(argv[1], NULL, 10));

		prepare_spdif(state, AX_PCM_16, AX_44_1KHZ_RATE, 2);

		while (write_fpga_msg(state, data, data_len) == 0)
			usleep(100);

		spdif_destroy(state);

		return 0;
	}
	return -1;
}
