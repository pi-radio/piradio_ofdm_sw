#include <stdio.h>
#include <string.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include "piradio.h"
#include <chrono>
#include <thread>
#define TEMPLATE_SIZE            1024
#define SYNC_WORD_SIZE           1024
#define MAP_SIZE                 32
#define UNIFIED_BUFF_SIZE        (1024 * 2)


int main(int argc, char* argv[]) {
	struct ifreq ifr;
	struct config_data conf;
	char* template_fn, *sync_word_fn, *map_fn;
	FILE* template_fp, *sync_word_fp, *map_fp;
	uint32_t ttemplate[1024];
	uint32_t sync_word[1024];
	uint32_t map[32];
	uint32_t unified_config_struct[UNIFIED_BUFF_SIZE];
	uint32_t sync_temp[2048];
	uint8_t modulation = 0;
	uint8_t skip = 0;
	uint8_t skip_mod = 0;
	uint8_t send_udp = 0;
	uint8_t config_log = 0;
	uint8_t get_log = 0;
	ifr.ifr_ifru.ifru_data = &conf;
	int sock_rx_corr_fd;
	uint32_t sync_word_shifted[1024];

	struct sockaddr_in     servaddr;
	memset(&servaddr, 0, sizeof(servaddr));

	// Filling server information
	servaddr.sin_family = AF_INET;
	servaddr.sin_port = htons(16543);
	servaddr.sin_addr.s_addr = inet_addr("192.168.4.5");
	uint32_t * log_tx = (uint32_t *)malloc(10 * 10 * 1280 * 4);
	uint32_t * log_rx = (uint32_t *)malloc(10 * 10 * 1280 * 4);
	FILE* file_tx;
	FILE* file_rx;
	file_tx = fopen("/mnt/sd-mmcblk0p1/log_tx.bin", "wb");
	file_rx = fopen("log_rx.bin", "wb");
	int opt;
	int ifft = 0;
	size_t usec;
	while ((opt = getopt(argc, argv, "m:s:t:r:kpu:cgi:")) != -1) {
		switch (opt) {
		case 'i':
			ifft = atoi(optarg);
		case 't':
			template_fn = (char *)optarg;
			break;
		case 'm':
			map_fn = (char *)optarg;
			break;
		case 's':
			sync_word_fn = (char *)optarg;
			break;
		case 'r' :
			modulation = atoi(optarg);
			break;
		case 'k' :
			skip = 1;
			break;
		case 'p':
			skip_mod = 1;
			break;
		case 'u':
			send_udp = 1;
			usec = atoi(optarg);
			break;
		case 'c':
			config_log = 1;
			break;
		case 'g':
			get_log = 1;
			break;
		default:
			exit(0);
		}
	}
	if ((sock_rx_corr_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		perror("socket creation failed");
		exit(EXIT_FAILURE);
	}
	if(config_log){
		conf.length = SYNC_WORD_SIZE ;
		strcpy(ifr.ifr_name, "piradio00");
		ioctl(sock_rx_corr_fd, PIRADIO_CONFIG_FIR, &ifr);
	}
	if(get_log){
		conf.data = (uint8_t*)log_tx;
		conf.length = SYNC_WORD_SIZE ;
		strcpy(ifr.ifr_name, "piradio00");
		//ioctl(sock_rx_corr_fd, PIRADIO_CONFIG_FFT_TX, &ifr);
		//fwrite(log_tx, sizeof(uint32_t), 8 * 10 * 1280, file_tx );
		conf.data = (uint8_t*)log_rx;
		ioctl(sock_rx_corr_fd, PIRADIO_CONFIG_FFT_RX, &ifr);
		fwrite(log_rx, sizeof(uint32_t), 10 * 10 * 1280, file_rx );
		//fclose(file_tx);
		fclose(file_rx);
	}
	if(ifft){
		conf.length = ifft ;
		strcpy(ifr.ifr_name, "piradio00");
		ioctl(sock_rx_corr_fd, PIRADIO_CONFIG_FREQ_OFF, &ifr);
	}

	if(!skip){
		template_fp = fopen(template_fn, "rb");
		//sync_word_fp = fopen(sync_word_fn, "rb");
		map_fp = fopen(map_fn, "rb");

		fread(sync_temp, sizeof(uint32_t), 1024*2, template_fp);
		//fread(sync_word, sizeof(uint32_t), 1024, sync_word_fp);
		fread(map, sizeof(uint32_t), 32, map_fp);

		fclose(template_fp);
		//fclose(sync_word_fp);
		fclose(map_fp);
		memcpy(&sync_word[0], &sync_temp[0],1024 * sizeof(uint32_t));
		memcpy(ttemplate, &sync_temp[1024], 1024 * sizeof(uint32_t));
		memcpy(&sync_word_shifted[0], &sync_word[512], 512 * sizeof(uint32_t));
		memcpy(&sync_word_shifted[512], &sync_word[0], 512 * sizeof(uint32_t));
		for (int i=0 ; i < 1024 * 2 ; i+=2){
			unified_config_struct[i] = ttemplate[i/2];
			unified_config_struct[i + 1] = (map[(i/2)/32] >> ((i/2) % 32)) & 0x01;
		}

		conf.data = (uint8_t *)sync_word_shifted;
		conf.length = SYNC_WORD_SIZE * sizeof(uint32_t);
		strcpy(ifr.ifr_name, "piradio00");
		ioctl(sock_rx_corr_fd, PIRADIO_CONFIG_CORRELATOR, &ifr);
		usleep(100);
		conf.data = (uint8_t *)sync_word;
		conf.length = SYNC_WORD_SIZE * sizeof(uint32_t);
		strcpy(ifr.ifr_name, "piradio00");
		ioctl(sock_rx_corr_fd, PIRADIO_CONFIG_FRAMER, &ifr);
		usleep(100);
		conf.data = (uint8_t *)unified_config_struct;
		conf.length = UNIFIED_BUFF_SIZE * sizeof(uint32_t);
		strcpy(ifr.ifr_name, "piradio00");
		ioctl(sock_rx_corr_fd, PIRADIO_CONFIG_FRAMER, &ifr);
		strcpy(ifr.ifr_name, "piradio01");
		ioctl(sock_rx_corr_fd, PIRADIO_CONFIG_STHRESH, &ifr);
	}
	if(!skip_mod){
		//usleep(100);
		//conf.length = modulation;
		//strcpy(ifr.ifr_name, "piradio00");
		//ioctl(sock_rx_corr_fd, PIRADIO_CONFIG_MOD, &ifr);
		//strcpy(ifr.ifr_name, "piradio01");
		//ioctl(sock_rx_corr_fd, PIRADIO_CONFIG_MOD, &ifr);
	}
	if(send_udp){
		uint8_t testbuf[500];
		for(int i =0 ; i < 500 ; i++){
			testbuf[i] = rand() % 256;
		}
		while(1){
			std::this_thread::sleep_for(std::chrono::nanoseconds(usec));	
			sendto(sock_rx_corr_fd, (const char *)testbuf, 500,
					MSG_DONTWAIT, (const struct sockaddr *) &servaddr,
						sizeof(servaddr));
		}
	}
}
