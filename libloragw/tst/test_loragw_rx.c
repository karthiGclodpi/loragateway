/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2013 Semtech-Cycleo

Description:
	Minimum test program for the loragw_hal 'library'

License: Revised BSD License, see LICENSE.TXT file include in the project
Maintainer: Sylvain Miermont, Jiapeng Li
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

/* fix an issue between POSIX and C99 */
#if __STDC_VERSION__ >= 199901L
	#define _XOPEN_SOURCE 600
#else
	#define _XOPEN_SOURCE 500
#endif

#include <stdint.h>		/* C99 types */
#include <stdbool.h>	/* bool type */
#include <stdio.h>		/* printf */
#include <string.h>		/* memset */
#include <signal.h>		/* sigaction */
#include <stdlib.h>
#include <stdarg.h>

#include <unistd.h>		/* getopt access */

#include "loragw_hal.h"
#include "loragw_reg.h"
#include "loragw_aux.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

static int exit_sig = 0; /* 1 -> application terminates cleanly (shut down hardware, close open files, etc) */
static int quit_sig = 0; /* 1 -> application terminates without shutting down the hardware */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DECLARATION ---------------------------------------- */

static void sig_handler(int sigio);

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

static void sig_handler(int sigio) {
	if (sigio == SIGQUIT) {
		quit_sig = 1;;
	} else if ((sigio == SIGINT) || (sigio == SIGTERM)) {
		exit_sig = 1;
	}
}

#define CHANNEL_NUM		8
#define F_RX_0 			(0)
#define F_RX_1			(0)

int offset_tab[CHANNEL_NUM][3] = {
	{-400000, 0, F_RX_0,},
	{-200000, 0, F_RX_0,},
	{      0, 0, F_RX_0,},
	{ 200000, 0, F_RX_0,},
	{-400000, 1, F_RX_1,},
	{-200000, 1, F_RX_1,},
	{      0, 1, F_RX_1,},
	{ 200000, 1, F_RX_1,},
};

char *str;
/* describe command line options */
void usage(void) 
{
	printf("*** Library version information ***\n%s\n\n", lgw_version_info());
	printf("Usage: %s FreqA FreqB\n", str);
	printf("FreqA<float> and FreqB<float> are in MHz\n");
	printf("Eg: %s 433.1 434.1\n", str);
	printf("To set Radio A: 433.1/433.3/433.5/433.7\n");
	printf("       Radio B: 434.1/434.3/434.5/434.7\n");
	printf("FreqA, FreqB are start freqeuncy of each radio\n");
	printf("Channel offset 200KHz\n");
}

#define LOG_PRIORITY_FATAL  0
#define LOG_PRIORITY_ALERT  1
#define LOG_PRIORITY_CRIT   2
#define LOG_PRIORITY_ERROR  3
#define LOG_PRIORITY_WARN   4
#define LOG_PRIORITY_NOTICE 5
#define LOG_PRIORITY_INFO   6
#define LOG_PRIORITY_DEBUG  7
#define LOG_PRIORITY_TRACE  8

void lgw_log(int priority, const char *format, ...)
{
  switch (priority) {
    case LOG_PRIORITY_FATAL:
      printf("\033[37;41;1m");
      break;
    case LOG_PRIORITY_ALERT:
    case LOG_PRIORITY_CRIT:
    case LOG_PRIORITY_ERROR:
      printf("\033[31;1m");
      break;
    case LOG_PRIORITY_WARN:
      printf("\033[33;1m");
      break;
    case LOG_PRIORITY_NOTICE:
      printf("\033[34;1m");
      break;
    default:
      printf("\033[32m");
  }
  va_list va;
  va_start(va, format);
  vprintf(format, va);
  printf("\033[0m");
  printf("\n");
  fflush(stdout);
}

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main(int argc, char ** argv)
{
	struct sigaction sigact; /* SIGQUIT&SIGINT&SIGTERM signal handling */
	
	struct lgw_conf_rxrf_s rfconf;
	struct lgw_conf_rxif_s ifconf;
	
	struct lgw_pkt_rx_s rxpkt[4]; /* array containing up to 4 inbound packets metadata */
	struct lgw_pkt_rx_s *p; /* pointer on a RX packet */
	
	int i, j;
	int nb_pkt;
	int rx_freq[2];
	double f;
	int channel_num = CHANNEL_NUM;
	int rf_chain;
	str = argv[0];
	
	switch(argc){
		case 1:
			rx_freq[0] = 0;
			rx_freq[1] = 0;
			break;
		case 2:
			f = atof(argv[1]) * 1000000;
			rx_freq[0] = f;
			rx_freq[1] = 0;
			channel_num = 4;
			if(rx_freq[0] == 0){
				usage();
				exit(-1);
			}
			rx_freq[0] += 400000;
			break;
		case 3:
			f = atof(argv[1]) * 1000000;
			rx_freq[0] = f;
			f = atof(argv[2]) * 1000000;
			rx_freq[1] = f;
			channel_num = CHANNEL_NUM;
			if(rx_freq[0] == 0 || rx_freq[1] == 0){
				usage();
				exit(-1);
			}
			rx_freq[0] += 400000;
			rx_freq[1] += 400000;
			break;
		default:
			usage();
			exit(-1);
			break;
	}

	/* configure signal handling */
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigact.sa_handler = sig_handler;
	sigaction(SIGQUIT, &sigact, NULL);
	sigaction(SIGINT, &sigact, NULL);
	sigaction(SIGTERM, &sigact, NULL);

	/* set configuration for RF chains */
	memset(&rfconf, 0, sizeof(rfconf));

	/* set configuration for LoRa multi-SF channels (bandwidth cannot be set) */
	memset(&ifconf, 0, sizeof(ifconf));

	lgw_auto_check();

	if( rx_freq[1] == 0 && rx_freq[0] == 0){
		if( lgw_get_radio_id(0) == ID_SX1255 ){
			rx_freq[0] = 433500000;
		}else if( lgw_get_radio_id(0) == ID_SX1257 ){
			rx_freq[0] = 868500000;
		}else{
			lgw_log(LOG_PRIORITY_FATAL, "Radio A chip unknown");
			usage();
			return -1;
		}

		if( lgw_get_radio_id(1) == ID_SX1255 ){
			rx_freq[1] = 434500000;
		}else if( lgw_get_radio_id(1) == ID_SX1257 ){
			rx_freq[1] = 869500000;
		}else{
			lgw_log(LOG_PRIORITY_FATAL, "Radio B chip unknown");
			usage();
			return -1;
		}		
	}

	for(rf_chain=0; rf_chain<2; rf_chain++) {
		switch(lgw_freq_validate(rf_chain, rx_freq[rf_chain])){
			case 0:
				break;
			case -1:
			default:
				lgw_log(LOG_PRIORITY_FATAL, "RF chain unsupported");
				usage();
				return -1;
			case -2:
				lgw_log(LOG_PRIORITY_FATAL, "ERROR: Radio %c freqeuncy must be between 400MHz and 510MHz for SX1255", (rf_chain == 0? 'A': 'B'));
				usage();
				return -1;
			case -3:
				lgw_log(LOG_PRIORITY_FATAL, "ERROR: Radio %c freqeuncy must be between 862MHz and 1020MHz for SX1257", (rf_chain == 0? 'A': 'B'));
				usage();
				return -1;
			case -4:
				lgw_log(LOG_PRIORITY_FATAL, "ERROR: Radio %c chip unknown", (rf_chain == 0? 'A': 'B'));
				usage();
				return -1;
		}

		/** use RF Chain A only */
		if(channel_num <= 4){
			break;
		}
	}
	
	for(i=0; i<8; i++){
		offset_tab[i][2] = rx_freq[i/4];
	}

	/* beginning of LoRa concentrator-specific code */
	printf("Beginning of test for loragw_hal.c\n");
	
	printf("*** Library version information ***\n%s\n\n", lgw_version_info());
	
	printf("F_RX0 = %d, F_RX1 = %d\n", rx_freq[0], rx_freq[1]);

	printf("%d freqeuncy channels are selected\n", channel_num);
	for(i=0; i<channel_num; i++){
		printf("channel: %d, freq: %d\n", i, offset_tab[i][0] + offset_tab[i][2]);
	}

	/* initialize all channels */
	for(i=0; i<channel_num; i++){
		rfconf.enable = true;
		rfconf.freq_hz = offset_tab[i][2];
		lgw_rxrf_setconf(offset_tab[i][1], rfconf);

		ifconf.enable = true;
		ifconf.rf_chain = offset_tab[i][1];
		ifconf.freq_hz = offset_tab[i][0];
		ifconf.datarate = DR_LORA_MULTI;
		lgw_rxif_setconf(i, ifconf); /* chain 0: LoRa 125kHz, all SF, on f0 - 0.3 MHz */
	}
	
	/* set configuration for LoRa 'stand alone' channel */
	memset(&ifconf, 0, sizeof(ifconf));
	ifconf.enable = true;
	ifconf.rf_chain = 0;
	ifconf.freq_hz = 0;
	ifconf.bandwidth = BW_250KHZ;
	ifconf.datarate = DR_LORA_SF10;
	lgw_rxif_setconf(8, ifconf); /* chain 8: LoRa 250kHz, SF10, on f0 MHz */
	
	/* set configuration for FSK channel */
	memset(&ifconf, 0, sizeof(ifconf));
	ifconf.enable = true;
	ifconf.rf_chain = 1;
	ifconf.freq_hz = 0;
	ifconf.bandwidth = BW_250KHZ;
	ifconf.datarate = 64000;
	lgw_rxif_setconf(9, ifconf); /* chain 9: FSK 64kbps, on f1 MHz */
	
	/* connect, configure and start the LoRa concentrator */
	i = lgw_start();
	if (i == LGW_HAL_SUCCESS) {
		printf("*** Concentrator started ***\n");
	} else {
		printf("*** Impossible to start concentrator ***\n");
		return -1;
	}
	
	/* once configured, dump content of registers to a file, for reference */
	// FILE * reg_dump = NULL;
	// reg_dump = fopen("reg_dump.log", "w");
	// if (reg_dump != NULL) {
		// lgw_reg_check(reg_dump);
		// fclose(reg_dump);
	// }
	
	while ((quit_sig != 1) && (exit_sig != 1)) {
		
		/* fetch N packets */
		nb_pkt = lgw_receive(ARRAY_SIZE(rxpkt), rxpkt);
		
		if (nb_pkt == 0) {
			wait_ms(300);
		} else {
			/* display received packets */
			for(i=0; i < nb_pkt; ++i) {
				p = &rxpkt[i];
				printf("---\nRcv pkt #%d >>", i+1);
				printf("freq:%d\n", offset_tab[p->if_chain][2]+offset_tab[p->if_chain][0]);
				if (p->status == STAT_CRC_OK) {
					printf(" if_chain:%2d", p->if_chain);
					printf(" tstamp:%010u", p->count_us);
					printf(" size:%3u", p->size);
					switch (p-> modulation) {
						case MOD_LORA: printf(" LoRa"); break;
						case MOD_FSK: printf(" FSK"); break;
						default: printf(" modulation?");
					}
					switch (p->datarate) {
						case DR_LORA_SF7: printf(" SF7"); break;
						case DR_LORA_SF8: printf(" SF8"); break;
						case DR_LORA_SF9: printf(" SF9"); break;
						case DR_LORA_SF10: printf(" SF10"); break;
						case DR_LORA_SF11: printf(" SF11"); break;
						case DR_LORA_SF12: printf(" SF12"); break;
						default: printf(" datarate?");
					}
					switch (p->coderate) {
						case CR_LORA_4_5: printf(" CR1(4/5)"); break;
						case CR_LORA_4_6: printf(" CR2(2/3)"); break;
						case CR_LORA_4_7: printf(" CR3(4/7)"); break;
						case CR_LORA_4_8: printf(" CR4(1/2)"); break;
						default: printf(" coderate?");
					}
					printf("\n");
					printf(" RSSI:%+6.1f SNR:%+5.1f (min:%+5.1f, max:%+5.1f) payload:\n", p->rssi, p->snr, p->snr_min, p->snr_max);
					
					for (j = 0; j < p->size; ++j) {
						printf(" %02X", p->payload[j]);
					}
					printf(" #\n");
				} else if (p->status == STAT_CRC_BAD) {
					printf(" if_chain:%2d", p->if_chain);
					printf(" tstamp:%010u", p->count_us);
					printf(" size:%3u\n", p->size);
					printf(" CRC error, damaged packet\n\n");
				} else if (p->status == STAT_NO_CRC){
					printf(" if_chain:%2d", p->if_chain);
					printf(" tstamp:%010u", p->count_us);
					printf(" size:%3u\n", p->size);
					printf(" no CRC\n\n");
				} else {
					printf(" if_chain:%2d", p->if_chain);
					printf(" tstamp:%010u", p->count_us);
					printf(" size:%3u\n", p->size);
					printf(" invalid status ?!?\n\n");
				}
			}
		}
	}
	
	if (exit_sig == 1) {
		/* clean up before leaving */
		lgw_stop();
	}
	
	printf("\nEnd of test for loragw_hal.c\n");
	return 0;
}

/* --- EOF ------------------------------------------------------------------ */
