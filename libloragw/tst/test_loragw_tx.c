/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2013 Semtech-Cycleo

Description:
	TX test function

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
#include <stdio.h>		/* printf fprintf sprintf fopen fputs */
#include <stdarg.h>

#include <string.h>		/* memset */
#include <signal.h>		/* sigaction */
#include <unistd.h>		/* getopt access */
#include <stdlib.h>		/* exit codes */

#include "loragw_hal.h"
#include "loragw_aux.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a)	(sizeof(a) / sizeof((a)[0]))
#define MSG(args...)	fprintf(stderr, args) /* message that is destined to the user */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */



/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES (GLOBAL) ------------------------------------------- */

/* signal handling variables */
struct sigaction sigact; /* SIGQUIT&SIGINT&SIGTERM signal handling */
static int exit_sig = 0; /* 1 -> application terminates cleanly (shut down hardware, close open files, etc) */
static int quit_sig = 0; /* 1 -> application terminates without shutting down the hardware */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DECLARATION ---------------------------------------- */

static void sig_handler(int sigio);

void usage (void);

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

static void sig_handler(int sigio) {
	if (sigio == SIGQUIT) {
		quit_sig = 1;;
	} else if ((sigio == SIGINT) || (sigio == SIGTERM)) {
		exit_sig = 1;
	}
}

/* describe command line options */
void usage(void) {
	printf("*** Library version information ***\n%s\n\n", lgw_version_info());
	printf( "Available options:\n");
	printf( " -h print this help\n");
	printf( " -f <float> target frequency in MHz\n");
	printf( " -s <uint> Spreading Factor\n");
	printf( " -b <uint> Modulation bandwidth in kHz\n");
	printf( " -p <int> RF power (dBm)\n");
	printf( " -r <uint> LoRa preamble length (symbols)\n");
	printf( " -z <uint> payload size (bytes)\n");
	printf( " -t <uint> pause between packets (ms)\n");
	printf( " -x <int> numbers of times the sequence is repeated (-1 for continuous)\n");
	printf( " -i send packet using inverted modulation polarity \n");
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

int main(int argc, char **argv)
{
	int i;
	uint8_t status_var;

	/* user entry parameters */
	int xi = 0;
	double f0, f1;
	uint32_t f_min[LGW_RF_CHAIN_NB];
	uint32_t f_max[LGW_RF_CHAIN_NB];

	/* application parameters */
	uint32_t f_target[LGW_RF_CHAIN_NB] = {0}; /* target frequency */
	int sf = 10; /* SF10 by default */
	int bw = 125; /* 125kHz bandwidth by default */
	int pow = 14; /* 14 dBm by default */
	int preamb = 8; /* 8 symbol preamble by default */
	int pl_size = 16; /* 16 bytes payload by default */
	int delay = 1000; /* 1 second between packets by default */
	int repeat = -1; /* by default, repeat until stopped */
	bool invert = false;

	/* RF configuration (TX fail if RF chain is not enabled) */
	struct lgw_conf_rxrf_s rfconf = {true, 0};

	/* allocate memory for packet sending */
	struct lgw_pkt_tx_s txpkt; /* array containing 1 outbound packet + metadata */

	/* loop variables (also use as counters in the packet payload) */
	uint16_t cycle_count = 0;

	uint8_t rf_chain_enable = 0; /** 0: both disabled, 1: Chain A enable, 2: Chain B enable, 3: Both enable */

	/* parse command line options */
	while ((i = getopt (argc, argv, "hf:s:b:p:r:z:t:x:i")) != -1) {
		switch (i) {
			case 'h':
				usage();
				return EXIT_FAILURE;
				break;

			case 'f': /* -f <float> target frequency in MHz */
				i = sscanf(optarg, "%lf:%lf", &f0, &f1);
				f_target[0] = (uint32_t)((f0*1e6) + 0.5); /* .5 Hz offset to get rounding instead of truncating */
				f_target[1] = (uint32_t)((f1*1e6) + 0.5); /* .5 Hz offset to get rounding instead of truncating */

				if(f0){
					rf_chain_enable |= 0x01;
				}
				if(f1){
					rf_chain_enable |= 0x02;
				}

				break;

			case 's': /* -s <int> Spreading Factor */
				i = sscanf(optarg, "%i", &xi);
				if ((i != 1) || (xi < 7) || (xi > 12)) {
					MSG("ERROR: invalid spreading factor\n");
					usage();
					return EXIT_FAILURE;
				} else {
					sf = xi;
				}
				break;

			case 'b': /* -b <int> Modulation bandwidth in kHz */
				i = sscanf(optarg, "%i", &xi);
				if ((i != 1) || ((xi != 125)&&(xi != 250)&&(xi != 500))) {
					MSG("ERROR: invalid LoRa bandwidth\n");
					usage();
					return EXIT_FAILURE;
				} else {
					bw = xi;
				}
				break;

			case 'p': /* -p <int> RF power */
				i = sscanf(optarg, "%i", &xi);
				if ((i != 1) || (xi < -60) || (xi > 60)) {
					MSG("ERROR: invalid RF power\n");
					usage();
					return EXIT_FAILURE;
				} else {
					pow = xi;
				}
				break;

			case 'r': /* -r <uint> preamble length (symbols) */
				i = sscanf(optarg, "%i", &xi);
				if ((i != 1) || (xi < 6)) {
					MSG("ERROR: preamble length must be >6 symbols \n");
					usage();
					return EXIT_FAILURE;
				} else {
					preamb = xi;
				}
				break;

			case 'z': /* -z <uint> payload length (bytes) */
				i = sscanf(optarg, "%i", &xi);
				if ((i != 1) || (xi <= 0)) {
					MSG("ERROR: invalid payload size\n");
					usage();
					return EXIT_FAILURE;
				} else {
					pl_size = xi;
				}
				break;

			case 't': /* -t <int> pause between packets (ms) */
				i = sscanf(optarg, "%i", &xi);
				if ((i != 1) || (xi < 0)) {
					MSG("ERROR: invalid time between packets\n");
					usage();
					return EXIT_FAILURE;
				} else {
					delay = xi;
				}
				break;

			case 'x': /* -x <int> numbers of times the sequence is repeated */
				i = sscanf(optarg, "%i", &xi);
				if ((i != 1) || (xi < -1)) {
					MSG("ERROR: invalid number of repeats\n");
					usage();
					return EXIT_FAILURE;
				} else {
					repeat = xi;
				}
				break;

			case 'i': /* -i send packet using inverted modulation polarity */
				invert = true;
				break;

			default:
				MSG("ERROR: argument parsing\n");
				usage();
				return EXIT_FAILURE;
		}
	}

	lgw_auto_check();
	if( f_target[0] == 0 && f_target[1] == 0 ){
		if( lgw_get_radio_id(0) == ID_SX1255 ){
			f_target[0] = 433500000;
			rf_chain_enable |= 0x01;
		}else if( lgw_get_radio_id(0) == ID_SX1257 ){
			f_target[0] = 868500000;
			rf_chain_enable |= 0x01;
		}else{
			lgw_log(LOG_PRIORITY_FATAL, "Radio A chip unknown");
			usage();
			return -1;
		}
		if( lgw_get_radio_id(1) == ID_SX1255 ){
			f_target[1] = 433500000;
			rf_chain_enable |= 0x02;
		}else if( lgw_get_radio_id(1) == ID_SX1257 ){
			f_target[1] = 868500000;
			rf_chain_enable |= 0x02;
		}else{
			lgw_log(LOG_PRIORITY_FATAL, "Radio A chip unknown");
			usage();
			return -1;
		}
	}

	if(rf_chain_enable == 0){
		lgw_log(LOG_PRIORITY_FATAL, "No Radio is enabled.");
		return -1;
	}

	if( lgw_get_radio_id(0) == ID_SX1255 ){
		/* check parameter sanity */
		f_min[0] = LGW_RF_SX1255_FREQ_MIN + (500 * bw);
		f_max[0] = LGW_RF_SX1255_FREQ_MAX - (500 * bw);
	}else if( lgw_get_radio_id(0) == ID_SX1257 ){
		f_min[0] = LGW_RF_SX1257_FREQ_MIN + (500 * bw);
		f_max[0] = LGW_RF_SX1257_FREQ_MAX - (500 * bw);
	}else{
		lgw_log(LOG_PRIORITY_FATAL, "Radio A chip unknown");
		usage();
		return -1;
	}
	if( lgw_get_radio_id(1) == ID_SX1255 ){
		/* check parameter sanity */
		f_min[1] = LGW_RF_SX1255_FREQ_MIN + (500 * bw);
		f_max[1] = LGW_RF_SX1255_FREQ_MAX - (500 * bw);
	}else if( lgw_get_radio_id(1) == ID_SX1257 ){
		f_min[1] = LGW_RF_SX1257_FREQ_MIN + (500 * bw);
		f_max[1] = LGW_RF_SX1257_FREQ_MAX - (500 * bw);
	}else{
		lgw_log(LOG_PRIORITY_FATAL, "Radio A chip unknown");
		usage();
		return -1;
	}

	if ( (rf_chain_enable & 0x01) &&  ((f_target[0] < f_min[0]) || (f_target[0] > f_max[0])) ){
		MSG("ERROR: frequency out of authorized band (accounting for modulation bandwidth)\n");
		return EXIT_FAILURE;
	}
	printf("Sending %i packets on %u Hz (BW %i kHz, SF %i, %i bytes payload, %i symbols preamble) at %i dBm, with %i ms between each\n", \
		repeat, f_target[0], bw, sf, pl_size, preamb, pow, delay);

	if ( (rf_chain_enable & 0x02) && ((f_target[1] < f_min[1]) || (f_target[1] > f_max[1])) ){
		MSG("ERROR: frequency out of authorized band (accounting for modulation bandwidth)\n");
		return EXIT_FAILURE;
	}
	printf("Sending %i packets on %u Hz (BW %i kHz, SF %i, %i bytes payload, %i symbols preamble) at %i dBm, with %i ms between each\n", \
		repeat, f_target[1], bw, sf, pl_size, preamb, pow, delay);

	/* configure signal handling */
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigact.sa_handler = sig_handler;
	sigaction(SIGQUIT, &sigact, NULL);
	sigaction(SIGINT, &sigact, NULL);
	sigaction(SIGTERM, &sigact, NULL);

	/* starting the concentrator */ //TODO
	rfconf.freq_hz = f_target[0];
	lgw_rxrf_setconf(0, rfconf);
	rfconf.freq_hz = f_target[1];
	lgw_rxrf_setconf(1, rfconf);

	i = lgw_start();
	if (i == LGW_HAL_SUCCESS) {
		MSG("INFO: concentrator started, packet can be sent\n");
	} else {
		MSG("ERROR: failed to start the concentrator\n");
		return EXIT_FAILURE;
	}

	/* fill-up payload and parameters */
	memset(&txpkt, 0, sizeof(txpkt));
	txpkt.freq_hz = f_target[0];
	txpkt.tx_mode = IMMEDIATE;
	txpkt.rf_chain = 0;
	txpkt.rf_power = pow;
	txpkt.modulation = MOD_LORA;
	switch (bw) {
		case 125: txpkt.bandwidth = BW_125KHZ; break;
		case 250: txpkt.bandwidth = BW_250KHZ; break;
		case 500: txpkt.bandwidth = BW_500KHZ; break;
		default:
			MSG("ERROR: invalid 'bw' variable\n");
			return EXIT_FAILURE;
	}
	switch (sf) {
		case  7: txpkt.datarate = DR_LORA_SF7;  break;
		case  8: txpkt.datarate = DR_LORA_SF8;  break;
		case  9: txpkt.datarate = DR_LORA_SF9;  break;
		case 10: txpkt.datarate = DR_LORA_SF10; break;
		case 11: txpkt.datarate = DR_LORA_SF11; break;
		case 12: txpkt.datarate = DR_LORA_SF12; break;
		default:
			MSG("ERROR: invalid 'sf' variable\n");
			return EXIT_FAILURE;
	}
	txpkt.coderate = CR_LORA_4_5;
	txpkt.invert_pol = invert;
	txpkt.preamble = preamb;
	txpkt.size = pl_size;
	strcpy((char *)txpkt.payload, "TEST**abcdefghijklmnopqrstuvwxyz0123456789" ); /* abc.. is for padding */

	/* main loop */
	cycle_count = 0;
	while ((repeat == -1) || (cycle_count < repeat)) {
		++cycle_count;

		/* refresh counters in payload (big endian, for readability) */
		txpkt.payload[4] = (uint8_t)(cycle_count >> 8); /* MSB */
		txpkt.payload[5] = (uint8_t)(cycle_count & 0x00FF); /* LSB */

		/* send packet */
		printf("Sending packet number %u ...", cycle_count);
		if( rf_chain_enable & 0x01){
			txpkt.rf_chain = 0;
			txpkt.freq_hz = f_target[0];
			i = lgw_send(txpkt); /* non-blocking scheduling of TX packet */
			if (i != LGW_HAL_SUCCESS) {
				printf("ERROR\n");
				return EXIT_FAILURE;
			}
			/* wait for packet to finish sending */
			do {
				wait_ms(5);
				lgw_status(TX_STATUS, &status_var); /* get TX status */
			} while (status_var != TX_FREE);
			printf("OK\n");
		}

		if( rf_chain_enable & 0x02){
			txpkt.rf_chain = 1;
			txpkt.freq_hz = f_target[1];
			i = lgw_send(txpkt); /* non-blocking scheduling of TX packet */
			if (i != LGW_HAL_SUCCESS) {
				printf("ERROR\n");
				return EXIT_FAILURE;
			}

			/* wait for packet to finish sending */
			do {
				wait_ms(5);
				lgw_status(TX_STATUS, &status_var); /* get TX status */
			} while (status_var != TX_FREE);
			printf("OK\n");
		}

		/* wait inter-packet delay */
		wait_ms(delay);

		/* exit loop on user signals */
		if ((quit_sig == 1) || (exit_sig == 1)) {
			break;
		}
	}

	/* clean up before leaving */
	lgw_stop();

	printf("Exiting LoRa concentrator TX test program\n");
	return EXIT_SUCCESS;
}

/* --- EOF ------------------------------------------------------------------ */
