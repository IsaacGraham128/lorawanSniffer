/*
MIT License

Copyright (c) 2022 IsaacGraham128

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

/* fix an issue between POSIX and C99 */
#if __STDC_VERSION__ >= 199901L
    #define _XOPEN_SOURCE 700
#else
    #define _XOPEN_SOURCE 500
#endif

#include <stdint.h>     /* C99 types */
#include <stdbool.h>    /* bool type */
#include <stdio.h>      /* printf fprintf sprintf fopen fputs getline */

#include <string.h>     /* memset */
#include <signal.h>     /* sigaction */
#include <time.h>       /* time clock_gettime strftime gmtime clock_nanosleep*/
#include <unistd.h>     /* getopt access fork */
#include <stdlib.h>     /* atoi, malloc */
#include <errno.h>      /* error messages */
#include <math.h>       /* round */

#include <pthread.h>
#include <sys/queue.h>  /* STAILQ queue */
#include <sys/sendfile.h>

#include <ctype.h>      /* isdigit */

#include "parson.h"
#include "base64.h"
#include "loragw_hal.h"
#include "loragw_aux.h"
#include "loragw_gps.h"

/* Includes for server functionality */
#include <netinet/in.h>
#include <sys/socket.h>
#include <asm-generic/socket.h>

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a)   (sizeof(a) / sizeof((a)[0]))

/* message macros that utilise the wider logging function */
#define MSG_INFO(format, ...)   print_log("INFO: " format __VA_OPT__(,) __VA_ARGS__)
#define MSG_WARN(format, ...)   print_log("WARNING: " format __VA_OPT__(,) __VA_ARGS__)
#define MSG_ERR(format, ...)    print_log("ERROR: " format __VA_OPT__(,) __VA_ARGS__)
#define MSG_LOG(format, ...)    print_log("LOG: " format __VA_OPT__(,) __VA_ARGS__)

/* Logging macro function */
/* Modified to match kibana visualisation style */
#define print_log(format, ...) {                                                                    \
    int i = 0;                                                                                      \
    char time_str[40] = {'\0'};                                                                     \
    log_file = fopen(log_file_name, "a");                                                           \
    struct timespec local_time;                                                                     \
    clock_gettime(CLOCK_REALTIME, &local_time);                                                     \
    strftime(time_str, ARRAY_SIZE(time_str),"%b %d, %Y @ %H:%M:%S", localtime(&local_time.tv_sec)); \
    if (verbose) {                                                                                  \
        fprintf(stdout, format __VA_OPT__(,) __VA_ARGS__);                                          \
    }                                                                                               \
    fprintf(log_file, "%s - " format , time_str __VA_OPT__(,) __VA_ARGS__);                         \
    i = fclose(log_file);                                                                           \
    if (i < 0) {                                                                                    \
        printf("Failed to close log file\n");                                                       \
        sniffer_exit();                                                                             \
    }                                                                                               \
}

/* -------------------------------------------------------------------------- */
/* --- PRIVATE TYPES -------------------------------------------------------- */

/* spectral scan */
typedef struct spectral_scan_s {
    bool enable;            /* enable spectral scan thread */
    uint32_t freq_hz_start; /* first channel frequency, in Hz */
    uint8_t nb_chan;        /* number of channels to scan (200kHz between each channel) */
    uint16_t nb_scan;       /* number of scan points for each frequency scan */
    uint32_t pace_s;        /* number of seconds between 2 scans in the thread */
} spectral_scan_t;                                                                                                   \

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#ifndef VERSION_STRING
    #define VERSION_STRING "undefined"
#endif

#define OPTION_ARGS         ":acdhv"

#define JSON_CONF_DEFAULT   "conf_server.json"

#define PORT                    8000

#define BITS_PREAMBLE_N_SYNC    98
#define BITS_PHDR_N_CRC         64

#define BITRATE_DR0             250         /* Bitrate(bit/sec) for SF12@125KHz*/
#define BITRATE_DR1             440         /* Bitrate(bit/sec) for SF11@125KHz*/
#define BITRATE_DR2             980         /* Bitrate(bit/sec) for SF10@125KHz*/
#define BITRATE_DR3             1760        /* Bitrate(bit/sec) for SF9@125KHz*/
#define BITRATE_DR4             3125        /* Bitrate(bit/sec) for SF8@125KHz*/
#define BITRATE_DR5             5470        /* Bitrate(bit/sec) for SF7@125KHz*/

/* signal handling variables */
struct sigaction sigact; /* SIGQUIT&SIGINT&SIGTERM signal handling */
volatile bool exit_sig = false; /* 1 -> application terminates cleanly (shut down hardware, close open files, etc) */
volatile bool quit_sig = false; /* 1 -> application terminates without shutting down the hardware */

/* hardware access control and correction */
pthread_mutex_t mx_concent = PTHREAD_MUTEX_INITIALIZER; /* control access to the concentrator */

/* Gateway specificities */
static int8_t antenna_gain = 0;

/* TX capabilities */
static struct lgw_tx_gain_lut_s txlut[LGW_RF_CHAIN_NB]; /* TX gain table */
static uint32_t tx_freq_min[LGW_RF_CHAIN_NB]; /* lowest frequency supported by TX chain */
static uint32_t tx_freq_max[LGW_RF_CHAIN_NB]; /* highest frequency supported by TX chain */
static bool tx_enable[LGW_RF_CHAIN_NB] = {false}; /* Is TX enabled for a given RF chain ? */

/* Interface type */
static lgw_com_type_t com_type = LGW_COM_SPI;

/* Logging stuff */
static bool verbose = false;
static FILE * log_file = NULL;
static char log_file_name[128];

static spectral_scan_t spectral_scan_params = {
    .enable = false,
    .freq_hz_start = 0,
    .nb_chan = 0,
    .nb_scan = 0,
    .pace_s = 10
};

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DECLARATION ---------------------------------------- */

/* General runtime functions */
static void usage(void);

static void sig_handler(int sigio);

/* Auxilliary help functions */

static int sniffer_start(void);

static int sniffer_stop(void);

static void sniffer_exit(void);

/* Configuration parsing files */
static int parse_SX130x_configuration(const char * conf_file);

/* Log file interaction */
static void log_open (char* file_name);

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS  ----------------------------------------- */

static void usage( void )
{
    printf("~~~ Library version string~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
    printf(" %s\n", lgw_version_info());
    printf("~~~ Available options ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
    printf(" -a keep all logs\n");
    printf(" -c <filename>  use config file other than 'conf.json'\n");
    printf(" -d create process as daemon\n");
    printf(" -h print this help\n");
    printf(" -v print all log messages to stdout\n");
    printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
}

/**
 * Handle any SIG interrupts.
 */ 
static void sig_handler(int sigio) {
    if (sigio == SIGQUIT) {
        MSG_INFO("SIGQUIT Recieved\n");
        quit_sig = true;
    } else if ((sigio == SIGINT) || (sigio == SIGTERM)) {
        MSG_INFO("SIGINT/TERM Recieved\n");
        exit_sig = true;
    } else {
        MSG_INFO("Unknown signal given\n");
    }
    return;
}

/**
 * Wrapper function for starting concentrator. Prints stuff nicely :)
 * 
 * @return  -1 on failure, otherwise 0
 */
static int sniffer_start(void) {

    int i;

    if (com_type == LGW_COM_SPI) {
        /* Board reset */
        if (system("./reset_lgw.sh start") != 0) {
            printf("ERROR: failed to reset SX1302, check your reset_lgw.sh script\n");
            exit(EXIT_FAILURE);
        }
    }

    i = lgw_start();

    if (i == LGW_HAL_SUCCESS) {
        MSG_INFO("concentrator started, packet can now be received\n");
    } else {
        MSG_ERR("failed to start the concentrator\n");
        return -1;
    }

    return 0;
}

/**
 * Wrapper function for starting concentrator. Prints stuff nicely :)
 * 
 * @return  -1 on failure, otherwise 0
 */
static int sniffer_stop(void) {

    int i;

    i = lgw_stop();

    if (i == LGW_HAL_SUCCESS) {
        MSG_INFO("Concentrator stopped successfully\n");
    } else {
        MSG_WARN("Failed to stop concentrator successfully\n");
        return -1;
    }

    if (com_type == LGW_COM_SPI) {
        /* Board reset */
        if (system("./reset_lgw.sh stop") != 0) {
            printf("ERROR: failed to reset SX1302, check your reset_lgw.sh script\n");
            exit(EXIT_FAILURE);
        }
    }

    return 0;
}

/**
 * Special exiting function. Turns of the gateway concentrator.
 * 
 * Is it needed? Idk, just nice I guess...
*/
static void sniffer_exit(void) {

    sniffer_stop();
    exit(EXIT_FAILURE);
}

static int parse_SX130x_configuration(const char * conf_file) {
    int i, j, number;
    char param_name[32]; /* used to generate variable parameter names */
    const char *str; /* used to store string value from JSON object */
    const char conf_obj_name[] = "SX130x_conf";
    JSON_Value *root_val = NULL;
    JSON_Value *val = NULL;
    JSON_Object *conf_obj = NULL;
    JSON_Object *conf_txgain_obj;
    JSON_Object *conf_ts_obj;
    JSON_Object *conf_sx1261_obj = NULL;
    JSON_Object *conf_scan_obj = NULL;
    JSON_Object *conf_lbt_obj = NULL;
    JSON_Object *conf_lbtchan_obj = NULL;
    JSON_Array *conf_txlut_array = NULL;
    JSON_Array *conf_lbtchan_array = NULL;
    JSON_Array *conf_demod_array = NULL;

    struct lgw_conf_board_s boardconf;
    struct lgw_conf_rxrf_s rfconf;
    struct lgw_conf_rxif_s ifconf;
    struct lgw_conf_demod_s demodconf;
    struct lgw_conf_ftime_s tsconf;
    struct lgw_conf_sx1261_s sx1261conf;
    uint32_t sf, bw, fdev;
    bool sx1250_tx_lut;
    size_t size;

    /* try to parse JSON */
    root_val = json_parse_file_with_comments(conf_file);
    if (root_val == NULL) {
        MSG_ERR("ERROR: %s is not a valid JSON file\n", conf_file);
        exit(EXIT_FAILURE);
    }

    /* point to the gateway configuration object */
    conf_obj = json_object_get_object(json_value_get_object(root_val), conf_obj_name);
    if (conf_obj == NULL) {
        MSG_ERR("INFO: %s does not contain a JSON object named %s\n", conf_file, conf_obj_name);
        return -1;
    } else {
        MSG_ERR("INFO: %s does contain a JSON object named %s, parsing SX1302 parameters\n", conf_file, conf_obj_name);
    }

    /* set board configuration */
    memset(&boardconf, 0, sizeof boardconf); /* initialize configuration structure */
    str = json_object_get_string(conf_obj, "com_type");
    if (str == NULL) {
        MSG_ERR("ERROR: com_type must be configured in %s\n", conf_file);
        return -1;
    } else if (!strncmp(str, "SPI", 3) || !strncmp(str, "spi", 3)) {
        boardconf.com_type = LGW_COM_SPI;
    } else if (!strncmp(str, "USB", 3) || !strncmp(str, "usb", 3)) {
        boardconf.com_type = LGW_COM_USB;
    } else {
        MSG_ERR("ERROR: invalid com type: %s (should be SPI or USB)\n", str);
        return -1;
    }
    com_type = boardconf.com_type;
    str = json_object_get_string(conf_obj, "com_path");
    if (str != NULL) {
        strncpy(boardconf.com_path, str, sizeof boardconf.com_path);
        boardconf.com_path[sizeof boardconf.com_path - 1] = '\0'; /* ensure string termination */
    } else {
        MSG_ERR("ERROR: com_path must be configured in %s\n", conf_file);
        return -1;
    }
    val = json_object_get_value(conf_obj, "lorawan_public"); /* fetch value (if possible) */
    if (json_value_get_type(val) == JSONBoolean) {
        boardconf.lorawan_public = (bool)json_value_get_boolean(val);
    } else {
        MSG_WARN("WARNING: Data type for lorawan_public seems wrong, please check\n");
        boardconf.lorawan_public = false;
    }
    val = json_object_get_value(conf_obj, "clksrc"); /* fetch value (if possible) */
    if (json_value_get_type(val) == JSONNumber) {
        boardconf.clksrc = (uint8_t)json_value_get_number(val);
    } else {
        MSG_WARN("WARNING: Data type for clksrc seems wrong, please check\n");
        boardconf.clksrc = 0;
    }
    val = json_object_get_value(conf_obj, "full_duplex"); /* fetch value (if possible) */
    if (json_value_get_type(val) == JSONBoolean) {
        boardconf.full_duplex = (bool)json_value_get_boolean(val);
    } else {
        MSG_WARN("WARNING: Data type for full_duplex seems wrong, please check\n");
        boardconf.full_duplex = false;
    }
    MSG_WARN("INFO: com_type %s, com_path %s, lorawan_public %d, clksrc %d, full_duplex %d\n", (boardconf.com_type == LGW_COM_SPI) ? "SPI" : "USB", boardconf.com_path, boardconf.lorawan_public, boardconf.clksrc, boardconf.full_duplex);
    /* all parameters parsed, submitting configuration to the HAL */
    if (lgw_board_setconf(&boardconf) != LGW_HAL_SUCCESS) {
        MSG_WARN("ERROR: Failed to configure board\n");
        return -1;
    }

    /* set antenna gain configuration */
    val = json_object_get_value(conf_obj, "antenna_gain"); /* fetch value (if possible) */
    if (val != NULL) {
        if (json_value_get_type(val) == JSONNumber) {
            antenna_gain = (int8_t)json_value_get_number(val);
        } else {
            MSG_WARN("WARNING: Data type for antenna_gain seems wrong, please check\n");
            antenna_gain = 0;
        }
    }
    MSG_WARN("INFO: antenna_gain %d dBi\n", antenna_gain);

    /* set timestamp configuration */
    conf_ts_obj = json_object_get_object(conf_obj, "fine_timestamp");
    if (conf_ts_obj == NULL) {
        MSG_WARN("INFO: %s does not contain a JSON object for fine timestamp\n", conf_file);
    } else {
        val = json_object_get_value(conf_ts_obj, "enable"); /* fetch value (if possible) */
        if (json_value_get_type(val) == JSONBoolean) {
            tsconf.enable = (bool)json_value_get_boolean(val);
        } else {
            MSG_WARN("WARNING: Data type for fine_timestamp.enable seems wrong, please check\n");
            tsconf.enable = false;
        }
        if (tsconf.enable == true) {
            str = json_object_get_string(conf_ts_obj, "mode");
            if (str == NULL) {
                MSG_WARN("ERROR: fine_timestamp.mode must be configured in %s\n", conf_file);
                return -1;
            } else if (!strncmp(str, "high_capacity", 13) || !strncmp(str, "HIGH_CAPACITY", 13)) {
                tsconf.mode = LGW_FTIME_MODE_HIGH_CAPACITY;
            } else if (!strncmp(str, "all_sf", 6) || !strncmp(str, "ALL_SF", 6)) {
                tsconf.mode = LGW_FTIME_MODE_ALL_SF;
            } else {
                MSG_WARN("ERROR: invalid fine timestamp mode: %s (should be high_capacity or all_sf)\n", str);
                return -1;
            }
            MSG_WARN("INFO: Configuring precision timestamp with %s mode\n", str);

            /* all parameters parsed, submitting configuration to the HAL */
            if (lgw_ftime_setconf(&tsconf) != LGW_HAL_SUCCESS) {
                MSG_WARN("ERROR: Failed to configure fine timestamp\n");
                return -1;
            }
        } else {
            MSG_WARN("INFO: Configuring legacy timestamp\n");
        }
    }

    /* set SX1261 configuration */
    memset(&sx1261conf, 0, sizeof sx1261conf); /* initialize configuration structure */
    conf_sx1261_obj = json_object_get_object(conf_obj, "sx1261_conf"); /* fetch value (if possible) */
    if (conf_sx1261_obj == NULL) {
        MSG_WARN("INFO: no configuration for SX1261\n");
    } else {
        /* Global SX1261 configuration */
        str = json_object_get_string(conf_sx1261_obj, "spi_path");
        if (str != NULL) {
            strncpy(sx1261conf.spi_path, str, sizeof sx1261conf.spi_path);
            sx1261conf.spi_path[sizeof sx1261conf.spi_path - 1] = '\0'; /* ensure string termination */
        } else {
            MSG_WARN("INFO: SX1261 spi_path is not configured in %s\n", conf_file);
        }
        val = json_object_get_value(conf_sx1261_obj, "rssi_offset"); /* fetch value (if possible) */
        if (json_value_get_type(val) == JSONNumber) {
            sx1261conf.rssi_offset = (int8_t)json_value_get_number(val);
        } else {
            MSG_WARN("WARNING: Data type for sx1261_conf.rssi_offset seems wrong, please check\n");
            sx1261conf.rssi_offset = 0;
        }

        /* Spectral Scan configuration */
        conf_scan_obj = json_object_get_object(conf_sx1261_obj, "spectral_scan"); /* fetch value (if possible) */
        if (conf_scan_obj == NULL) {
            MSG_WARN("INFO: no configuration for Spectral Scan\n");
        } else {
            val = json_object_get_value(conf_scan_obj, "enable"); /* fetch value (if possible) */
            if (json_value_get_type(val) == JSONBoolean) {
                /* Enable background spectral scan thread in packet forwarder */
                spectral_scan_params.enable = (bool)json_value_get_boolean(val);
            } else {
                MSG_WARN("WARNING: Data type for spectral_scan.enable seems wrong, please check\n");
            }
            if (spectral_scan_params.enable == true) {
                /* Enable the sx1261 radio hardware configuration to allow spectral scan */
                sx1261conf.enable = true;
                MSG_WARN("INFO: Spectral Scan with SX1261 is enabled\n");

                /* Get Spectral Scan Parameters */
                val = json_object_get_value(conf_scan_obj, "freq_start"); /* fetch value (if possible) */
                if (json_value_get_type(val) == JSONNumber) {
                    spectral_scan_params.freq_hz_start = (uint32_t)json_value_get_number(val);
                } else {
                    MSG_WARN("WARNING: Data type for spectral_scan.freq_start seems wrong, please check\n");
                }
                val = json_object_get_value(conf_scan_obj, "nb_chan"); /* fetch value (if possible) */
                if (json_value_get_type(val) == JSONNumber) {
                    spectral_scan_params.nb_chan = (uint8_t)json_value_get_number(val);
                } else {
                    MSG_WARN("WARNING: Data type for spectral_scan.nb_chan seems wrong, please check\n");
                }
                val = json_object_get_value(conf_scan_obj, "nb_scan"); /* fetch value (if possible) */
                if (json_value_get_type(val) == JSONNumber) {
                    spectral_scan_params.nb_scan = (uint16_t)json_value_get_number(val);
                } else {
                    MSG_WARN("WARNING: Data type for spectral_scan.nb_scan seems wrong, please check\n");
                }
                val = json_object_get_value(conf_scan_obj, "pace_s"); /* fetch value (if possible) */
                if (json_value_get_type(val) == JSONNumber) {
                    spectral_scan_params.pace_s = (uint32_t)json_value_get_number(val);
                } else {
                    MSG_WARN("WARNING: Data type for spectral_scan.pace_s seems wrong, please check\n");
                }
            }
        }

        /* LBT configuration */
        conf_lbt_obj = json_object_get_object(conf_sx1261_obj, "lbt"); /* fetch value (if possible) */
        if (conf_lbt_obj == NULL) {
            MSG_WARN("INFO: no configuration for LBT\n");
        } else {
            val = json_object_get_value(conf_lbt_obj, "enable"); /* fetch value (if possible) */
            if (json_value_get_type(val) == JSONBoolean) {
                sx1261conf.lbt_conf.enable = (bool)json_value_get_boolean(val);
            } else {
                MSG_WARN("WARNING: Data type for lbt.enable seems wrong, please check\n");
            }
            if (sx1261conf.lbt_conf.enable == true) {
                /* Enable the sx1261 radio hardware configuration to allow spectral scan */
                sx1261conf.enable = true;
                MSG_WARN("INFO: Listen-Before-Talk with SX1261 is enabled\n");

                val = json_object_get_value(conf_lbt_obj, "rssi_target"); /* fetch value (if possible) */
                if (json_value_get_type(val) == JSONNumber) {
                    sx1261conf.lbt_conf.rssi_target = (int8_t)json_value_get_number(val);
                } else {
                    MSG_WARN("WARNING: Data type for lbt.rssi_target seems wrong, please check\n");
                    sx1261conf.lbt_conf.rssi_target = 0;
                }
                /* set LBT channels configuration */
                conf_lbtchan_array = json_object_get_array(conf_lbt_obj, "channels");
                if (conf_lbtchan_array != NULL) {
                    sx1261conf.lbt_conf.nb_channel = json_array_get_count(conf_lbtchan_array);
                    MSG_WARN("INFO: %u LBT channels configured\n", sx1261conf.lbt_conf.nb_channel);
                }
                for (i = 0; i < (int)sx1261conf.lbt_conf.nb_channel; i++) {
                    /* Sanity check */
                    if (i >= LGW_LBT_CHANNEL_NB_MAX) {
                        MSG_WARN("ERROR: LBT channel %d not supported, skip it\n", i);
                        break;
                    }
                    /* Get LBT channel configuration object from array */
                    conf_lbtchan_obj = json_array_get_object(conf_lbtchan_array, i);

                    /* Channel frequency */
                    val = json_object_dotget_value(conf_lbtchan_obj, "freq_hz"); /* fetch value (if possible) */
                    if (val != NULL) {
                        if (json_value_get_type(val) == JSONNumber) {
                            sx1261conf.lbt_conf.channels[i].freq_hz = (uint32_t)json_value_get_number(val);
                        } else {
                            MSG_WARN("WARNING: Data type for lbt.channels[%d].freq_hz seems wrong, please check\n", i);
                            sx1261conf.lbt_conf.channels[i].freq_hz = 0;
                        }
                    } else {
                        MSG_WARN("ERROR: no frequency defined for LBT channel %d\n", i);
                        return -1;
                    }

                    /* Channel bandiwdth */
                    val = json_object_dotget_value(conf_lbtchan_obj, "bandwidth"); /* fetch value (if possible) */
                    if (val != NULL) {
                        if (json_value_get_type(val) == JSONNumber) {
                            bw = (uint32_t)json_value_get_number(val);
                            switch(bw) {
                                case 500000: sx1261conf.lbt_conf.channels[i].bandwidth = BW_500KHZ; break;
                                case 250000: sx1261conf.lbt_conf.channels[i].bandwidth = BW_250KHZ; break;
                                case 125000: sx1261conf.lbt_conf.channels[i].bandwidth = BW_125KHZ; break;
                                default: sx1261conf.lbt_conf.channels[i].bandwidth = BW_UNDEFINED;
                            }
                        } else {
                            MSG_WARN("WARNING: Data type for lbt.channels[%d].freq_hz seems wrong, please check\n", i);
                            sx1261conf.lbt_conf.channels[i].bandwidth = BW_UNDEFINED;
                        }
                    } else {
                        MSG_WARN("ERROR: no bandiwdth defined for LBT channel %d\n", i);
                        return -1;
                    }

                    /* Channel scan time */
                    val = json_object_dotget_value(conf_lbtchan_obj, "scan_time_us"); /* fetch value (if possible) */
                    if (val != NULL) {
                        if (json_value_get_type(val) == JSONNumber) {
                            if ((uint16_t)json_value_get_number(val) == 128) {
                                sx1261conf.lbt_conf.channels[i].scan_time_us = LGW_LBT_SCAN_TIME_128_US;
                            } else if ((uint16_t)json_value_get_number(val) == 5000) {
                                sx1261conf.lbt_conf.channels[i].scan_time_us = LGW_LBT_SCAN_TIME_5000_US;
                            } else {
                                MSG_WARN("ERROR: scan time not supported for LBT channel %d, must be 128 or 5000\n", i);
                                return -1;
                            }
                        } else {
                            MSG_WARN("WARNING: Data type for lbt.channels[%d].scan_time_us seems wrong, please check\n", i);
                            sx1261conf.lbt_conf.channels[i].scan_time_us = 0;
                        }
                    } else {
                        MSG_WARN("ERROR: no scan_time_us defined for LBT channel %d\n", i);
                        return -1;
                    }

                    /* Channel transmit time */
                    val = json_object_dotget_value(conf_lbtchan_obj, "transmit_time_ms"); /* fetch value (if possible) */
                    if (val != NULL) {
                        if (json_value_get_type(val) == JSONNumber) {
                            sx1261conf.lbt_conf.channels[i].transmit_time_ms = (uint16_t)json_value_get_number(val);
                        } else {
                            MSG_WARN("WARNING: Data type for lbt.channels[%d].transmit_time_ms seems wrong, please check\n", i);
                            sx1261conf.lbt_conf.channels[i].transmit_time_ms = 0;
                        }
                    } else {
                        MSG_WARN("ERROR: no transmit_time_ms defined for LBT channel %d\n", i);
                        return -1;
                    }
                }
            }
        }

        /* all parameters parsed, submitting configuration to the HAL */
        if (lgw_sx1261_setconf(&sx1261conf) != LGW_HAL_SUCCESS) {
            MSG_WARN("ERROR: Failed to configure the SX1261 radio\n");
            return -1;
        }
    }

    /* set configuration for RF chains */
    for (i = 0; i < LGW_RF_CHAIN_NB; ++i) {
        memset(&rfconf, 0, sizeof rfconf); /* initialize configuration structure */
        snprintf(param_name, sizeof param_name, "radio_%i", i); /* compose parameter path inside JSON structure */
        val = json_object_get_value(conf_obj, param_name); /* fetch value (if possible) */
        if (json_value_get_type(val) != JSONObject) {
            MSG_WARN("INFO: no configuration for radio %i\n", i);
            continue;
        }
        /* there is an object to configure that radio, let's parse it */
        snprintf(param_name, sizeof param_name, "radio_%i.enable", i);
        val = json_object_dotget_value(conf_obj, param_name);
        if (json_value_get_type(val) == JSONBoolean) {
            rfconf.enable = (bool)json_value_get_boolean(val);
        } else {
            rfconf.enable = false;
        }
        if (rfconf.enable == false) { /* radio disabled, nothing else to parse */
            MSG_WARN("INFO: radio %i disabled\n", i);
        } else  { /* radio enabled, will parse the other parameters */
            snprintf(param_name, sizeof param_name, "radio_%i.freq", i);
            rfconf.freq_hz = (uint32_t)json_object_dotget_number(conf_obj, param_name);
            snprintf(param_name, sizeof param_name, "radio_%i.rssi_offset", i);
            rfconf.rssi_offset = (float)json_object_dotget_number(conf_obj, param_name);
            snprintf(param_name, sizeof param_name, "radio_%i.rssi_tcomp.coeff_a", i);
            rfconf.rssi_tcomp.coeff_a = (float)json_object_dotget_number(conf_obj, param_name);
            snprintf(param_name, sizeof param_name, "radio_%i.rssi_tcomp.coeff_b", i);
            rfconf.rssi_tcomp.coeff_b = (float)json_object_dotget_number(conf_obj, param_name);
            snprintf(param_name, sizeof param_name, "radio_%i.rssi_tcomp.coeff_c", i);
            rfconf.rssi_tcomp.coeff_c = (float)json_object_dotget_number(conf_obj, param_name);
            snprintf(param_name, sizeof param_name, "radio_%i.rssi_tcomp.coeff_d", i);
            rfconf.rssi_tcomp.coeff_d = (float)json_object_dotget_number(conf_obj, param_name);
            snprintf(param_name, sizeof param_name, "radio_%i.rssi_tcomp.coeff_e", i);
            rfconf.rssi_tcomp.coeff_e = (float)json_object_dotget_number(conf_obj, param_name);
            snprintf(param_name, sizeof param_name, "radio_%i.type", i);
            str = json_object_dotget_string(conf_obj, param_name);
            if (!strncmp(str, "SX1255", 6)) {
                rfconf.type = LGW_RADIO_TYPE_SX1255;
            } else if (!strncmp(str, "SX1257", 6)) {
                rfconf.type = LGW_RADIO_TYPE_SX1257;
            } else if (!strncmp(str, "SX1250", 6)) {
                rfconf.type = LGW_RADIO_TYPE_SX1250;
            } else {
                MSG_WARN("WARNING: invalid radio type: %s (should be SX1255 or SX1257 or SX1250)\n", str);
            }
            snprintf(param_name, sizeof param_name, "radio_%i.single_input_mode", i);
            val = json_object_dotget_value(conf_obj, param_name);
            if (json_value_get_type(val) == JSONBoolean) {
                rfconf.single_input_mode = (bool)json_value_get_boolean(val);
            } else {
                rfconf.single_input_mode = false;
            }

            snprintf(param_name, sizeof param_name, "radio_%i.tx_enable", i);
            val = json_object_dotget_value(conf_obj, param_name);
            if (json_value_get_type(val) == JSONBoolean) {
                rfconf.tx_enable = (bool)json_value_get_boolean(val);
                tx_enable[i] = rfconf.tx_enable; /* update global context for later check */
                if (rfconf.tx_enable == true) {
                    /* tx is enabled on this rf chain, we need its frequency range */
                    snprintf(param_name, sizeof param_name, "radio_%i.tx_freq_min", i);
                    tx_freq_min[i] = (uint32_t)json_object_dotget_number(conf_obj, param_name);
                    snprintf(param_name, sizeof param_name, "radio_%i.tx_freq_max", i);
                    tx_freq_max[i] = (uint32_t)json_object_dotget_number(conf_obj, param_name);
                    if ((tx_freq_min[i] == 0) || (tx_freq_max[i] == 0)) {
                        MSG_WARN("WARNING: no frequency range specified for TX rf chain %d\n", i);
                    }

                    /* set configuration for tx gains */
                    memset(&txlut[i], 0, sizeof txlut[i]); /* initialize configuration structure */
                    snprintf(param_name, sizeof param_name, "radio_%i.tx_gain_lut", i);
                    conf_txlut_array = json_object_dotget_array(conf_obj, param_name);
                    if (conf_txlut_array != NULL) {
                        txlut[i].size = json_array_get_count(conf_txlut_array);
                        /* Detect if we have a sx125x or sx1250 configuration */
                        conf_txgain_obj = json_array_get_object(conf_txlut_array, 0);
                        val = json_object_dotget_value(conf_txgain_obj, "pwr_idx");
                        if (val != NULL) {
                            printf("INFO: Configuring Tx Gain LUT for rf_chain %u with %u indexes for sx1250\n", i, txlut[i].size);
                            sx1250_tx_lut = true;
                        } else {
                            printf("INFO: Configuring Tx Gain LUT for rf_chain %u with %u indexes for sx125x\n", i, txlut[i].size);
                            sx1250_tx_lut = false;
                        }
                        /* Parse the table */
                        for (j = 0; j < (int)txlut[i].size; j++) {
                             /* Sanity check */
                            if (j >= TX_GAIN_LUT_SIZE_MAX) {
                                printf("ERROR: TX Gain LUT [%u] index %d not supported, skip it\n", i, j);
                                break;
                            }
                            /* Get TX gain object from LUT */
                            conf_txgain_obj = json_array_get_object(conf_txlut_array, j);
                            /* rf power */
                            val = json_object_dotget_value(conf_txgain_obj, "rf_power");
                            if (json_value_get_type(val) == JSONNumber) {
                                txlut[i].lut[j].rf_power = (int8_t)json_value_get_number(val);
                            } else {
                                printf("WARNING: Data type for %s[%d] seems wrong, please check\n", "rf_power", j);
                                txlut[i].lut[j].rf_power = 0;
                            }
                            /* PA gain */
                            val = json_object_dotget_value(conf_txgain_obj, "pa_gain");
                            if (json_value_get_type(val) == JSONNumber) {
                                txlut[i].lut[j].pa_gain = (uint8_t)json_value_get_number(val);
                            } else {
                                printf("WARNING: Data type for %s[%d] seems wrong, please check\n", "pa_gain", j);
                                txlut[i].lut[j].pa_gain = 0;
                            }
                            if (sx1250_tx_lut == false) {
                                /* DIG gain */
                                val = json_object_dotget_value(conf_txgain_obj, "dig_gain");
                                if (json_value_get_type(val) == JSONNumber) {
                                    txlut[i].lut[j].dig_gain = (uint8_t)json_value_get_number(val);
                                } else {
                                    printf("WARNING: Data type for %s[%d] seems wrong, please check\n", "dig_gain", j);
                                    txlut[i].lut[j].dig_gain = 0;
                                }
                                /* DAC gain */
                                val = json_object_dotget_value(conf_txgain_obj, "dac_gain");
                                if (json_value_get_type(val) == JSONNumber) {
                                    txlut[i].lut[j].dac_gain = (uint8_t)json_value_get_number(val);
                                } else {
                                    printf("WARNING: Data type for %s[%d] seems wrong, please check\n", "dac_gain", j);
                                    txlut[i].lut[j].dac_gain = 3; /* This is the only dac_gain supported for now */
                                }
                                /* MIX gain */
                                val = json_object_dotget_value(conf_txgain_obj, "mix_gain");
                                if (json_value_get_type(val) == JSONNumber) {
                                    txlut[i].lut[j].mix_gain = (uint8_t)json_value_get_number(val);
                                } else {
                                    printf("WARNING: Data type for %s[%d] seems wrong, please check\n", "mix_gain", j);
                                    txlut[i].lut[j].mix_gain = 0;
                                }
                            } else {
                                /* TODO: rework this, should not be needed for sx1250 */
                                txlut[i].lut[j].mix_gain = 5;

                                /* power index */
                                val = json_object_dotget_value(conf_txgain_obj, "pwr_idx");
                                if (json_value_get_type(val) == JSONNumber) {
                                    txlut[i].lut[j].pwr_idx = (uint8_t)json_value_get_number(val);
                                } else {
                                    printf("WARNING: Data type for %s[%d] seems wrong, please check\n", "pwr_idx", j);
                                    txlut[i].lut[j].pwr_idx = 0;
                                }
                            }
                        }
                        /* all parameters parsed, submitting configuration to the HAL */
                        if (txlut[i].size > 0) {
                            if (lgw_txgain_setconf(i, &txlut[i]) != LGW_HAL_SUCCESS) {
                                MSG_WARN("ERROR: Failed to configure concentrator TX Gain LUT for rf_chain %u\n", i);
                                return -1;
                            }
                        } else {
                            MSG_WARN("WARNING: No TX gain LUT defined for rf_chain %u\n", i);
                        }
                    } else {
                        MSG_WARN("WARNING: No TX gain LUT defined for rf_chain %u\n", i);
                    }
                }
            } else {
                rfconf.tx_enable = false;
            }
            MSG_WARN("INFO: radio %i enabled (type %s), center frequency %u, RSSI offset %f, tx enabled %d, single input mode %d\n", i, str, rfconf.freq_hz, rfconf.rssi_offset, rfconf.tx_enable, rfconf.single_input_mode);
        }
        /* all parameters parsed, submitting configuration to the HAL */
        if (lgw_rxrf_setconf(i, &rfconf) != LGW_HAL_SUCCESS) {
            MSG_WARN("ERROR: invalid configuration for radio %i\n", i);
            return -1;
        }
    }

    /* set configuration for demodulators */
    memset(&demodconf, 0, sizeof demodconf); /* initialize configuration structure */
    val = json_object_get_value(conf_obj, "chan_multiSF_All"); /* fetch value (if possible) */
    if (json_value_get_type(val) != JSONObject) {
        MSG_WARN("INFO: no configuration for LoRa multi-SF spreading factors enabling\n");
    } else {
        conf_demod_array = json_object_dotget_array(conf_obj, "chan_multiSF_All.spreading_factor_enable");
        if ((conf_demod_array != NULL) && ((size = json_array_get_count(conf_demod_array)) <= LGW_MULTI_NB)) {
            for (i = 0; i < (int)size; i++) {
                number = json_array_get_number(conf_demod_array, i);
                if (number < 5 || number > 12) {
                    MSG_WARN("WARNING: failed to parse chan_multiSF_All.spreading_factor_enable (wrong value at idx %d)\n", i);
                    demodconf.multisf_datarate = 0xFF; /* enable all SFs */
                    break;
                } else {
                    /* set corresponding bit in the bitmask SF5 is LSB -> SF12 is MSB */
                    demodconf.multisf_datarate |= (1 << (number - 5));
                }
            }
        } else {
            MSG_WARN("WARNING: failed to parse chan_multiSF_All.spreading_factor_enable\n");
            demodconf.multisf_datarate = 0xFF; /* enable all SFs */
        }
        /* all parameters parsed, submitting configuration to the HAL */
        if (lgw_demod_setconf(&demodconf) != LGW_HAL_SUCCESS) {
            MSG_WARN("ERROR: invalid configuration for demodulation parameters\n");
            return -1;
        }
    }

    /* set configuration for Lora multi-SF channels (bandwidth cannot be set) */
    for (i = 0; i < LGW_MULTI_NB; ++i) {
        memset(&ifconf, 0, sizeof ifconf); /* initialize configuration structure */
        snprintf(param_name, sizeof param_name, "chan_multiSF_%i", i); /* compose parameter path inside JSON structure */
        val = json_object_get_value(conf_obj, param_name); /* fetch value (if possible) */
        if (json_value_get_type(val) != JSONObject) {
            MSG_WARN("INFO: no configuration for Lora multi-SF channel %i\n", i);
            continue;
        }
        /* there is an object to configure that Lora multi-SF channel, let's parse it */
        snprintf(param_name, sizeof param_name, "chan_multiSF_%i.enable", i);
        val = json_object_dotget_value(conf_obj, param_name);
        if (json_value_get_type(val) == JSONBoolean) {
            ifconf.enable = (bool)json_value_get_boolean(val);
        } else {
            ifconf.enable = false;
        }
        if (ifconf.enable == false) { /* Lora multi-SF channel disabled, nothing else to parse */
            MSG_WARN("INFO: Lora multi-SF channel %i disabled\n", i);
        } else  { /* Lora multi-SF channel enabled, will parse the other parameters */
            snprintf(param_name, sizeof param_name, "chan_multiSF_%i.radio", i);
            ifconf.rf_chain = (uint32_t)json_object_dotget_number(conf_obj, param_name);
            snprintf(param_name, sizeof param_name, "chan_multiSF_%i.if", i);
            ifconf.freq_hz = (int32_t)json_object_dotget_number(conf_obj, param_name);
            // TODO: handle individual SF enabling and disabling (spread_factor)
            MSG_WARN("INFO: Lora multi-SF channel %i>  radio %i, IF %i Hz, 125 kHz bw, SF 5 to 12\n", i, ifconf.rf_chain, ifconf.freq_hz);
        }
        /* all parameters parsed, submitting configuration to the HAL */
        if (lgw_rxif_setconf(i, &ifconf) != LGW_HAL_SUCCESS) {
            MSG_WARN("ERROR: invalid configuration for Lora multi-SF channel %i\n", i);
            return -1;
        }
    }

    /* set configuration for Lora standard channel */
    memset(&ifconf, 0, sizeof ifconf); /* initialize configuration structure */
    val = json_object_get_value(conf_obj, "chan_Lora_std"); /* fetch value (if possible) */
    if (json_value_get_type(val) != JSONObject) {
        MSG_WARN("INFO: no configuration for Lora standard channel\n");
    } else {
        val = json_object_dotget_value(conf_obj, "chan_Lora_std.enable");
        if (json_value_get_type(val) == JSONBoolean) {
            ifconf.enable = (bool)json_value_get_boolean(val);
        } else {
            ifconf.enable = false;
        }
        if (ifconf.enable == false) {
            MSG_WARN("INFO: Lora standard channel %i disabled\n", i);
        } else  {
            ifconf.rf_chain = (uint32_t)json_object_dotget_number(conf_obj, "chan_Lora_std.radio");
            ifconf.freq_hz = (int32_t)json_object_dotget_number(conf_obj, "chan_Lora_std.if");
            bw = (uint32_t)json_object_dotget_number(conf_obj, "chan_Lora_std.bandwidth");
            switch(bw) {
                case 500000: ifconf.bandwidth = BW_500KHZ; break;
                case 250000: ifconf.bandwidth = BW_250KHZ; break;
                case 125000: ifconf.bandwidth = BW_125KHZ; break;
                default: ifconf.bandwidth = BW_UNDEFINED;
            }
            sf = (uint32_t)json_object_dotget_number(conf_obj, "chan_Lora_std.spread_factor");
            switch(sf) {
                case  5: ifconf.datarate = DR_LORA_SF5;  break;
                case  6: ifconf.datarate = DR_LORA_SF6;  break;
                case  7: ifconf.datarate = DR_LORA_SF7;  break;
                case  8: ifconf.datarate = DR_LORA_SF8;  break;
                case  9: ifconf.datarate = DR_LORA_SF9;  break;
                case 10: ifconf.datarate = DR_LORA_SF10; break;
                case 11: ifconf.datarate = DR_LORA_SF11; break;
                case 12: ifconf.datarate = DR_LORA_SF12; break;
                default: ifconf.datarate = DR_UNDEFINED;
            }
            val = json_object_dotget_value(conf_obj, "chan_Lora_std.implicit_hdr");
            if (json_value_get_type(val) == JSONBoolean) {
                ifconf.implicit_hdr = (bool)json_value_get_boolean(val);
            } else {
                ifconf.implicit_hdr = false;
            }
            if (ifconf.implicit_hdr == true) {
                val = json_object_dotget_value(conf_obj, "chan_Lora_std.implicit_payload_length");
                if (json_value_get_type(val) == JSONNumber) {
                    ifconf.implicit_payload_length = (uint8_t)json_value_get_number(val);
                } else {
                    MSG_WARN("ERROR: payload length setting is mandatory for implicit header mode\n");
                    return -1;
                }
                val = json_object_dotget_value(conf_obj, "chan_Lora_std.implicit_crc_en");
                if (json_value_get_type(val) == JSONBoolean) {
                    ifconf.implicit_crc_en = (bool)json_value_get_boolean(val);
                } else {
                    MSG_WARN("ERROR: CRC enable setting is mandatory for implicit header mode\n");
                    return -1;
                }
                val = json_object_dotget_value(conf_obj, "chan_Lora_std.implicit_coderate");
                if (json_value_get_type(val) == JSONNumber) {
                    ifconf.implicit_coderate = (uint8_t)json_value_get_number(val);
                } else {
                    MSG_WARN("ERROR: coding rate setting is mandatory for implicit header mode\n");
                    return -1;
                }
            }

            MSG_WARN("INFO: Lora std channel> radio %i, IF %i Hz, %u Hz bw, SF %u, %s\n", ifconf.rf_chain, ifconf.freq_hz, bw, sf, (ifconf.implicit_hdr == true) ? "Implicit header" : "Explicit header");
        }
        if (lgw_rxif_setconf(8, &ifconf) != LGW_HAL_SUCCESS) {
            MSG_WARN("ERROR: invalid configuration for Lora standard channel\n");
            return -1;
        }
    }

    /* set configuration for FSK channel */
    memset(&ifconf, 0, sizeof ifconf); /* initialize configuration structure */
    val = json_object_get_value(conf_obj, "chan_FSK"); /* fetch value (if possible) */
    if (json_value_get_type(val) != JSONObject) {
        MSG_WARN("INFO: no configuration for FSK channel\n");
    } else {
        val = json_object_dotget_value(conf_obj, "chan_FSK.enable");
        if (json_value_get_type(val) == JSONBoolean) {
            ifconf.enable = (bool)json_value_get_boolean(val);
        } else {
            ifconf.enable = false;
        }
        if (ifconf.enable == false) {
            MSG_WARN("INFO: FSK channel %i disabled\n", i);
        } else  {
            ifconf.rf_chain = (uint32_t)json_object_dotget_number(conf_obj, "chan_FSK.radio");
            ifconf.freq_hz = (int32_t)json_object_dotget_number(conf_obj, "chan_FSK.if");
            bw = (uint32_t)json_object_dotget_number(conf_obj, "chan_FSK.bandwidth");
            fdev = (uint32_t)json_object_dotget_number(conf_obj, "chan_FSK.freq_deviation");
            ifconf.datarate = (uint32_t)json_object_dotget_number(conf_obj, "chan_FSK.datarate");

            /* if chan_FSK.bandwidth is set, it has priority over chan_FSK.freq_deviation */
            if ((bw == 0) && (fdev != 0)) {
                bw = 2 * fdev + ifconf.datarate;
            }
            if      (bw == 0)      ifconf.bandwidth = BW_UNDEFINED;
#if 0 /* TODO */
            else if (bw <= 7800)   ifconf.bandwidth = BW_7K8HZ;
            else if (bw <= 15600)  ifconf.bandwidth = BW_15K6HZ;
            else if (bw <= 31200)  ifconf.bandwidth = BW_31K2HZ;
            else if (bw <= 62500)  ifconf.bandwidth = BW_62K5HZ;
#endif
            else if (bw <= 125000) ifconf.bandwidth = BW_125KHZ;
            else if (bw <= 250000) ifconf.bandwidth = BW_250KHZ;
            else if (bw <= 500000) ifconf.bandwidth = BW_500KHZ;
            else ifconf.bandwidth = BW_UNDEFINED;

            MSG_WARN("INFO: FSK channel> radio %i, IF %i Hz, %u Hz bw, %u bps datarate\n", ifconf.rf_chain, ifconf.freq_hz, bw, ifconf.datarate);
        }
        if (lgw_rxif_setconf(9, &ifconf) != LGW_HAL_SUCCESS) {
            MSG_WARN("ERROR: invalid configuration for FSK channel\n");
            return -1;
        }
    }
    json_value_free(root_val);

    return 0;
}

/**
 * Open new log file. This log file is written to by all "MSG_" logging functions,
 * regardless of verbose status.
*/
static void log_open (char* file_name) {

    struct timespec local_time;
    char iso_date[40];
    
    clock_gettime(CLOCK_REALTIME, &local_time);
    strftime(iso_date, ARRAY_SIZE(iso_date),"%Y%m%dT%H%M%SZ", localtime(&local_time.tv_sec)); /* format yyyymmddThhmmssZ */

    sprintf(log_file_name, "%s.txt", file_name);
    log_file = fopen(log_file_name, "w"); /* create log file to check if its possible */
    if (log_file == NULL) {
        printf("impossible to create log file %s\n", log_file_name);
        sniffer_exit();
    }

    fclose(log_file);

    MSG_INFO("Now writing to log file %s\n", log_file_name);
    return;
}

/**
 * Max_ppm is the maximum packets per minute I will allow. Its probs only gonna be 100% lol
 * Scaler is how much I wanna increase after each period
 * Total time I want each offered load test to run
*/
void experiment_offered_load(uint16_t max_ppm, uint8_t scaler, uint16_t test_duration_secs) {

    struct lgw_pkt_tx_s pkt;
    unsigned long ms_per_minute = 60000;
    time_t run_time = 0;
    time_t start_time = 0;
    unsigned long wait_time_ms = 0;
    uint16_t packets_per_minute = 1;
    uint16_t fcnt = 1;
    uint8_t tx_status;
    int i;

    /* Transmission parameters */
    pkt.freq_hz = 916800000;
    pkt.tx_mode = 0; // Send transmission immediately
    // pkt.count_us = 0; // No delay or timestamp - might not need this
    pkt.rf_chain = 0; // Radio 0 - its the only one with transmissions enabled
    pkt.rf_power = 12;
    pkt.modulation = MOD_LORA;
    pkt.bandwidth = BW_125KHZ;
    pkt.datarate = DR_LORA_SF7; // SLOW
    pkt.coderate = CR_LORA_4_5;

    // Packet characteristics
    pkt.preamble = 8; // Standard LoRa preamble for SF7-12
    pkt.no_crc = false;
    pkt.no_header = false;
    pkt.size = 255;

    // Do the packet stuff - setting MHDR
    pkt.payload[0] = 0xE0; // MHDR - Set as PRP
    pkt.payload[1] = 0x12; // FHDR - DevAddr[0] 
    pkt.payload[2] = 0x34; // FHDR - DevAddr[1] 
    pkt.payload[3] = 0x56; // FHDR - DevAddr[2]
    pkt.payload[4] = 0x78; // FHDR - DevAddr[3]
    pkt.payload[5] = 0xA0; // FCtrl - set as an ACK and ADR
    
    pkt.payload[8] = 0x69; // Funny number FPort

    for (i = 9; i < 255; i++)
        pkt.payload[i] = i;

    
    for (; packets_per_minute <= max_ppm; packets_per_minute = packets_per_minute * scaler) {

        /* (Re)set FCnt cause this may be a new test! */
        fcnt = 1; // Set it to 1! Cant have 0
        pkt.payload[6] = fcnt & 0x00FF;
        pkt.payload[7] = fcnt >> 8;

        MSG_INFO("Starting Packets Per Minute (PPM) at %d test\n", packets_per_minute);
        wait_time_ms = ms_per_minute / packets_per_minute;

        /* Reset our run time... */
        run_time = 0;

        while ((run_time < test_duration_secs) && (!exit_sig && !quit_sig)) {
            /* Get time first */
            start_time = time(NULL);
            /* Lets send that packet */
            i = lgw_status(pkt.rf_chain, 1, &tx_status);
            if (i == LGW_HAL_ERROR) {
                MSG_ERR("lgw_status failed with code %d\n", tx_status);
            } else {
                printf("Good to use?\n");
                if (tx_status == TX_EMITTING) {
                    wait_ms(wait_time_ms); // Lets ruin everythiing
                } else if (tx_status == TX_FREE) {
                    i = lgw_send(&pkt);
                    printf("So does it send of fucking not???\n");
                    if (i != LGW_HAL_SUCCESS) {
                        MSG_ERR("failed to send for some reason\n");
                    } else {
                        printf("And it does sned so wtf?\n");
                        /* Just some safety wrapping lol */
                        fcnt++;

                        /* Update the FCnt */
                        pkt.payload[6] = fcnt & 0x00FF;
                        pkt.payload[7] = fcnt >> 8;
                    }
                }
            }
            run_time += time(NULL) - start_time;
        }

        if (!exit_sig || !quit_sig) {
            break;
        }

        MSG_INFO("Ending Packets Per Minute (PPM) at %d test\n", packets_per_minute);
        wait_ms(ms_per_minute); // Waiting 1 minute to seperate our times
    }
}

/**
 * Function for selective jamming operations. Select the part of a LoRaWAN packet to jam.
 * @param frame_selection   Frame part to jam (0 - Preamble & Sync, 1 - PHDR & CRC, 2 - FRMPayload, 3- final CRC)
 * @param pkt               Pointer to the packet object to send
 * @param socket            Socket file descriptor for the client connection
 * @param tx_power          Transmission power to be used by the second (jamming radio)
*/
void jamming_selective(struct lgw_pkt_tx_s *ref_pkt, uint8_t frame_section, int socket, int8_t attempts) {

    struct lgw_pkt_tx_s pkt;
    int ret;
    float ms_time_to_wait = 0;
    float bitrate = 0;
    char buffer_fcnt[5] = {'F', 'C', 'T', 0, 0};
    uint8_t fcnt = 0;

    /* copy memory details into the new struct */
    memcpy((void*)&pkt, (void*)ref_pkt, sizeof(struct lgw_pkt_tx_s));

    // Do the packet stuff - setting MHDR
    pkt.payload[0] = 0xE0; // MHDR - Set as PRP
    pkt.payload[1] = 0x12; // FHDR - DevAddr[0] 
    pkt.payload[2] = 0x34; // FHDR - DevAddr[1] 
    pkt.payload[3] = 0x56; // FHDR - DevAddr[2]
    pkt.payload[4] = 0x78; // FHDR - DevAddr[3]
    pkt.payload[5] = 0xA0; // FCtrl - set as an ACK and ADR
    pkt.payload[6] = 0x00; // FCnt[0]
    pkt.payload[7] = 0x00; // FCnt[1]
    pkt.payload[8] = 0x69; // Funny number FPort

    /* Get our datarate (bits/sec)*/
    switch(pkt.datarate) {
        case DR_LORA_SF7:   bitrate = BITRATE_DR5; break;
        case DR_LORA_SF8:   bitrate = BITRATE_DR4; break;
        case DR_LORA_SF9:   bitrate = BITRATE_DR3; break;
        case DR_LORA_SF10:  bitrate = BITRATE_DR2; break;
        case DR_LORA_SF11:  bitrate = BITRATE_DR1; break;
        case DR_LORA_SF12:  bitrate = BITRATE_DR0; break;
        default:            MSG_ERR("Unknown spreading factor found");
    }

    /* Make our time to wait */
    switch(frame_section) {
        case 0 :
            //MSG_INFO("Transmission will interrupt the Preamble and Sync Word.\n");
            ms_time_to_wait = 0;
            break;
        case 1 :
            //MSG_INFO("Transmission will interrupt the PHDR and CRC.\n");
            ms_time_to_wait = (BITS_PREAMBLE_N_SYNC / bitrate) * 1e3;
            ms_time_to_wait = (BITS_PHDR_N_CRC / bitrate) * 1e3;
            break;
        case 2 :
            //MSG_INFO("Transmission will interrupt the FRMPayload.\n");
            ms_time_to_wait = (BITS_PREAMBLE_N_SYNC / bitrate) * 1e3;
            ms_time_to_wait += (BITS_PHDR_N_CRC / bitrate) * 1e3;
            break;
        case 3 :
            //MSG_INFO("Transmission will interrupt the final CRC.\n");
            ms_time_to_wait = (BITS_PREAMBLE_N_SYNC / bitrate) * 1e3;
            ms_time_to_wait += (BITS_PHDR_N_CRC / bitrate) * 1e3;
            ms_time_to_wait = (255 * 8 / bitrate) * 1e3; // 255 is the size of the second radio
            break;
        default: 
            MSG_ERR("Bad frame section selected. Exiting function");
            return;
    }

    for (int i = 0; i < attempts; i++) {
        /* Set initial FCnt to 1*/
        fcnt++;
        pkt.payload[6] = fcnt & 0x00FF; // FCnt[0]
        buffer_fcnt[3] = pkt.payload[6];
        pkt.payload[7] = fcnt >> 8;     // FCnt[1]
        buffer_fcnt[4] = pkt.payload[7];

        send(socket, buffer_fcnt, sizeof(buffer_fcnt), 0);  

        /* Send the interrupt message */
        if (ms_time_to_wait != 0)
            wait_ms((unsigned long)ms_time_to_wait);
        ret = lgw_send(&pkt);
        if (ret != LGW_HAL_SUCCESS) {
            MSG_ERR("Failed to transmit packet.\n")
        } else {
            // Do nothing
        }  

        /* Wait before sending another message */
        wait_ms(500 - ms_time_to_wait);
    }

    MSG_INFO("Selective jam complete. %d packets sent\n", fcnt);
}

/**
 * Jammer_pkt_size must be 8 or greater to account for the MACPayload header data.
*/
void jamming_scaling (struct lgw_pkt_tx_s *ref_pkt, int socket, int jammer_pkt_size, long test_duration_secs, long jammer_spacing_ms, long desired_spacing_ms, uint16_t* fcnt_jam, uint16_t* fcnt_des) {

    struct lgw_pkt_tx_s pkt;
    struct timespec start_time, end_time;
    long run_time, time_diff, radio_jammer_ms, radio_desired_ms;
    float airtime;
    uint64_t transmitted_jammer = 0;
    uint64_t transmitted_desired = 0;
    int i;
    uint16_t fcnt = *fcnt_jam;
    uint16_t fcnt_client = *fcnt_des;
    uint8_t tx_status;
    char buffer_fcnt[] = {'F', 'C', 'T', 0, 0};

    /* copy memory details into the new struct */
    memcpy((void*)&pkt, (void*)ref_pkt, sizeof(struct lgw_pkt_tx_s));

    // Do the packet stuff - setting MHDR
    pkt.payload[0] = 0xE0; // MHDR - Set as PRP
    pkt.payload[1] = 0x12; // FHDR - DevAddr[0] 
    pkt.payload[2] = 0x34; // FHDR - DevAddr[1] 
    pkt.payload[3] = 0x56; // FHDR - DevAddr[2]
    pkt.payload[4] = 0x78; // FHDR - DevAddr[3]
    pkt.payload[5] = 0xA0; // FCtrl - set as an ACK and ADR
    pkt.payload[6] = 0x00; // FCnt[0]
    pkt.payload[7] = 0x00; // FCnt[1]
    pkt.payload[8] = 0x69; // Funny number FPort

    for (int i = 9; i < jammer_pkt_size; i++)
        pkt.payload[i] = i;

    /* Get our datarate (bits/sec)*/
    airtime = (pkt.size + 8 + 4.25 + 8 + 2) * 8;
    switch (pkt.datarate) {
        case DR_LORA_SF7:   airtime = airtime / BITRATE_DR5; break;
        case DR_LORA_SF8:   airtime = airtime / BITRATE_DR4; break;
        case DR_LORA_SF9:   airtime = airtime / BITRATE_DR3; break;
        case DR_LORA_SF10:  airtime = airtime / BITRATE_DR2; break;
        case DR_LORA_SF11:  airtime = airtime / BITRATE_DR1; break;
        case DR_LORA_SF12:  airtime = airtime / BITRATE_DR0; break;
        default:            MSG_ERR("Unknown spreading factor found");
    }

    run_time = 0;
    radio_desired_ms = desired_spacing_ms;
    radio_jammer_ms = radio_desired_ms / 2;

    while((run_time < test_duration_secs) && (!exit_sig && !quit_sig)) {

        clock_gettime(CLOCK_MONOTONIC, &start_time);

        if (radio_desired_ms >= desired_spacing_ms) {
            /* Load data and send it */
            buffer_fcnt[3] = fcnt_client & 0x00FF;
            buffer_fcnt[4] = fcnt_client >> 8;
            send(socket, buffer_fcnt, sizeof(buffer_fcnt), 0);

            /* Update our counters */
            fcnt_client++;
            transmitted_desired++;
            radio_desired_ms = 0; /* Reset for next send */
        }

        if (radio_jammer_ms >= jammer_spacing_ms) {
            i = lgw_status(0, 1, &tx_status); // Check radio 0
            if (i == LGW_HAL_ERROR) {
                MSG_ERR("lgw_status failed with code %d\n", tx_status);
            } else {
                if (tx_status == TX_FREE) {
                    // Update frame counter and send!
                    pkt.payload[6] = fcnt & 0x00FF;
                    pkt.payload[7] = fcnt >> 8;

                    i = lgw_send(&pkt);
                    if (i != LGW_HAL_SUCCESS) {
                        MSG_ERR("failed to send for some reason\n");
                    } else {
                        /* Update our counters */
                        transmitted_jammer++;
                        fcnt++;
                        radio_jammer_ms = 0; // Reset for next send
                    }
                }
            }
        }

        wait_ms(10);

        clock_gettime(CLOCK_MONOTONIC, &end_time);
        time_diff = (end_time.tv_sec - start_time.tv_sec) * 1e3 + (end_time.tv_nsec - start_time.tv_nsec)/ 1e6;
        radio_jammer_ms += time_diff;
        radio_desired_ms += time_diff;
        run_time += end_time.tv_sec - start_time.tv_sec;

    }

    if (exit_sig || quit_sig) // Code to exit
        return; // Return to exit this function immediately, shutdown will be handled in main

    /* Log message for transmission count - for debugging help */
    MSG_INFO("Scaling jam complete (Packets Sent: Jammer [%llu], Desired [%llu])\n", transmitted_jammer, transmitted_desired);

    /* Set fcnt values that are passed back */
    *fcnt_jam = fcnt;
    *fcnt_des = fcnt_client;
}

/**
 * Utility cleanup function should a SIGSTOP or SIGINT be recieved.
 * Attempts to close any sockets, stop the concentrator card, log the exit,
 * and return EXIT_SUCCESS
 * @param socket    Socket to close
 * @param server_fd Integer of the file descriptor to shutdown
*/
void interrupt_cleanup (int socket, int server_fd) {

    char* exit_str = "exit";

    send(socket, exit_str, strlen(exit_str), 0);

    /* close the client socket */ 
    close(socket);

    /* closing the server socket */ 
    shutdown(server_fd, SHUT_RDWR);

    sniffer_stop();
    MSG_INFO("Successfully exited our packet stinker program\n");

    exit(EXIT_SUCCESS);
}

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main(int argc, char **argv) {

    /* Socket related variables */
    int server_fd, new_socket;
    struct sockaddr_in address;
    int addrlen = sizeof(address);
    int opt = 1;
    char buffer_tx[5] =     {'T', 'X', 0, 0, 0};

    /* return management variable */
    int i;

    /* configuration file related */
    const char defaut_conf_fname[] = JSON_CONF_DEFAULT;
    const char * conf_fname = defaut_conf_fname; /* pointer to a string we won't touch */

    /* deamonise handling variables */
    pid_t pid;
    bool daemonise = false;
    uint16_t fcnt = 1;
    uint16_t fcnt_client = 1;

    char file_helper[50];

    /* variables for managing the functions below lol */
    uint16_t packets_per_minute = 1;
    uint8_t scaler = 2;
    long test_duration_secs = 60; //Currently 3mins per test
    unsigned long ms_per_minute = 60000;
    long int wait_time_ms = 0;
    float airtime = 0;
    long int packet_airtime_ms;


    /* transmitting packet */
    struct lgw_pkt_tx_s pkt;

    /* Transmission parameters */
    pkt.freq_hz = 916800000;
    pkt.tx_mode = 0; // Send transmission immediately
    // pkt.count_us = 0; // No delay or timestamp - might not need this
    pkt.rf_chain = 0; // Radio 0 - its the only one with transmissions enabled
    pkt.rf_power = 12;
    pkt.modulation = MOD_LORA;
    pkt.bandwidth = BW_125KHZ;
    pkt.datarate = DR_LORA_SF7; // SLOW
    pkt.coderate = CR_LORA_4_5;

    // Packet characteristics
    pkt.preamble = 8; // Standard LoRa preamble for SF7-12
    pkt.no_crc = false;
    pkt.no_header = false;
    pkt.size = 17;

    // Do the packet stuff - setting MHDR
    pkt.payload[0] = 0xE0; // MHDR - Set as PRP
    pkt.payload[1] = 0x12; // FHDR - DevAddr[0] 
    pkt.payload[2] = 0x34; // FHDR - DevAddr[1] 
    pkt.payload[3] = 0x56; // FHDR - DevAddr[2]
    pkt.payload[4] = 0x78; // FHDR - DevAddr[3]
    pkt.payload[5] = 0xA0; // FCtrl - set as an ACK and ADR
    pkt.payload[6] = 0x01; // FCnt[0]
    pkt.payload[7] = 0x00; // FCnt[1]
    pkt.payload[8] = 0x69; // Funny number FPort

    for (i = 9; i < 17; i++)
        pkt.payload[i] = i;

    airtime = (pkt.size + 8 + 4.25 + 8 + 2) * 8;
    switch (pkt.datarate) {
        case DR_LORA_SF7:   airtime = airtime / BITRATE_DR5; break;
        case DR_LORA_SF8:   airtime = airtime / BITRATE_DR4; break;
        case DR_LORA_SF9:   airtime = airtime / BITRATE_DR3; break;
        case DR_LORA_SF10:  airtime = airtime / BITRATE_DR2; break;
        case DR_LORA_SF11:  airtime = airtime / BITRATE_DR1; break;
        case DR_LORA_SF12:  airtime = airtime / BITRATE_DR0; break;
        default:            MSG_ERR("Unknown spreading factor found");
    }
    packet_airtime_ms = (long int)(airtime * 1e3);

    /* parse command line options */
    while( (i = getopt( argc, argv, OPTION_ARGS )) != -1 )
    {
        switch( i )
        {
        case 'c':
            conf_fname = optarg;
            break;

        case 'd':
            printf("INFO: Creating as daemon...\n");
            daemonise = true;
            break;

        case 'h':
            usage( );
            return EXIT_SUCCESS;
            break;

        case 'v':
            verbose =  true;
            break;

        default:
            printf("ERROR: argument parsing options, use -h option for help\n" );
            usage( );
            return EXIT_FAILURE;
        }
    }

    if (daemonise) {
        pid = fork();
        if (pid < 0) {
            printf("ERROR: Failed to daemonise\n");
            exit(EXIT_FAILURE); /* unable to daemonise */
        }
        if (pid > 0) {
            exit(EXIT_SUCCESS); /* parent termination */
        }
        printf("INFO: daemon created successfully\n");
    }

    /* opening log file */
    log_open("stinker_server");

    /* check device can open shells with system function */
    i = system(NULL);

    if (i == 0) {
        MSG_ERR("[main] Unable to open shell\n");
        exit(EXIT_FAILURE);
    }

    /* configuration files management */
    if (access(conf_fname, R_OK) == 0) { /* if there is a global conf, parse it  */
        MSG_INFO("[main] found configuration file %s, parsing it\n", conf_fname);
        i = parse_SX130x_configuration(conf_fname);
        if (i != 0) {
            MSG_ERR("[main] No \"SX130x_conf\" field in the chosen (or default) JSON\n");
            exit(EXIT_FAILURE);
        }
    } else {
        MSG_ERR("[main] failed to find any configuration file named %s\n", conf_fname);
        exit(EXIT_FAILURE);
    }

    /* starting the concentrator */
    if (sniffer_start()) {
        MSG_ERR("[main] Failed to start sniffer\n");
        exit(EXIT_FAILURE);
    }

    /* configure signal handling */
    sigemptyset(&sigact.sa_mask);
    sigact.sa_flags = 0;
    sigact.sa_handler = sig_handler;
    sigaction(SIGQUIT, &sigact, NULL); /* Ctrl-\ */
    sigaction(SIGINT, &sigact, NULL); /* Ctrl-C */
    sigaction(SIGTERM, &sigact, NULL); /* default "kill" command */

    /* Get ready for socket handling */
    /* Creating socket file descriptor */
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }
  
    // Forcefully attaching socket to the port 8080
    if (setsockopt(server_fd, SOL_SOCKET,
                   SO_REUSEADDR | SO_REUSEPORT, &opt,
                   sizeof(opt))) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);
  
    // Forcefully attaching socket to the port 8080
    if (bind(server_fd, (struct sockaddr*)&address,
             sizeof(address))
        < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    if (listen(server_fd, 3) < 0) {
        perror("listen");
        exit(EXIT_FAILURE);
    }
    if ((new_socket
         = accept(server_fd, (struct sockaddr*)&address,
                  (socklen_t*)&addrlen))
        < 0) {
        perror("accept");
        exit(EXIT_FAILURE);
    }

    sprintf(file_helper, "demo_showcase_scaled_jamming");
    log_open(file_helper);

    /* Set test duration */
    test_duration_secs = 20;
    /* Set frame counters */
    fcnt = 1; 
    fcnt_client = 1;

    for (int j = 0; j < 3; j++) {
        buffer_tx[2] = 12;
        send(new_socket, buffer_tx, sizeof(buffer_tx), 0);

        MSG_INFO("Loop %d\n", j);
        packets_per_minute = 64;
        wait_time_ms = (long)(ms_per_minute / packets_per_minute);
        
        while (wait_time_ms > packet_airtime_ms) {

            MSG_INFO("PPM: %d. Delay %lums\n", packets_per_minute, wait_time_ms);
            
            for (int j = 27; j > 26; j--) {
                pkt.rf_power = j;

                jamming_scaling(&pkt, new_socket, 17, test_duration_secs, wait_time_ms, 3000, &fcnt, &fcnt_client);

                wait_ms(test_duration_secs * 1000);

                if (exit_sig || quit_sig) // Code to exit
                    interrupt_cleanup(new_socket, server_fd);
            }

            packets_per_minute *= scaler;                               // Packets per minute as per scaler
            wait_time_ms = (long)(ms_per_minute / packets_per_minute);  // Update the wait time
        }
    }

    /* Send exit cmd to client sniffer and cleanup here */
    interrupt_cleanup(new_socket, server_fd);
}  

/* --- EOF ------------------------------------------------------------------ */
