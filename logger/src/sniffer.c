/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2013 Semtech-Cycleo

Description:
    Configure LoRa concentrator and record received packets in a log file

License: Revised BSD License, see LICENSE.TXT file include in the project
Maintainer: Sylvain Miermont
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

/* fix an issue between POSIX and C99 */
#if __STDC_VERSION__ >= 199901L
    #define _XOPEN_SOURCE 600
#else
    #define _XOPEN_SOURCE 500
#endif

#include <stdint.h>     /* C99 types */
#include <stdbool.h>    /* bool type */
#include <stdio.h>      /* printf fprintf sprintf fopen fputs */

#include <string.h>     /* memset */
#include <signal.h>     /* sigaction */
#include <time.h>       /* time clock_gettime strftime gmtime clock_nanosleep*/
#include <unistd.h>     /* getopt access */
#include <stdlib.h>     /* atoi */

#include "parson.h"
#include "base64.h"
#include "loragw_hal.h"
#include "loragw_gps.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a)   (sizeof(a) / sizeof((a)[0]))
#define MSG(args...)    fprintf(stderr,"loragw_pkt_logger: " args) /* message that is destined to the user */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#ifndef VERSION_STRING
    #define VERSION_STRING "undefined"
#endif

#define JSON_CONF_DEFAULT   "conf.json"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES (GLOBAL) ------------------------------------------- */

/* signal handling variables */
struct sigaction sigact; /* SIGQUIT&SIGINT&SIGTERM signal handling */
static int exit_sig = 0; /* 1 -> application terminates cleanly (shut down hardware, close open files, etc) */
static int quit_sig = 0; /* 1 -> application terminates without shutting down the hardware */

/* configuration variables needed by the application  */
static uint64_t lgwm = 0; /* LoRa gateway MAC address */
static char lgwm_str[17];

/* clock and log file management */
static time_t now_time;
static time_t log_start_time;
static FILE * log_file = NULL;
static char log_file_name[64];

/* GPS configuration and synchronization */
static char gps_tty_path[64] = "\0"; /* path of the TTY port GPS is connected on */
static int gps_tty_fd = -1; /* file descriptor of the GPS TTY port */
static bool gps_enabled = false; /* is GPS enabled on that gateway ? */

/* GPS time reference */
//static pthread_mutex_t mx_timeref = PTHREAD_MUTEX_INITIALIZER; /* control access to GPS time reference */
static bool gps_ref_valid; /* is GPS reference acceptable (ie. not too old) */
static struct tref time_reference_gps; /* time reference used for GPS <-> timestamp conversion */

/* Reference coordinates, for broadcasting (beacon) */
static struct coord_s reference_coord;

/* Enable faking the GPS coordinates of the gateway */
static bool gps_fake_enable; /* enable the feature */

/* Gateway specificities */
static int8_t antenna_gain = 0;

/* TX capabilities */
static struct lgw_tx_gain_lut_s txlut[LGW_RF_CHAIN_NB]; /* TX gain table */
static uint32_t tx_freq_min[LGW_RF_CHAIN_NB]; /* lowest frequency supported by TX chain */
static uint32_t tx_freq_max[LGW_RF_CHAIN_NB]; /* highest frequency supported by TX chain */
static bool tx_enable[LGW_RF_CHAIN_NB] = {false}; /* Is TX enabled for a given RF chain ? */

static uint32_t nb_pkt_log[LGW_IF_CHAIN_NB][8]; /* [CH][SF] */
static uint32_t nb_pkt_received_lora = 0;
static uint32_t nb_pkt_received_fsk = 0;

static struct lgw_conf_debug_s debugconf;
static uint32_t nb_pkt_received_ref[16];

/* Interface type */
static lgw_com_type_t com_type = LGW_COM_USB;

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DECLARATION ---------------------------------------- */

static void open_log(void);

static void sig_handler(int sigio);

static int parse_SX130x_configuration(const char * conf_file);

static int parse_gateway_configuration(const char * conf_file);

static int parse_debug_configuration(const char * conf_file);

static void usage (void);

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

static void usage( void )
{
    printf("~~~ Library version string~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
    printf(" %s\n", lgw_version_info());
    printf("~~~ Available options ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
    printf(" -h  print this help\n");
    printf(" -c <filename>  use config file other than 'conf.json'\n");
    printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
}

static void sig_handler(int sigio) {
    if (sigio == SIGQUIT) {
        quit_sig = true;
    } else if ((sigio == SIGINT) || (sigio == SIGTERM)) {
        exit_sig = true;
    }
    return;
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
    //JSON_Object *conf_scan_obj = NULL;
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
        MSG("ERROR: %s is not a valid JSON file\n", conf_file);
        exit(EXIT_FAILURE);
    }

    /* point to the gateway configuration object */
    conf_obj = json_object_get_object(json_value_get_object(root_val), conf_obj_name);
    if (conf_obj == NULL) {
        MSG("INFO: %s does not contain a JSON object named %s\n", conf_file, conf_obj_name);
        return -1;
    } else {
        MSG("INFO: %s does contain a JSON object named %s, parsing SX1302 parameters\n", conf_file, conf_obj_name);
    }

    /* set board configuration */
    memset(&boardconf, 0, sizeof boardconf); /* initialize configuration structure */
    str = json_object_get_string(conf_obj, "com_type");
    if (str == NULL) {
        MSG("ERROR: com_type must be configured in %s\n", conf_file);
        return -1;
    } else if (!strncmp(str, "SPI", 3) || !strncmp(str, "spi", 3)) {
        boardconf.com_type = LGW_COM_SPI;
    } else if (!strncmp(str, "USB", 3) || !strncmp(str, "usb", 3)) {
        boardconf.com_type = LGW_COM_USB;
    } else {
        MSG("ERROR: invalid com type: %s (should be SPI or USB)\n", str);
        return -1;
    }
    com_type = boardconf.com_type;
    str = json_object_get_string(conf_obj, "com_path");
    if (str != NULL) {
        strncpy(boardconf.com_path, str, sizeof boardconf.com_path);
        boardconf.com_path[sizeof boardconf.com_path - 1] = '\0'; /* ensure string termination */
    } else {
        MSG("ERROR: com_path must be configured in %s\n", conf_file);
        return -1;
    }
    val = json_object_get_value(conf_obj, "lorawan_public"); /* fetch value (if possible) */
    if (json_value_get_type(val) == JSONBoolean) {
        boardconf.lorawan_public = (bool)json_value_get_boolean(val);
    } else {
        MSG("WARNING: Data type for lorawan_public seems wrong, please check\n");
        boardconf.lorawan_public = false;
    }
    val = json_object_get_value(conf_obj, "clksrc"); /* fetch value (if possible) */
    if (json_value_get_type(val) == JSONNumber) {
        boardconf.clksrc = (uint8_t)json_value_get_number(val);
    } else {
        MSG("WARNING: Data type for clksrc seems wrong, please check\n");
        boardconf.clksrc = 0;
    }
    val = json_object_get_value(conf_obj, "full_duplex"); /* fetch value (if possible) */
    if (json_value_get_type(val) == JSONBoolean) {
        boardconf.full_duplex = (bool)json_value_get_boolean(val);
    } else {
        MSG("WARNING: Data type for full_duplex seems wrong, please check\n");
        boardconf.full_duplex = false;
    }
    MSG("INFO: com_type %s, com_path %s, lorawan_public %d, clksrc %d, full_duplex %d\n", (boardconf.com_type == LGW_COM_SPI) ? "SPI" : "USB", boardconf.com_path, boardconf.lorawan_public, boardconf.clksrc, boardconf.full_duplex);
    /* all parameters parsed, submitting configuration to the HAL */
    if (lgw_board_setconf(&boardconf) != LGW_HAL_SUCCESS) {
        MSG("ERROR: Failed to configure board\n");
        return -1;
    }

    /* set antenna gain configuration */
    val = json_object_get_value(conf_obj, "antenna_gain"); /* fetch value (if possible) */
    if (val != NULL) {
        if (json_value_get_type(val) == JSONNumber) {
            antenna_gain = (int8_t)json_value_get_number(val);
        } else {
            MSG("WARNING: Data type for antenna_gain seems wrong, please check\n");
            antenna_gain = 0;
        }
    }
    MSG("INFO: antenna_gain %d dBi\n", antenna_gain);

    /* set timestamp configuration */
    conf_ts_obj = json_object_get_object(conf_obj, "fine_timestamp");
    if (conf_ts_obj == NULL) {
        MSG("INFO: %s does not contain a JSON object for fine timestamp\n", conf_file);
    } else {
        val = json_object_get_value(conf_ts_obj, "enable"); /* fetch value (if possible) */
        if (json_value_get_type(val) == JSONBoolean) {
            tsconf.enable = (bool)json_value_get_boolean(val);
        } else {
            MSG("WARNING: Data type for fine_timestamp.enable seems wrong, please check\n");
            tsconf.enable = false;
        }
        if (tsconf.enable == true) {
            str = json_object_get_string(conf_ts_obj, "mode");
            if (str == NULL) {
                MSG("ERROR: fine_timestamp.mode must be configured in %s\n", conf_file);
                return -1;
            } else if (!strncmp(str, "high_capacity", 13) || !strncmp(str, "HIGH_CAPACITY", 13)) {
                tsconf.mode = LGW_FTIME_MODE_HIGH_CAPACITY;
            } else if (!strncmp(str, "all_sf", 6) || !strncmp(str, "ALL_SF", 6)) {
                tsconf.mode = LGW_FTIME_MODE_ALL_SF;
            } else {
                MSG("ERROR: invalid fine timestamp mode: %s (should be high_capacity or all_sf)\n", str);
                return -1;
            }
            MSG("INFO: Configuring precision timestamp with %s mode\n", str);

            /* all parameters parsed, submitting configuration to the HAL */
            if (lgw_ftime_setconf(&tsconf) != LGW_HAL_SUCCESS) {
                MSG("ERROR: Failed to configure fine timestamp\n");
                return -1;
            }
        } else {
            MSG("INFO: Configuring legacy timestamp\n");
        }
    }

    /* set SX1261 configuration */
    memset(&sx1261conf, 0, sizeof sx1261conf); /* initialize configuration structure */
    conf_sx1261_obj = json_object_get_object(conf_obj, "sx1261_conf"); /* fetch value (if possible) */
    if (conf_sx1261_obj == NULL) {
        MSG("INFO: no configuration for SX1261\n");
    } else {
        /* Global SX1261 configuration */
        str = json_object_get_string(conf_sx1261_obj, "spi_path");
        if (str != NULL) {
            strncpy(sx1261conf.spi_path, str, sizeof sx1261conf.spi_path);
            sx1261conf.spi_path[sizeof sx1261conf.spi_path - 1] = '\0'; /* ensure string termination */
        } else {
            MSG("INFO: SX1261 spi_path is not configured in %s\n", conf_file);
        }
        val = json_object_get_value(conf_sx1261_obj, "rssi_offset"); /* fetch value (if possible) */
        if (json_value_get_type(val) == JSONNumber) {
            sx1261conf.rssi_offset = (int8_t)json_value_get_number(val);
        } else {
            MSG("WARNING: Data type for sx1261_conf.rssi_offset seems wrong, please check\n");
            sx1261conf.rssi_offset = 0;
        }

        /* Spectral Scan configuration */
        // conf_scan_obj = json_object_get_object(conf_sx1261_obj, "spectral_scan"); /* fetch value (if possible) */
        // if (conf_scan_obj == NULL) {
        //     MSG("INFO: no configuration for Spectral Scan\n");
        // } else {
        //     val = json_object_get_value(conf_scan_obj, "enable"); /* fetch value (if possible) */
        //     if (json_value_get_type(val) == JSONBoolean) {
        //         /* Enable background spectral scan thread in packet forwarder */
        //         spectral_scan_params.enable = (bool)json_value_get_boolean(val);
        //     } else {
        //         MSG("WARNING: Data type for spectral_scan.enable seems wrong, please check\n");
        //     }
        //     if (spectral_scan_params.enable == true) {
        //         /* Enable the sx1261 radio hardware configuration to allow spectral scan */
        //         sx1261conf.enable = true;
        //         MSG("INFO: Spectral Scan with SX1261 is enabled\n");

        //         /* Get Spectral Scan Parameters */
        //         val = json_object_get_value(conf_scan_obj, "freq_start"); /* fetch value (if possible) */
        //         if (json_value_get_type(val) == JSONNumber) {
        //             spectral_scan_params.freq_hz_start = (uint32_t)json_value_get_number(val);
        //         } else {
        //             MSG("WARNING: Data type for spectral_scan.freq_start seems wrong, please check\n");
        //         }
        //         val = json_object_get_value(conf_scan_obj, "nb_chan"); /* fetch value (if possible) */
        //         if (json_value_get_type(val) == JSONNumber) {
        //             spectral_scan_params.nb_chan = (uint8_t)json_value_get_number(val);
        //         } else {
        //             MSG("WARNING: Data type for spectral_scan.nb_chan seems wrong, please check\n");
        //         }
        //         val = json_object_get_value(conf_scan_obj, "nb_scan"); /* fetch value (if possible) */
        //         if (json_value_get_type(val) == JSONNumber) {
        //             spectral_scan_params.nb_scan = (uint16_t)json_value_get_number(val);
        //         } else {
        //             MSG("WARNING: Data type for spectral_scan.nb_scan seems wrong, please check\n");
        //         }
        //         val = json_object_get_value(conf_scan_obj, "pace_s"); /* fetch value (if possible) */
        //         if (json_value_get_type(val) == JSONNumber) {
        //             spectral_scan_params.pace_s = (uint32_t)json_value_get_number(val);
        //         } else {
        //             MSG("WARNING: Data type for spectral_scan.pace_s seems wrong, please check\n");
        //         }
        //     }
        // }

        /* LBT configuration */
        conf_lbt_obj = json_object_get_object(conf_sx1261_obj, "lbt"); /* fetch value (if possible) */
        if (conf_lbt_obj == NULL) {
            MSG("INFO: no configuration for LBT\n");
        } else {
            val = json_object_get_value(conf_lbt_obj, "enable"); /* fetch value (if possible) */
            if (json_value_get_type(val) == JSONBoolean) {
                sx1261conf.lbt_conf.enable = (bool)json_value_get_boolean(val);
            } else {
                MSG("WARNING: Data type for lbt.enable seems wrong, please check\n");
            }
            if (sx1261conf.lbt_conf.enable == true) {
                /* Enable the sx1261 radio hardware configuration to allow spectral scan */
                sx1261conf.enable = true;
                MSG("INFO: Listen-Before-Talk with SX1261 is enabled\n");

                val = json_object_get_value(conf_lbt_obj, "rssi_target"); /* fetch value (if possible) */
                if (json_value_get_type(val) == JSONNumber) {
                    sx1261conf.lbt_conf.rssi_target = (int8_t)json_value_get_number(val);
                } else {
                    MSG("WARNING: Data type for lbt.rssi_target seems wrong, please check\n");
                    sx1261conf.lbt_conf.rssi_target = 0;
                }
                /* set LBT channels configuration */
                conf_lbtchan_array = json_object_get_array(conf_lbt_obj, "channels");
                if (conf_lbtchan_array != NULL) {
                    sx1261conf.lbt_conf.nb_channel = json_array_get_count(conf_lbtchan_array);
                    MSG("INFO: %u LBT channels configured\n", sx1261conf.lbt_conf.nb_channel);
                }
                for (i = 0; i < (int)sx1261conf.lbt_conf.nb_channel; i++) {
                    /* Sanity check */
                    if (i >= LGW_LBT_CHANNEL_NB_MAX) {
                        MSG("ERROR: LBT channel %d not supported, skip it\n", i);
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
                            MSG("WARNING: Data type for lbt.channels[%d].freq_hz seems wrong, please check\n", i);
                            sx1261conf.lbt_conf.channels[i].freq_hz = 0;
                        }
                    } else {
                        MSG("ERROR: no frequency defined for LBT channel %d\n", i);
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
                            MSG("WARNING: Data type for lbt.channels[%d].freq_hz seems wrong, please check\n", i);
                            sx1261conf.lbt_conf.channels[i].bandwidth = BW_UNDEFINED;
                        }
                    } else {
                        MSG("ERROR: no bandiwdth defined for LBT channel %d\n", i);
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
                                MSG("ERROR: scan time not supported for LBT channel %d, must be 128 or 5000\n", i);
                                return -1;
                            }
                        } else {
                            MSG("WARNING: Data type for lbt.channels[%d].scan_time_us seems wrong, please check\n", i);
                            sx1261conf.lbt_conf.channels[i].scan_time_us = 0;
                        }
                    } else {
                        MSG("ERROR: no scan_time_us defined for LBT channel %d\n", i);
                        return -1;
                    }

                    /* Channel transmit time */
                    val = json_object_dotget_value(conf_lbtchan_obj, "transmit_time_ms"); /* fetch value (if possible) */
                    if (val != NULL) {
                        if (json_value_get_type(val) == JSONNumber) {
                            sx1261conf.lbt_conf.channels[i].transmit_time_ms = (uint16_t)json_value_get_number(val);
                        } else {
                            MSG("WARNING: Data type for lbt.channels[%d].transmit_time_ms seems wrong, please check\n", i);
                            sx1261conf.lbt_conf.channels[i].transmit_time_ms = 0;
                        }
                    } else {
                        MSG("ERROR: no transmit_time_ms defined for LBT channel %d\n", i);
                        return -1;
                    }
                }
            }
        }

        /* all parameters parsed, submitting configuration to the HAL */
        if (lgw_sx1261_setconf(&sx1261conf) != LGW_HAL_SUCCESS) {
            MSG("ERROR: Failed to configure the SX1261 radio\n");
            return -1;
        }
    }

    /* set configuration for RF chains */
    for (i = 0; i < LGW_RF_CHAIN_NB; ++i) {
        memset(&rfconf, 0, sizeof rfconf); /* initialize configuration structure */
        snprintf(param_name, sizeof param_name, "radio_%i", i); /* compose parameter path inside JSON structure */
        val = json_object_get_value(conf_obj, param_name); /* fetch value (if possible) */
        if (json_value_get_type(val) != JSONObject) {
            MSG("INFO: no configuration for radio %i\n", i);
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
            MSG("INFO: radio %i disabled\n", i);
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
                MSG("WARNING: invalid radio type: %s (should be SX1255 or SX1257 or SX1250)\n", str);
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
                        MSG("WARNING: no frequency range specified for TX rf chain %d\n", i);
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
                                MSG("ERROR: Failed to configure concentrator TX Gain LUT for rf_chain %u\n", i);
                                return -1;
                            }
                        } else {
                            MSG("WARNING: No TX gain LUT defined for rf_chain %u\n", i);
                        }
                    } else {
                        MSG("WARNING: No TX gain LUT defined for rf_chain %u\n", i);
                    }
                }
            } else {
                rfconf.tx_enable = false;
            }
            MSG("INFO: radio %i enabled (type %s), center frequency %u, RSSI offset %f, tx enabled %d, single input mode %d\n", i, str, rfconf.freq_hz, rfconf.rssi_offset, rfconf.tx_enable, rfconf.single_input_mode);
        }
        /* all parameters parsed, submitting configuration to the HAL */
        if (lgw_rxrf_setconf(i, &rfconf) != LGW_HAL_SUCCESS) {
            MSG("ERROR: invalid configuration for radio %i\n", i);
            return -1;
        }
    }

    /* set configuration for demodulators */
    memset(&demodconf, 0, sizeof demodconf); /* initialize configuration structure */
    val = json_object_get_value(conf_obj, "chan_multiSF_All"); /* fetch value (if possible) */
    if (json_value_get_type(val) != JSONObject) {
        MSG("INFO: no configuration for LoRa multi-SF spreading factors enabling\n");
    } else {
        conf_demod_array = json_object_dotget_array(conf_obj, "chan_multiSF_All.spreading_factor_enable");
        if ((conf_demod_array != NULL) && ((size = json_array_get_count(conf_demod_array)) <= LGW_MULTI_NB)) {
            for (i = 0; i < (int)size; i++) {
                number = json_array_get_number(conf_demod_array, i);
                if (number < 5 || number > 12) {
                    MSG("WARNING: failed to parse chan_multiSF_All.spreading_factor_enable (wrong value at idx %d)\n", i);
                    demodconf.multisf_datarate = 0xFF; /* enable all SFs */
                    break;
                } else {
                    /* set corresponding bit in the bitmask SF5 is LSB -> SF12 is MSB */
                    demodconf.multisf_datarate |= (1 << (number - 5));
                }
            }
        } else {
            MSG("WARNING: failed to parse chan_multiSF_All.spreading_factor_enable\n");
            demodconf.multisf_datarate = 0xFF; /* enable all SFs */
        }
        /* all parameters parsed, submitting configuration to the HAL */
        if (lgw_demod_setconf(&demodconf) != LGW_HAL_SUCCESS) {
            MSG("ERROR: invalid configuration for demodulation parameters\n");
            return -1;
        }
    }

    /* set configuration for Lora multi-SF channels (bandwidth cannot be set) */
    for (i = 0; i < LGW_MULTI_NB; ++i) {
        memset(&ifconf, 0, sizeof ifconf); /* initialize configuration structure */
        snprintf(param_name, sizeof param_name, "chan_multiSF_%i", i); /* compose parameter path inside JSON structure */
        val = json_object_get_value(conf_obj, param_name); /* fetch value (if possible) */
        if (json_value_get_type(val) != JSONObject) {
            MSG("INFO: no configuration for Lora multi-SF channel %i\n", i);
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
            MSG("INFO: Lora multi-SF channel %i disabled\n", i);
        } else  { /* Lora multi-SF channel enabled, will parse the other parameters */
            snprintf(param_name, sizeof param_name, "chan_multiSF_%i.radio", i);
            ifconf.rf_chain = (uint32_t)json_object_dotget_number(conf_obj, param_name);
            snprintf(param_name, sizeof param_name, "chan_multiSF_%i.if", i);
            ifconf.freq_hz = (int32_t)json_object_dotget_number(conf_obj, param_name);
            // TODO: handle individual SF enabling and disabling (spread_factor)
            MSG("INFO: Lora multi-SF channel %i>  radio %i, IF %i Hz, 125 kHz bw, SF 5 to 12\n", i, ifconf.rf_chain, ifconf.freq_hz);
        }
        /* all parameters parsed, submitting configuration to the HAL */
        if (lgw_rxif_setconf(i, &ifconf) != LGW_HAL_SUCCESS) {
            MSG("ERROR: invalid configuration for Lora multi-SF channel %i\n", i);
            return -1;
        }
    }

    /* set configuration for Lora standard channel */
    memset(&ifconf, 0, sizeof ifconf); /* initialize configuration structure */
    val = json_object_get_value(conf_obj, "chan_Lora_std"); /* fetch value (if possible) */
    if (json_value_get_type(val) != JSONObject) {
        MSG("INFO: no configuration for Lora standard channel\n");
    } else {
        val = json_object_dotget_value(conf_obj, "chan_Lora_std.enable");
        if (json_value_get_type(val) == JSONBoolean) {
            ifconf.enable = (bool)json_value_get_boolean(val);
        } else {
            ifconf.enable = false;
        }
        if (ifconf.enable == false) {
            MSG("INFO: Lora standard channel %i disabled\n", i);
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
                    MSG("ERROR: payload length setting is mandatory for implicit header mode\n");
                    return -1;
                }
                val = json_object_dotget_value(conf_obj, "chan_Lora_std.implicit_crc_en");
                if (json_value_get_type(val) == JSONBoolean) {
                    ifconf.implicit_crc_en = (bool)json_value_get_boolean(val);
                } else {
                    MSG("ERROR: CRC enable setting is mandatory for implicit header mode\n");
                    return -1;
                }
                val = json_object_dotget_value(conf_obj, "chan_Lora_std.implicit_coderate");
                if (json_value_get_type(val) == JSONNumber) {
                    ifconf.implicit_coderate = (uint8_t)json_value_get_number(val);
                } else {
                    MSG("ERROR: coding rate setting is mandatory for implicit header mode\n");
                    return -1;
                }
            }

            MSG("INFO: Lora std channel> radio %i, IF %i Hz, %u Hz bw, SF %u, %s\n", ifconf.rf_chain, ifconf.freq_hz, bw, sf, (ifconf.implicit_hdr == true) ? "Implicit header" : "Explicit header");
        }
        if (lgw_rxif_setconf(8, &ifconf) != LGW_HAL_SUCCESS) {
            MSG("ERROR: invalid configuration for Lora standard channel\n");
            return -1;
        }
    }

    /* set configuration for FSK channel */
    memset(&ifconf, 0, sizeof ifconf); /* initialize configuration structure */
    val = json_object_get_value(conf_obj, "chan_FSK"); /* fetch value (if possible) */
    if (json_value_get_type(val) != JSONObject) {
        MSG("INFO: no configuration for FSK channel\n");
    } else {
        val = json_object_dotget_value(conf_obj, "chan_FSK.enable");
        if (json_value_get_type(val) == JSONBoolean) {
            ifconf.enable = (bool)json_value_get_boolean(val);
        } else {
            ifconf.enable = false;
        }
        if (ifconf.enable == false) {
            MSG("INFO: FSK channel %i disabled\n", i);
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

            MSG("INFO: FSK channel> radio %i, IF %i Hz, %u Hz bw, %u bps datarate\n", ifconf.rf_chain, ifconf.freq_hz, bw, ifconf.datarate);
        }
        if (lgw_rxif_setconf(9, &ifconf) != LGW_HAL_SUCCESS) {
            MSG("ERROR: invalid configuration for FSK channel\n");
            return -1;
        }
    }
    json_value_free(root_val);

    return 0;
}

static int parse_gateway_configuration(const char * conf_file) {
    const char conf_obj_name[] = "gateway_conf";
    JSON_Value *root_val;
    JSON_Object *conf_obj = NULL;
    JSON_Value *val = NULL; /* needed to detect the absence of some fields */
    const char *str; /* pointer to sub-strings in the JSON data */
    unsigned long long ull = 0;

    /* try to parse JSON */
    root_val = json_parse_file_with_comments(conf_file);
    if (root_val == NULL) {
        MSG("ERROR: %s is not a valid JSON file\n", conf_file);
        exit(EXIT_FAILURE);
    }

    /* point to the gateway configuration object */
    conf_obj = json_object_get_object(json_value_get_object(root_val), conf_obj_name);
    if (conf_obj == NULL) {
        MSG("INFO: %s does not contain a JSON object named %s\n", conf_file, conf_obj_name);
        return -1;
    } else {
        MSG("INFO: %s does contain a JSON object named %s, parsing gateway parameters\n", conf_file, conf_obj_name);
    }

    /* gateway unique identifier (aka MAC address) (optional) */
    str = json_object_get_string(conf_obj, "gateway_ID");
    if (str != NULL) {
        sscanf(str, "%llx", &ull);
        lgwm = ull;
        MSG("INFO: gateway MAC address is configured to %016llX\n", ull);
    }

    /* server hostname or IP address (optional) */
    // str = json_object_get_string(conf_obj, "server_address");
    // if (str != NULL) {
    //     strncpy(serv_addr, str, sizeof serv_addr);
    //     serv_addr[sizeof serv_addr - 1] = '\0'; /* ensure string termination */
    //     MSG("INFO: server hostname or IP address is configured to \"%s\"\n", serv_addr);
    // }

    /* get up and down ports (optional) */
    // val = json_object_get_value(conf_obj, "serv_port_up");
    // if (val != NULL) {
    //     snprintf(serv_port_up, sizeof serv_port_up, "%u", (uint16_t)json_value_get_number(val));
    //     MSG("INFO: upstream port is configured to \"%s\"\n", serv_port_up);
    // }
    // val = json_object_get_value(conf_obj, "serv_port_down");
    // if (val != NULL) {
    //     snprintf(serv_port_down, sizeof serv_port_down, "%u", (uint16_t)json_value_get_number(val));
    //     MSG("INFO: downstream port is configured to \"%s\"\n", serv_port_down);
    // }

    /* get keep-alive interval (in seconds) for downstream (optional) */
    // val = json_object_get_value(conf_obj, "keepalive_interval");
    // if (val != NULL) {
    //     keepalive_time = (int)json_value_get_number(val);
    //     MSG("INFO: downstream keep-alive interval is configured to %u seconds\n", keepalive_time);
    // }

    /* get interval (in seconds) for statistics display (optional) */
    // val = json_object_get_value(conf_obj, "stat_interval");
    // if (val != NULL) {
    //     stat_interval = (unsigned)json_value_get_number(val);
    //     MSG("INFO: statistics display interval is configured to %u seconds\n", stat_interval);
    // }

    /* get time-out value (in ms) for upstream datagrams (optional) */
    // val = json_object_get_value(conf_obj, "push_timeout_ms");
    // if (val != NULL) {
    //     push_timeout_half.tv_usec = 500 * (long int)json_value_get_number(val);
    //     MSG("INFO: upstream PUSH_DATA time-out is configured to %u ms\n", (unsigned)(push_timeout_half.tv_usec / 500));
    // }

    /* packet filtering parameters */
    // val = json_object_get_value(conf_obj, "forward_crc_valid");
    // if (json_value_get_type(val) == JSONBoolean) {
    //     fwd_valid_pkt = (bool)json_value_get_boolean(val);
    // }
    // MSG("INFO: packets received with a valid CRC will%s be forwarded\n", (fwd_valid_pkt ? "" : " NOT"));
    // val = json_object_get_value(conf_obj, "forward_crc_error");
    // if (json_value_get_type(val) == JSONBoolean) {
    //     fwd_error_pkt = (bool)json_value_get_boolean(val);
    // }
    // MSG("INFO: packets received with a CRC error will%s be forwarded\n", (fwd_error_pkt ? "" : " NOT"));
    // val = json_object_get_value(conf_obj, "forward_crc_disabled");
    // if (json_value_get_type(val) == JSONBoolean) {
    //     fwd_nocrc_pkt = (bool)json_value_get_boolean(val);
    // }
    // MSG("INFO: packets received with no CRC will%s be forwarded\n", (fwd_nocrc_pkt ? "" : " NOT"));

    /* GPS module TTY path (optional) */
    str = json_object_get_string(conf_obj, "gps_tty_path");
    if (str != NULL) {
        strncpy(gps_tty_path, str, sizeof gps_tty_path);
        gps_tty_path[sizeof gps_tty_path - 1] = '\0'; /* ensure string termination */
        MSG("INFO: GPS serial port path is configured to \"%s\"\n", gps_tty_path);
    }

    /* get reference coordinates */
    val = json_object_get_value(conf_obj, "ref_latitude");
    if (val != NULL) {
        reference_coord.lat = (double)json_value_get_number(val);
        MSG("INFO: Reference latitude is configured to %f deg\n", reference_coord.lat);
    }
    val = json_object_get_value(conf_obj, "ref_longitude");
    if (val != NULL) {
        reference_coord.lon = (double)json_value_get_number(val);
        MSG("INFO: Reference longitude is configured to %f deg\n", reference_coord.lon);
    }
    val = json_object_get_value(conf_obj, "ref_altitude");
    if (val != NULL) {
        reference_coord.alt = (short)json_value_get_number(val);
        MSG("INFO: Reference altitude is configured to %i meters\n", reference_coord.alt);
    }

    /* Gateway GPS coordinates hardcoding (aka. faking) option */
    val = json_object_get_value(conf_obj, "fake_gps");
    if (json_value_get_type(val) == JSONBoolean) {
        gps_fake_enable = (bool)json_value_get_boolean(val);
        if (gps_fake_enable == true) {
            MSG("INFO: fake GPS is enabled\n");
        } else {
            MSG("INFO: fake GPS is disabled\n");
        }
    }

    /* Beacon signal period (optional) */
    // val = json_object_get_value(conf_obj, "beacon_period");
    // if (val != NULL) {
    //     beacon_period = (uint32_t)json_value_get_number(val);
    //     if ((beacon_period > 0) && (beacon_period < 6)) {
    //         MSG("ERROR: invalid configuration for Beacon period, must be >= 6s\n");
    //         return -1;
    //     } else {
    //         MSG("INFO: Beaconing period is configured to %u seconds\n", beacon_period);
    //     }
    // }

    /* Beacon TX frequency (optional) */
    // val = json_object_get_value(conf_obj, "beacon_freq_hz");
    // if (val != NULL) {
    //     beacon_freq_hz = (uint32_t)json_value_get_number(val);
    //     MSG("INFO: Beaconing signal will be emitted at %u Hz\n", beacon_freq_hz);
    // }

    /* Number of beacon channels (optional) */
    // val = json_object_get_value(conf_obj, "beacon_freq_nb");
    // if (val != NULL) {
    //     beacon_freq_nb = (uint8_t)json_value_get_number(val);
    //     MSG("INFO: Beaconing channel number is set to %u\n", beacon_freq_nb);
    // }

    /* Frequency step between beacon channels (optional) */
    // val = json_object_get_value(conf_obj, "beacon_freq_step");
    // if (val != NULL) {
    //     beacon_freq_step = (uint32_t)json_value_get_number(val);
    //     MSG("INFO: Beaconing channel frequency step is set to %uHz\n", beacon_freq_step);
    // }

    /* Beacon datarate (optional) */
    // val = json_object_get_value(conf_obj, "beacon_datarate");
    // if (val != NULL) {
    //     beacon_datarate = (uint8_t)json_value_get_number(val);
    //     MSG("INFO: Beaconing datarate is set to SF%d\n", beacon_datarate);
    // }

    /* Beacon modulation bandwidth (optional) */
    // val = json_object_get_value(conf_obj, "beacon_bw_hz");
    // if (val != NULL) {
    //     beacon_bw_hz = (uint32_t)json_value_get_number(val);
    //     MSG("INFO: Beaconing modulation bandwidth is set to %dHz\n", beacon_bw_hz);
    // }

    /* Beacon TX power (optional) */
    // val = json_object_get_value(conf_obj, "beacon_power");
    // if (val != NULL) {
    //     beacon_power = (int8_t)json_value_get_number(val);
    //     MSG("INFO: Beaconing TX power is set to %ddBm\n", beacon_power);
    // }

    /* Beacon information descriptor (optional) */
    // val = json_object_get_value(conf_obj, "beacon_infodesc");
    // if (val != NULL) {
    //     beacon_infodesc = (uint8_t)json_value_get_number(val);
    //     MSG("INFO: Beaconing information descriptor is set to %u\n", beacon_infodesc);
    // }

    /* Auto-quit threshold (optional) */
    // val = json_object_get_value(conf_obj, "autoquit_threshold");
    // if (val != NULL) {
    //     autoquit_threshold = (uint32_t)json_value_get_number(val);
    //     MSG("INFO: Auto-quit after %u non-acknowledged PULL_DATA\n", autoquit_threshold);
    // }

    /* free JSON parsing data structure */
    json_value_free(root_val);
    return 0;
}

static int parse_debug_configuration(const char * conf_file) {
    int i;
    const char conf_obj_name[] = "debug_conf";
    JSON_Value *root_val;
    JSON_Object *conf_obj = NULL;
    JSON_Array *conf_array = NULL;
    JSON_Object *conf_obj_array = NULL;
    const char *str; /* pointer to sub-strings in the JSON data */

    /* Initialize structure */
    memset(&debugconf, 0, sizeof debugconf);

    /* try to parse JSON */
    root_val = json_parse_file_with_comments(conf_file);
    if (root_val == NULL) {
        MSG("ERROR: %s is not a valid JSON file\n", conf_file);
        exit(EXIT_FAILURE);
    }

    /* point to the gateway configuration object */
    conf_obj = json_object_get_object(json_value_get_object(root_val), conf_obj_name);
    if (conf_obj == NULL) {
        MSG("INFO: %s does not contain a JSON object named %s\n", conf_file, conf_obj_name);
        json_value_free(root_val);
        return -1;
    } else {
        MSG("INFO: %s does contain a JSON object named %s, parsing debug parameters\n", conf_file, conf_obj_name);
    }

    /* Get reference payload configuration */
    conf_array = json_object_get_array (conf_obj, "ref_payload");
    if (conf_array != NULL) {
        debugconf.nb_ref_payload = json_array_get_count(conf_array);
        MSG("INFO: got %u debug reference payload\n", debugconf.nb_ref_payload);

        for (i = 0; i < (int)debugconf.nb_ref_payload; i++) {
            conf_obj_array = json_array_get_object(conf_array, i);
            /* id */
            str = json_object_get_string(conf_obj_array, "id");
            if (str != NULL) {
                sscanf(str, "0x%08X", &(debugconf.ref_payload[i].id));
                MSG("INFO: reference payload ID %d is 0x%08X\n", i, debugconf.ref_payload[i].id);
            }

            /* global count */
            nb_pkt_received_ref[i] = 0;
        }
    }

    /* Get log file configuration */
    str = json_object_get_string(conf_obj, "log_file");
    if (str != NULL) {
        strncpy(debugconf.log_file_name, str, sizeof debugconf.log_file_name);
        debugconf.log_file_name[sizeof debugconf.log_file_name - 1] = '\0'; /* ensure string termination */
        MSG("INFO: setting debug log file name to %s\n", debugconf.log_file_name);
    }

    /* Commit configuration */
    if (lgw_debug_setconf(&debugconf) != LGW_HAL_SUCCESS) {
        MSG("ERROR: Failed to configure debug\n");
        json_value_free(root_val);
        return -1;
    }

    /* free JSON parsing data structure */
    json_value_free(root_val);
    return 0;
}

void open_log(void) {
    int i;
    char iso_date[20];

    strftime(iso_date,ARRAY_SIZE(iso_date),"%Y%m%dT%H%M%SZ",gmtime(&now_time)); /* format yyyymmddThhmmssZ */
    log_start_time = now_time; /* keep track of when the log was started, for log rotation */

    sprintf(log_file_name, "pktlog_%s_%s.csv", lgwm_str, iso_date);
    log_file = fopen(log_file_name, "a"); /* create log file, append if file already exist */
    if (log_file == NULL) {
        MSG("ERROR: impossible to create log file %s\n", log_file_name);
        exit(EXIT_FAILURE);
    }

    i = fprintf(log_file, "\"gateway ID\",\"node MAC\",\"UTC timestamp\",\"us count\",\"frequency\",\"RF chain\",\"RX chain\",\"status\",\"size\",\"modulation\",\"bandwidth\",\"datarate\",\"coderate\",\"RSSI\",\"SNR\",\"payload\"\n");
    if (i < 0) {
        MSG("ERROR: impossible to write to log file %s\n", log_file_name);
        exit(EXIT_FAILURE);
    }

    MSG("INFO: Now writing to log file %s\n", log_file_name);
    return;
}

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main(int argc, char **argv)
{
    int i, j; /* loop and temporary variables */
    int x;

    //struct timespec sleep_time = {0, 3000000}; /* 3 ms */

    /* clock and log rotation management */
    // int log_rotate_interval = 3600; /* by default, rotation every hour */
    // int time_check = 0; /* variable used to limit the number of calls to time() function */
    // unsigned long pkt_in_log = 0; /* count the number of packet written in each log file */

    /* configuration file related */
    const char defaut_conf_fname[] = JSON_CONF_DEFAULT;
    const char * conf_fname = defaut_conf_fname; /* pointer to a string we won't touch */

    /* allocate memory for packet fetching and processing */
    // struct lgw_pkt_rx_s rxpkt[16]; /* array containing up to 16 inbound packets metadata */
    // struct lgw_pkt_rx_s *p; /* pointer on a RX packet */
    // int nb_pkt;

    /* local timestamp variables until we get accurate GPS time */
    // struct timespec fetch_time;
    // char fetch_timestamp[30];
    // struct tm * x;

    /* parse command line options */
    while( (i = getopt( argc, argv, "hc:" )) != -1 )
    {
        switch( i )
        {
        case 'h':
            usage( );
            return EXIT_SUCCESS;
            break;

        case 'c':
            conf_fname = optarg;
            break;

        default:
            printf( "ERROR: argument parsing options, use -h option for help\n" );
            usage( );
            return EXIT_FAILURE;
        }
    }

    /* configure signal handling */
    sigemptyset(&sigact.sa_mask);
    sigact.sa_flags = 0;
    sigact.sa_handler = sig_handler;
    sigaction(SIGQUIT, &sigact, NULL);
    sigaction(SIGINT, &sigact, NULL);
    sigaction(SIGTERM, &sigact, NULL);

    /* configuration files management */
    if (access(conf_fname, R_OK) == 0) { /* if there is a global conf, parse it  */
        MSG("INFO: found configuration file %s, parsing it\n", conf_fname);
        x = parse_SX130x_configuration(conf_fname);
        if (x != 0) {
            exit(EXIT_FAILURE);
        }
        x = parse_gateway_configuration(conf_fname);
        if (x != 0) {
            exit(EXIT_FAILURE);
        }
        x = parse_debug_configuration(conf_fname);
        if (x != 0) {
            MSG("INFO: no debug configuration\n");
        }
    } else {
        MSG("ERROR: [main] failed to find any configuration file named %s\n", conf_fname);
        exit(EXIT_FAILURE);
    }

    /* starting the concentrator */
    i = lgw_start();
    if (i == LGW_HAL_SUCCESS) {
        MSG("INFO: concentrator started, packet can now be received\n");
    } else {
        MSG("ERROR: failed to start the concentrator\n");
        return EXIT_FAILURE;
    }

    /* transform the MAC address into a string */
    // sprintf(lgwm_str, "%08X%08X", (uint32_t)(lgwm >> 32), (uint32_t)(lgwm & 0xFFFFFFFF));

    // /* opening log file and writing CSV header*/
    // time(&now_time);
    // open_log();

    // /* main loop */
    // while ((quit_sig != 1) && (exit_sig != 1)) {
    //     /* fetch packets */
    //     nb_pkt = lgw_receive(ARRAY_SIZE(rxpkt), rxpkt);
    //     if (nb_pkt == LGW_HAL_ERROR) {
    //         MSG("ERROR: failed packet fetch, exiting\n");
    //         return EXIT_FAILURE;
    //     } else if (nb_pkt == 0) {
    //         clock_nanosleep(CLOCK_MONOTONIC, 0, &sleep_time, NULL); /* wait a short time if no packets */
    //     } else {
    //         /* local timestamp generation until we get accurate GPS time */
    //         clock_gettime(CLOCK_REALTIME, &fetch_time);
    //         x = gmtime(&(fetch_time.tv_sec));
    //         sprintf(fetch_timestamp,"%04i-%02i-%02i %02i:%02i:%02i.%03liZ",(x->tm_year)+1900,(x->tm_mon)+1,x->tm_mday,x->tm_hour,x->tm_min,x->tm_sec,(fetch_time.tv_nsec)/1000000); /* ISO 8601 format */
    //     }

    //     /* log packets */
    //     for (i=0; i < nb_pkt; ++i) {
    //         p = &rxpkt[i];

    //         /* writing gateway ID */
    //         fprintf(log_file, "\"%08X%08X\",", (uint32_t)(lgwm >> 32), (uint32_t)(lgwm & 0xFFFFFFFF));

    //         /* writing node MAC address */
    //         fputs("\"\",", log_file); // TODO: need to parse payload

    //         /* writing UTC timestamp*/
    //         fprintf(log_file, "\"%s\",", fetch_timestamp);
    //         // TODO: replace with GPS time when available

    //         /* writing internal clock */
    //         fprintf(log_file, "%10u,", p->count_us);

    //         /* writing RX frequency */
    //         fprintf(log_file, "%10u,", p->freq_hz);

    //         /* writing RF chain */
    //         fprintf(log_file, "%u,", p->rf_chain);

    //         /* writing RX modem/IF chain */
    //         fprintf(log_file, "%2d,", p->if_chain);

    //         /* writing status */
    //         switch(p->status) {
    //             case STAT_CRC_OK:       fputs("\"CRC_OK\" ,", log_file); break;
    //             case STAT_CRC_BAD:      fputs("\"CRC_BAD\",", log_file); break;
    //             case STAT_NO_CRC:       fputs("\"NO_CRC\" ,", log_file); break;
    //             case STAT_UNDEFINED:    fputs("\"UNDEF\"  ,", log_file); break;
    //             default:                fputs("\"ERR\"    ,", log_file);
    //         }

    //         /* writing payload size */
    //         fprintf(log_file, "%3u,", p->size);

    //         /* writing modulation */
    //         switch(p->modulation) {
    //             case MOD_LORA:  fputs("\"LORA\",", log_file); break;
    //             case MOD_FSK:   fputs("\"FSK\" ,", log_file); break;
    //             default:        fputs("\"ERR\" ,", log_file);
    //         }

    //         /* writing bandwidth */
    //         switch(p->bandwidth) {
    //             case BW_500KHZ:     fputs("500000,", log_file); break;
    //             case BW_250KHZ:     fputs("250000,", log_file); break;
    //             case BW_125KHZ:     fputs("125000,", log_file); break;
    //             case BW_UNDEFINED:  fputs("0     ,", log_file); break;
    //             default:            fputs("-1    ,", log_file);
    //         }

    //         /* writing datarate */
    //         if (p->modulation == MOD_LORA) {
    //             switch (p->datarate) {
    //                 case DR_LORA_SF7:   fputs("\"SF7\"   ,", log_file); break;
    //                 case DR_LORA_SF8:   fputs("\"SF8\"   ,", log_file); break;
    //                 case DR_LORA_SF9:   fputs("\"SF9\"   ,", log_file); break;
    //                 case DR_LORA_SF10:  fputs("\"SF10\"  ,", log_file); break;
    //                 case DR_LORA_SF11:  fputs("\"SF11\"  ,", log_file); break;
    //                 case DR_LORA_SF12:  fputs("\"SF12\"  ,", log_file); break;
    //                 default:            fputs("\"ERR\"   ,", log_file);
    //             }
    //         } else if (p->modulation == MOD_FSK) {
    //             fprintf(log_file, "\"%6u\",", p->datarate);
    //         } else {
    //             fputs("\"ERR\"   ,", log_file);
    //         }

    //         /* writing coderate */
    //         switch (p->coderate) {
    //             case CR_LORA_4_5:   fputs("\"4/5\",", log_file); break;
    //             case CR_LORA_4_6:   fputs("\"2/3\",", log_file); break;
    //             case CR_LORA_4_7:   fputs("\"4/7\",", log_file); break;
    //             case CR_LORA_4_8:   fputs("\"1/2\",", log_file); break;
    //             case CR_UNDEFINED:  fputs("\"\"   ,", log_file); break;
    //             default:            fputs("\"ERR\",", log_file);
    //         }

    //         /* writing packet RSSI */
    //         fprintf(log_file, "%+.0f,", p->rssi);

    //         /* writing packet average SNR */
    //         fprintf(log_file, "%+5.1f,", p->snr);

    //         /* writing hex-encoded payload (bundled in 32-bit words) */
    //         fputs("\"", log_file);
    //         for (j = 0; j < p->size; ++j) {
    //             if ((j > 0) && (j%4 == 0)) fputs("-", log_file);
    //             fprintf(log_file, "%02X", p->payload[j]);
    //         }

    //         /* end of log file line */
    //         fputs("\"\n", log_file);
    //         fflush(log_file);
    //         ++pkt_in_log;
    //     }

    //     /* check time and rotate log file if necessary */
    //     ++time_check;
    //     if (time_check >= 8) {
    //         time_check = 0;
    //         time(&now_time);
    //         if (difftime(now_time, log_start_time) > log_rotate_interval) {
    //             fclose(log_file);
    //             MSG("INFO: log file %s closed, %lu packet(s) recorded\n", log_file_name, pkt_in_log);
    //             pkt_in_log = 0;
    //             open_log();
    //         }
    //     }
    // }


    // if (exit_sig == 1) {
    //     /* clean up before leaving */
    //     i = lgw_stop();
    //     if (i == LGW_HAL_SUCCESS) {
    //         MSG("INFO: concentrator stopped successfully\n");
    //     } else {
    //         MSG("WARNING: failed to stop concentrator successfully\n");
    //     }
    //     fclose(log_file);
    //     MSG("INFO: log file %s closed, %lu packet(s) recorded\n", log_file_name, pkt_in_log);
    // }

    i = lgw_stop();
    if (i == LGW_HAL_SUCCESS) {
        MSG("INFO: concentrator stopped successfully\n");
    } else {
        MSG("WARNING: failed to stop concentrator successfully\n");
    }
    MSG("INFO: Exiting packet logger program\n");
    return EXIT_SUCCESS;
}

/* --- EOF ------------------------------------------------------------------ */
