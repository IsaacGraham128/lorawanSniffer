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
#include <stdlib.h>     /* atoi, malloc */
#include <errno.h>      /* error messages */

#include <pthread.h>

#include "parson.h"
#include "base64.h"
#include "loragw_hal.h"
#include "loragw_aux.h"
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

#define JSON_CONF_DEFAULT   "conf_au_sub2.json"//"conf.json"
#define JSON_CONF_AU_SUB1   "conf_au_sub1.json"
#define JSON_CONF_AU_SUB2   "conf_au_sub2.json"

#define GPS_REF_MAX_AGE     30          /* maximum admitted delay in seconds of GPS loss before considering latest GPS sync unusable */
#define XERR_INIT_AVG       16          /* nb of measurements the XTAL correction is averaged on as initial value */
#define XERR_FILT_COEF      256         /* coefficient for low-pass XTAL error tracking */
#define DEFAULT_STAT        30          /* default time interval for statistics */

#define SF_COUNT            6           /* Number of spreading factors to be used */ 
#define RADIO_GROUP_COUNT   2           /* Number of radio groups */
#define DEFAULT_GROUP       1           /* Default radio group */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE TYPES -------------------------------------------------------- */

/* spectral scan */
typedef struct spectral_scan_s {
    bool enable;            /* enable spectral scan thread */
    uint32_t        freq_hz_start;  /* first channel frequency, in Hz */
    uint8_t         nb_chan;        /* number of channels to scan (200kHz between each channel) */
    uint16_t        nb_scan;        /* number of scan points for each frequency scan */
    uint32_t        pace_s;         /* number of seconds between 2 scans in the thread */
} spectral_scan_t;

/* device management */
typedef struct lora_device_s {
    uint32_t        device_adr;
    uint16_t        fcnt;
    bool            adr_status;
} lora_device_t;

/* spreading factor data reading */
/* backup pointers are used in cases where a realloc is required for more space */
typedef struct freq_data_s {
    uint32_t        freq_hz;                        /* Frequency of this spreading factor */
    uint8_t         sf;                             /* Spreading factor entry */
    uint16_t        devices_seen;                   /* Number of seen devices */
    uint16_t        devices_len;                    /* Current length of the devices array */
    uint16_t        messages_seen;                  /* Number of messages seen */
    uint16_t        messages_len;                   /* Current length of the RSSI and SNR arrays */
    uint16_t        unique_messages_seen;           /* Number of messages with unique FCnts */
    uint32_t        message_last_seen;              /* Timestamp of last message recieved */
    uint32_t        *devices;                       /* List of device addresses seen */
    float           *RSSI;                          /* List of RSSI values of recieved messages */
    float           *SNR;                           /* List of SNR values of recieved messages */
    bool            realloc;                        /* Flag to indicate the backups are in use due to REALLOC */
    lora_device_t   *b_devices;                     /* Backup list of devices seen */
    float           *b_RSSI;                        /* Backup list of RSSI values of recieved messages */
    float           *b_SNR;                         /* Backup list of SNR values of recieved messages */
} freq_data_t;

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES (GLOBAL) ------------------------------------------- */

/* statistics collection configuration variables */
static unsigned stat_interval = DEFAULT_STAT; /* time interval (in sec) at which statistics are collected and displayed */

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

/* hardware access control and correction */
pthread_mutex_t mx_concent = PTHREAD_MUTEX_INITIALIZER; /* control access to the concentrator */
static pthread_mutex_t mx_xcorr = PTHREAD_MUTEX_INITIALIZER; /* control access to the XTAL correction */
static bool xtal_correct_ok = false; /* set true when XTAL correction is stable enough */
static double xtal_correct = 1.0;

/* GPS configuration and synchronization */
static char gps_tty_path[64] = "\0"; /* path of the TTY port GPS is connected on */
static int gps_tty_fd = -1; /* file descriptor of the GPS TTY port */
static bool gps_enabled = false; /* is GPS enabled on that gateway ? */

/* GPS time reference */
static pthread_mutex_t mx_timeref = PTHREAD_MUTEX_INITIALIZER; /* control access to GPS time reference */
static bool gps_ref_valid; /* is GPS reference acceptable (ie. not too old) */
static struct tref time_reference_gps; /* time reference used for GPS <-> timestamp conversion */

/* Reference coordinates, for broadcasting (beacon) */
static struct coord_s reference_coord;

/* Enable faking the GPS coordinates of the gateway */
static bool gps_fake_enable; /* enable the feature */

static pthread_mutex_t mx_meas_gps = PTHREAD_MUTEX_INITIALIZER; /* control access to the GPS statistics */
static bool gps_coord_valid; /* could we get valid GPS coordinates ? */
static struct coord_s meas_gps_coord; /* GPS position of the gateway */
static struct coord_s meas_gps_err; /* GPS position of the gateway */

/* Gateway specificities */
static int8_t antenna_gain = 0;


static struct lgw_conf_debug_s debugconf;
static uint32_t nb_pkt_received_ref[16];

/* Interface type */
static lgw_com_type_t com_type = LGW_COM_USB;

/* Radio configuration structs */
static bool radio_group_swapping;
static int radio_group_current; /* Current radio group in use */
static struct lgw_conf_rxrf_s **rfconf; /* Matrix of radio groups [group][radio config] */

/* Spectral Scan */
static spectral_scan_t spectral_scan_params = {
    .enable = false,
    .freq_hz_start = 0,
    .nb_chan = 0,
    .nb_scan = 0,
    .pace_s = 10
};

/* -------------------------------------------------------------------------- */
/* --- PRIVATE STATISTIC VARIABLES (GLOBAL) --------------------------------- */

uint16_t devices_seen;
lora_device_t *devices;

bool realloc_flag;
uint8_t *confirmed_retransmissions;     /* List of retransmission numbers for confirmed messages */
uint8_t *unconfirmed_retransmissions;   /* List of retransmission numbers for UNconfirmed messages */
uint8_t *b_confirmed_retransmissions;   /* Realloc list of retransmission numbers for confirmed messages */
uint8_t *b_unconfirmed_retransmissions; /* Realloc list of retransmission numbers for UNconfirmed messages */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DECLARATION ---------------------------------------- */
static void usage (void);

static void sig_handler(int sigio);

static uint8_t find_channel_no(uint32_t freq);

static int parse_SX130x_configuration(const char * conf_file);

static int parse_gateway_configuration(const char * conf_file);

static int parse_debug_configuration(const char * conf_file);

static void gps_process_sync(void);

static void gps_process_coords(void);

/* threads */
void thread_listen(void);
void thread_gps(void);
void thread_valid(void);

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS  ----------------------------------------- */

static void usage( void )
{
    printf("~~~ Library version string~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
    printf(" %s\n", lgw_version_info());
    printf("~~~ Available options ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
    printf(" -h  print this help\n");
    printf(" -c <filename>  use config file other than 'conf.json'\n");
    printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
}

/**
 * Handle any SIG interrupts.
 */ 
static void sig_handler(int sigio) {
    if (sigio == SIGQUIT) {
        quit_sig = true;
    } else if ((sigio == SIGINT) || (sigio == SIGTERM)) {
        exit_sig = true;
    }
    return;
}

/**
 * Utility function to find the corresponding channel for a given frequency.
 * 
 * @param freq  Frequency to find channel number of
 * @return      Equivalent channel number
 */
static uint8_t find_channel_no(uint32_t freq) {
    double chan = (freq - 915200000) / 200e3;
    return (uint8_t)chan;
}

/**
 * Wrapper function for starting concentrator. Prints stuff nicely :)
 * 
 * @return  -1 on failure, otherwise 0
 */
static int start_sniffer(void) {
    int i;

    i = lgw_start();
    if (i == LGW_HAL_SUCCESS) {
        MSG("INFO: concentrator started, packet can now be received\n");
    } else {
        MSG("ERROR: failed to start the concentrator\n");
        return -1;
    }

    return 0;
}

/**
 * Wrapper function for starting concentrator. Prints stuff nicely :)
 * 
 * @return  -1 on failure, otherwise 0
 */
static int stop_sniffer(void) {

    int i;
    i = lgw_stop();
    if (i == LGW_HAL_SUCCESS) {
        MSG("INFO: concentrator stopped successfully\n");
    } else {
        MSG("WARNING: failed to stop concentrator successfully\n");
        return -1;
    }

    return 0;
}

/**
 * Cleanup function for allocated statistic memory.
 */
static void stat_cleanup(void) {

    int i;

    /* cleanup radio configuration */
    for (i = 0; i < RADIO_GROUP_COUNT; i++)
        free(rfconf[i]);

    free(rfconf);
}

/**
 * Initialise the given radio group for use. Initialises both radios 0 and 1 of the 
 * concentrator. 
 * @param group Radio group to initialise
 * @return      -1 on failure, 0 on success
 */
static int init_radio_group (int group) {

    int i;

    for (i = 0; i < LGW_RF_CHAIN_NB; i++) {
        if (lgw_rxrf_setconf(i, &rfconf[group][i]) != LGW_HAL_SUCCESS) {
            MSG("ERROR: invalid configuration for radio %i\n", i);
            return -1;
        } else {
            MSG("INFO: Group %d radio %d configured correctly\n", group, i);
        }
    }

    return 0;
}

static int parse_SX130x_configuration(const char * conf_file) {
    int i, j, number;
    char param_name[32]; /* used to generate variable parameter names */
    const char *str; /* used to store string value from JSON object */
    const char conf_obj_name[] = "SX130x_conf";
    JSON_Value *root_val = NULL;
    JSON_Value *val = NULL;
    JSON_Object *conf_obj = NULL;
    JSON_Object *conf_ts_obj;
    JSON_Object *conf_sx1261_obj = NULL;
    JSON_Object *conf_scan_obj = NULL;
    JSON_Array *conf_demod_array = NULL;

    struct lgw_conf_board_s boardconf;
    struct lgw_conf_rxif_s ifconf;
    struct lgw_conf_demod_s demodconf;
    struct lgw_conf_ftime_s tsconf;
    struct lgw_conf_sx1261_s sx1261conf;
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
        conf_scan_obj = json_object_get_object(conf_sx1261_obj, "spectral_scan"); /* fetch value (if possible) */
        if (conf_scan_obj == NULL) {
            MSG("INFO: no configuration for Spectral Scan\n");
        } else {
            val = json_object_get_value(conf_scan_obj, "enable"); /* fetch value (if possible) */
            if (json_value_get_type(val) == JSONBoolean) {
                /* Enable background spectral scan thread in packet forwarder */
                spectral_scan_params.enable = (bool)json_value_get_boolean(val);
            } else {
                MSG("WARNING: Data type for spectral_scan.enable seems wrong, please check\n");
            }
            if (spectral_scan_params.enable == true) {
                /* Enable the sx1261 radio hardware configuration to allow spectral scan */
                sx1261conf.enable = true;
                MSG("INFO: Spectral Scan with SX1261 is enabled\n");

                /* Get Spectral Scan Parameters */
                val = json_object_get_value(conf_scan_obj, "freq_start"); /* fetch value (if possible) */
                if (json_value_get_type(val) == JSONNumber) {
                    spectral_scan_params.freq_hz_start = (uint32_t)json_value_get_number(val);
                } else {
                    MSG("WARNING: Data type for spectral_scan.freq_start seems wrong, please check\n");
                }
                val = json_object_get_value(conf_scan_obj, "nb_chan"); /* fetch value (if possible) */
                if (json_value_get_type(val) == JSONNumber) {
                    spectral_scan_params.nb_chan = (uint8_t)json_value_get_number(val);
                } else {
                    MSG("WARNING: Data type for spectral_scan.nb_chan seems wrong, please check\n");
                }
                val = json_object_get_value(conf_scan_obj, "nb_scan"); /* fetch value (if possible) */
                if (json_value_get_type(val) == JSONNumber) {
                    spectral_scan_params.nb_scan = (uint16_t)json_value_get_number(val);
                } else {
                    MSG("WARNING: Data type for spectral_scan.nb_scan seems wrong, please check\n");
                }
                val = json_object_get_value(conf_scan_obj, "pace_s"); /* fetch value (if possible) */
                if (json_value_get_type(val) == JSONNumber) {
                    spectral_scan_params.pace_s = (uint32_t)json_value_get_number(val);
                } else {
                    MSG("WARNING: Data type for spectral_scan.pace_s seems wrong, please check\n");
                }
            }
        }

        /* all parameters parsed, submitting configuration to the HAL */
        if (lgw_sx1261_setconf(&sx1261conf) != LGW_HAL_SUCCESS) {
            MSG("ERROR: Failed to configure the SX1261 radio\n");
            return -1;
        }
    }



    /* Allocate and initialise memory for the radio information structs and statistics */
    rfconf = (struct lgw_conf_rxrf_s**)calloc(RADIO_GROUP_COUNT, sizeof(struct lgw_conf_rxrf_s*));
    for (i = 0; i < RADIO_GROUP_COUNT; i++) {
        rfconf[i] = (struct lgw_conf_rxrf_s*)calloc(LGW_RF_CHAIN_NB, sizeof(struct lgw_conf_rxrf_s));
    }

    radio_group_current = DEFAULT_GROUP;

    /* Group swapping configuration */
    val = json_object_dotget_value(conf_obj, "default_group");
    if (json_value_get_type(val) == JSONBoolean) {
        radio_group_swapping = (bool)json_value_get_boolean(val);
        MSG("INFO: Radio group swapping is %s\n", radio_group_swapping ? "enabled" : "disabled");
    } else {
        MSG("INFO: No group swapping configuration, assuming false\n");
    }

    val = json_object_dotget_value(conf_obj, "group_swapping");
    if (json_value_get_type(val) == JSONBoolean) {
        radio_group_swapping = (bool)json_value_get_boolean(val);
        MSG("INFO: Radio group swapping is %s\n", radio_group_swapping ? "enabled" : "disabled");
    } else {
        MSG("INFO: No group swapping configuration, assuming false\n");
    }

    /* set configuration for RF chains */
    number = 0;
    for (i = 0; i < RADIO_GROUP_COUNT; i++) {
        for (j = 0; j < LGW_RF_CHAIN_NB; j++) {
            snprintf(param_name, sizeof param_name, "radio_%d_%d", i, j); /* compose parameter path inside JSON structure */
            val = json_object_get_value(conf_obj, param_name); /* fetch value (if possible) */
            if (json_value_get_type(val) != JSONObject) {
                MSG("INFO: no configuration for group %d radio %d\n", i, j);
                number++;
                continue;
            }
            /* there is an object to configure that radio, let's parse it */
            snprintf(param_name, sizeof param_name, "radio_%d_%d.enable", i, j);
            val = json_object_dotget_value(conf_obj, param_name);
            if (json_value_get_type(val) == JSONBoolean) {
                rfconf[i][j].enable = (bool)json_value_get_boolean(val);
            } else {
                rfconf[i][j].enable = false;
            }
            if (rfconf[i][j].enable == false) { /* radio disabled, nothing else to parse */
                MSG("INFO: Group %d radio %i disabled\n", i, j);
            } else  { /* radio enabled, will parse the other parameters */
                snprintf(param_name, sizeof param_name, "radio_%d_%d.freq", i, j);
                rfconf[i][j].freq_hz = (uint32_t)json_object_dotget_number(conf_obj, param_name);
                snprintf(param_name, sizeof param_name, "radio_%d_%d.rssi_offset", i, j);
                rfconf[i][j].rssi_offset = (float)json_object_dotget_number(conf_obj, param_name);
                snprintf(param_name, sizeof param_name, "radio_%d_%d.rssi_tcomp.coeff_a", i, j);
                rfconf[i][j].rssi_tcomp.coeff_a = (float)json_object_dotget_number(conf_obj, param_name);
                snprintf(param_name, sizeof param_name, "radio_%d_%d.rssi_tcomp.coeff_b", i, j);
                rfconf[i][j].rssi_tcomp.coeff_b = (float)json_object_dotget_number(conf_obj, param_name);
                snprintf(param_name, sizeof param_name, "radio_%d_%d.rssi_tcomp.coeff_c", i, j);
                rfconf[i][j].rssi_tcomp.coeff_c = (float)json_object_dotget_number(conf_obj, param_name);
                snprintf(param_name, sizeof param_name, "radio_%d_%d.rssi_tcomp.coeff_d", i, j);
                rfconf[i][j].rssi_tcomp.coeff_d = (float)json_object_dotget_number(conf_obj, param_name);
                snprintf(param_name, sizeof param_name, "radio_%d_%d.rssi_tcomp.coeff_e", i, j);
                rfconf[i][j].rssi_tcomp.coeff_e = (float)json_object_dotget_number(conf_obj, param_name);
                snprintf(param_name, sizeof param_name, "radio_%d_%d.type", i, j);
                str = json_object_dotget_string(conf_obj, param_name);
                if (!strncmp(str, "SX1255", 6)) {
                    rfconf[i][j].type = LGW_RADIO_TYPE_SX1255;
                } else if (!strncmp(str, "SX1257", 6)) {
                    rfconf[i][j].type = LGW_RADIO_TYPE_SX1257;
                } else if (!strncmp(str, "SX1250", 6)) {
                    rfconf[i][j].type = LGW_RADIO_TYPE_SX1250;
                } else {
                    MSG("WARNING: invalid radio type: %s (should be SX1255 or SX1257 or SX1250)\n", str);
                }
                snprintf(param_name, sizeof param_name, "radio_%d_%d.single_input_mode", i, j);
                val = json_object_dotget_value(conf_obj, param_name);
                if (json_value_get_type(val) == JSONBoolean) {
                    rfconf[i][j].single_input_mode = (bool)json_value_get_boolean(val);
                } else {
                    rfconf[i][j].single_input_mode = false;
                }

                MSG("INFO: Group %d radio %d enabled (type %s), center frequency %u, RSSI offset %f\n", i, j, str, rfconf[i][j].freq_hz, rfconf[i][j].rssi_offset);
            }
        }
    }

    /* initialise the specific radio group */
    if (number == LGW_RF_CHAIN_NB * RADIO_GROUP_COUNT) {
        MSG("ERROR: No valid radio configurations given\n");
        return -1;
    } else {
        MSG("INFO: %d radios configured\n", number);
    }

    if (init_radio_group(radio_group_current)) {
        MSG("ERROR: Failed to initialise radio group %d\n", i);
        return -1;
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

    /* get interval (in seconds) for statistics display (optional) */
    val = json_object_get_value(conf_obj, "stat_interval");
    if (val != NULL) {
        stat_interval = (unsigned)json_value_get_number(val);
        MSG("INFO: statistics display interval is configured to %u seconds\n", stat_interval);
    }

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
/* --- THREAD 1: RECEIVING PACKETS ------------------------------------------ */
void thread_listen(void) {

    int i; /* loop and temporary variables */

    struct timespec sleep_time = {0, 3000000}; /* 3 ms */

    /* counting variables */
    unsigned long pkt_in_log = 0;

    /* allocate memory for packet fetching and processing */
    struct lgw_pkt_rx_s rxpkt[16]; /* array containing up to 16 inbound packets metadata */
    struct lgw_pkt_rx_s *p; /* pointer on a RX packet */
    int nb_pkt;

    /* local timestamp variables until we get accurate GPS time */
    struct timespec fetch_time;
    struct tm * xt;

    /* recieved packet variables */
    uint8_t  mote_mhdr = 0;
    uint32_t mote_addr = 0;
    uint8_t  mote_fctrl = 0;
    uint16_t mote_fcnt = 0;
    uint8_t  mote_fport = 0;

    /* offset variables */
    uint8_t mote_fopts_len = 0;

    printf("\n\n");
    while (!exit_sig && !quit_sig) {

        /* fetch packets */
        pthread_mutex_lock(&mx_concent);
        nb_pkt = lgw_receive(ARRAY_SIZE(rxpkt), rxpkt);
        pthread_mutex_unlock(&mx_concent);
        
        if (nb_pkt == LGW_HAL_ERROR) {
            MSG("ERROR: failed packet fetch, exiting\n");
            exit(EXIT_FAILURE);
        } else if (nb_pkt == 0) {
            clock_nanosleep(CLOCK_MONOTONIC, 0, &sleep_time, NULL); /* wait a short time if no packets */
        } else {
            /* local timestamp generation until we get accurate GPS time */
            clock_gettime(CLOCK_REALTIME, &fetch_time);
            xt = gmtime(&(fetch_time.tv_sec));
            printf("%04i-%02i-%02i %02i:%02i:%02i.%03liZ\n",(xt->tm_year)+1900,(xt->tm_mon)+1,xt->tm_mday,xt->tm_hour,xt->tm_min,xt->tm_sec,(fetch_time.tv_nsec)/1000000); /* ISO 8601 format */
        }

        /* log packets */
        for (i=0; i < nb_pkt; ++i) {
            p = &rxpkt[i];

            /* writing gateway ID */
            //fprintf(log_file, "\"%08X%08X\",", (uint32_t)(lgwm >> 32), (uint32_t)(lgwm & 0xFFFFFFFF));

            /* writing node MAC address */
            // fputs("\"\",", log_file); // TODO: need to parse payload

            // /* writing UTC timestamp*/
            printf("Packet %d (%ld)\n", i, ++pkt_in_log);

            /* writing packet status and transmission details*/
            printf("Status: ");
            switch(p->status) {
                case STAT_CRC_OK:       printf("\"CRC_OK\" \n"); break;
                case STAT_CRC_BAD:      printf("\"CRC_BAD\"\n"); break;
                case STAT_NO_CRC:       printf("\"NO_CRC\" \n"); break;
                case STAT_UNDEFINED:    printf("\"UNDEF\"  \n"); break;
                default:                printf("\"ERR\"    \n");
            }

            printf("Reception Details: ");
            printf("SNR %.1f, SNR_MIN %.1f, SNR_MAX %.1f, RSSI-C %.1f, RSSI-S %.1f\n", p->snr, p->snr_min, p->snr_max, p->rssic, p->rssis);

            printf("Channel Details:   ");
            printf("Channel %1u, Radio %1u, Frequency %.1lfMHz, Modem Id %2u,", p->if_chain, p->rf_chain, ((double)p->freq_hz / 1e6), p->modem_id);

            printf(" Bandwidth ");
            switch(p->bandwidth) {
                case BW_500KHZ:     printf("500kHz,"); break;
                case BW_250KHZ:     printf("250kHz,"); break;
                case BW_125KHZ:     printf("125kHz,"); break;
                case BW_UNDEFINED:  printf("0     ,"); break;
                default:            printf("-1    ,");
            }

            printf(" SF ");
            switch (p->datarate) {
                case DR_LORA_SF7:   printf("7  "); break;
                case DR_LORA_SF8:   printf("8  "); break;
                case DR_LORA_SF9:   printf("9  "); break;
                case DR_LORA_SF10:  printf("10 "); break;
                case DR_LORA_SF11:  printf("11 "); break;
                case DR_LORA_SF12:  printf("12 "); break;
                default:            printf("ERR");
            }
            printf("AU Channel %d", find_channel_no(p->freq_hz));
            printf("\n");

            /* writing packet data */
            printf("Packet: \n");
            printf("Size: %dbytes\n", p->size);

            /* Lets find our heading data */
            mote_mhdr = p->payload[0];
            mote_addr = p->payload[1] | p->payload[2] << 8 | p->payload[3] << 16 | p->payload[4] << 24;
            mote_fctrl = p->payload[5];
            mote_fcnt = p->payload[6] | p->payload[7] << 8;

            mote_fopts_len = mote_fctrl & 0x0F;

            mote_fport = p->payload[8 + mote_fopts_len];

            printf("MHDR - ");
            switch(mote_mhdr >> 5) {
                case 0b000 : printf("Join Request");            break;
                case 0b001 : printf("Join Accept");             break;
                case 0b010 : printf("Unconfirmed Data Up");     break;
                case 0b011 : printf("Unconfirmed Data Down");   break;
                case 0b100 : printf("Confirmed Data Up");       break;
                case 0b101 : printf("Confirmed Data Down");     break;
                case 0b110 : printf("RFU");                     break;
                case 0b111 : printf("Proprietary");             break;
            }
            printf("\n");

            printf("Packet recieve time: %ums\n", p->count_us);
            printf("Fine timestamp status is %d\n", p->ftime_received);
            if (p->ftime_received) {
                printf("Packet fine recieve time: %u\n", p->ftime);
            } else {

            }

            printf("Device Address %d\n", mote_addr);

            printf("ADR %s\n", (mote_fctrl & 0x80) ? "enabled" : "disabled");

            printf("FCnt %d\n", mote_fcnt);

            printf("FOpts were %d bytes long\n", mote_fopts_len);

            printf("Fport was %d ---> ", mote_fport);
            if (!mote_fport) {
                printf("Device message is MAC commands only!\n");
            } else {
                printf("Device message contains different commands!\n");
            }

            printf("Payload was ");
            for (int w = 8 + mote_fopts_len + 1; w < p->size - 4; w++) {
                printf("%hhx", p->payload[w]);
            }


            if (i == nb_pkt - 1) {
                printf("\n\n");
            }
        }
    }
    printf("%ld Packets heard!\n", pkt_in_log);
}

/* -------------------------------------------------------------------------- */
/* --- THREAD 2: PARSE GPS MESSAGE AND KEEP GATEWAY IN SYNC ----------------- */
static void gps_process_sync(void) {
    struct timespec gps_time;
    struct timespec utc;
    uint32_t trig_tstamp; /* concentrator timestamp associated with PPM pulse */
    int i = lgw_gps_get(&utc, &gps_time, NULL, NULL);

    /* get GPS time for synchronization */
    if (i != LGW_GPS_SUCCESS) {
        MSG("WARNING: [gps] could not get GPS time from GPS\n");
        return;
    }

    /* get timestamp captured on PPM pulse  */
    pthread_mutex_lock(&mx_concent);
    i = lgw_get_trigcnt(&trig_tstamp);
    pthread_mutex_unlock(&mx_concent);
    if (i != LGW_HAL_SUCCESS) {
        MSG("WARNING: [gps] failed to read concentrator timestamp\n");
        return;
    }

    /* try to update time reference with the new GPS time & timestamp */
    pthread_mutex_lock(&mx_timeref);
    i = lgw_gps_sync(&time_reference_gps, trig_tstamp, utc, gps_time);
    pthread_mutex_unlock(&mx_timeref);
    if (i != LGW_GPS_SUCCESS) {
        MSG("WARNING: [gps] GPS out of sync, keeping previous time reference\n");
    }
}

static void gps_process_coords(void) {
    /* position variable */
    struct coord_s coord;
    struct coord_s gpserr;
    int    i = lgw_gps_get(NULL, NULL, &coord, &gpserr);

    /* update gateway coordinates */
    pthread_mutex_lock(&mx_meas_gps);
    if (i == LGW_GPS_SUCCESS) {
        gps_coord_valid = true;
        meas_gps_coord = coord;
        meas_gps_err = gpserr;
        // TODO: report other GPS statistics (typ. signal quality & integrity)
    } else {
        gps_coord_valid = false;
    }
    pthread_mutex_unlock(&mx_meas_gps);
}

void thread_gps(void) {
    /* serial variables */
    char serial_buff[128]; /* buffer to receive GPS data */
    size_t wr_idx = 0;     /* pointer to end of chars in buffer */

    /* variables for PPM pulse GPS synchronization */
    enum gps_msg latest_msg; /* keep track of latest NMEA message parsed */

    /* initialize some variables before loop */
    memset(serial_buff, 0, sizeof serial_buff);

    while (!exit_sig && !quit_sig) {
        size_t rd_idx = 0;
        size_t frame_end_idx = 0;

        /* blocking non-canonical read on serial port */
        ssize_t nb_char = read(gps_tty_fd, serial_buff + wr_idx, LGW_GPS_MIN_MSG_SIZE);
        if (nb_char <= 0) {
            MSG("WARNING: [gps] read() returned value %zd\n", nb_char);
            continue;
        }
        wr_idx += (size_t)nb_char;

        /*******************************************
         * Scan buffer for UBX/NMEA sync chars and *
         * attempt to decode frame if one is found *
         *******************************************/
        while (rd_idx < wr_idx) {
            size_t frame_size = 0;

            /* Scan buffer for UBX sync char */
            if (serial_buff[rd_idx] == (char)LGW_GPS_UBX_SYNC_CHAR) {

                /***********************
                 * Found UBX sync char *
                 ***********************/
                latest_msg = lgw_parse_ubx(&serial_buff[rd_idx], (wr_idx - rd_idx), &frame_size);

                if (frame_size > 0) {
                    if (latest_msg == INCOMPLETE) {
                        /* UBX header found but frame appears to be missing bytes */
                        frame_size = 0;
                    } else if (latest_msg == INVALID) {
                        /* message header received but message appears to be corrupted */
                        MSG("WARNING: [gps] could not get a valid message from GPS (no time)\n");
                        frame_size = 0;
                    } else if (latest_msg == UBX_NAV_TIMEGPS) {
                        gps_process_sync();
                    }
                }
            } else if (serial_buff[rd_idx] == (char)LGW_GPS_NMEA_SYNC_CHAR) {
                /************************
                 * Found NMEA sync char *
                 ************************/
                /* scan for NMEA end marker (LF = 0x0a) */
                char* nmea_end_ptr = memchr(&serial_buff[rd_idx],(int)0x0a, (wr_idx - rd_idx));

                if(nmea_end_ptr) {
                    /* found end marker */
                    frame_size = nmea_end_ptr - &serial_buff[rd_idx] + 1;
                    latest_msg = lgw_parse_nmea(&serial_buff[rd_idx], frame_size);

                    if(latest_msg == INVALID || latest_msg == UNKNOWN) {
                        /* checksum failed */
                        frame_size = 0;
                    } else if (latest_msg == NMEA_RMC) { /* Get location from RMC frames */
                        gps_process_coords();
                    }
                }
            }

            if (frame_size > 0) {
                /* At this point message is a checksum verified frame
                   we're processed or ignored. Remove frame from buffer */
                rd_idx += frame_size;
                frame_end_idx = rd_idx;
            } else {
                rd_idx++;
            }
        } /* ...for(rd_idx = 0... */

        if (frame_end_idx) {
          /* Frames have been processed. Remove bytes to end of last processed frame */
          memcpy(serial_buff, &serial_buff[frame_end_idx], wr_idx - frame_end_idx);
          wr_idx -= frame_end_idx;
        } /* ...for(rd_idx = 0... */

        /* Prevent buffer overflow */
        if ((sizeof(serial_buff) - wr_idx) < LGW_GPS_MIN_MSG_SIZE) {
            memcpy(serial_buff, &serial_buff[LGW_GPS_MIN_MSG_SIZE], wr_idx - LGW_GPS_MIN_MSG_SIZE);
            wr_idx -= LGW_GPS_MIN_MSG_SIZE;
        }
    }
    MSG("\nINFO: End of GPS thread\n");
}

/* -------------------------------------------------------------------------- */
/* --- THREAD 3: CHECK TIME REFERENCE AND CALCULATE XTAL CORRECTION --------- */
void thread_valid(void) {

    /* GPS reference validation variables */
    long gps_ref_age = 0;
    bool ref_valid_local = false;
    double xtal_err_cpy;

    /* variables for XTAL correction averaging */
    unsigned init_cpt = 0;
    double init_acc = 0.0;
    double x;

    /* correction debug */
    // FILE * log_file = NULL;
    // time_t now_time;
    // char log_name[64];

    /* initialization */
    // time(&now_time);
    // strftime(log_name,sizeof log_name,"xtal_err_%Y%m%dT%H%M%SZ.csv",localtime(&now_time));
    // log_file = fopen(log_name, "w");
    // setbuf(log_file, NULL);
    // fprintf(log_file,"\"xtal_correct\",\"XERR_INIT_AVG %u XERR_FILT_COEF %u\"\n", XERR_INIT_AVG, XERR_FILT_COEF); // DEBUG

    /* main loop task */
    while (!exit_sig && !quit_sig) {
        wait_ms(1000);

        /* calculate when the time reference was last updated */
        pthread_mutex_lock(&mx_timeref);
        gps_ref_age = (long)difftime(time(NULL), time_reference_gps.systime);
        if ((gps_ref_age >= 0) && (gps_ref_age <= GPS_REF_MAX_AGE)) {
            /* time ref is ok, validate and  */
            gps_ref_valid = true;
            ref_valid_local = true;
            xtal_err_cpy = time_reference_gps.xtal_err;
            //printf("XTAL err: %.15lf (1/XTAL_err:%.15lf)\n", xtal_err_cpy, 1/xtal_err_cpy); // DEBUG
        } else {
            /* time ref is too old, invalidate */
            gps_ref_valid = false;
            ref_valid_local = false;
        }
        pthread_mutex_unlock(&mx_timeref);

        /* manage XTAL correction */
        if (ref_valid_local == false) {
            /* couldn't sync, or sync too old -> invalidate XTAL correction */
            pthread_mutex_lock(&mx_xcorr);
            xtal_correct_ok = false;
            xtal_correct = 1.0;
            pthread_mutex_unlock(&mx_xcorr);
            init_cpt = 0;
            init_acc = 0.0;
        } else {
            if (init_cpt < XERR_INIT_AVG) {
                /* initial accumulation */
                init_acc += xtal_err_cpy;
                ++init_cpt;
            } else if (init_cpt == XERR_INIT_AVG) {
                /* initial average calculation */
                pthread_mutex_lock(&mx_xcorr);
                xtal_correct = (double)(XERR_INIT_AVG) / init_acc;
                //printf("XERR_INIT_AVG=%d, init_acc=%.15lf\n", XERR_INIT_AVG, init_acc);
                xtal_correct_ok = true;
                pthread_mutex_unlock(&mx_xcorr);
                ++init_cpt;
                // fprintf(log_file,"%.18lf,\"average\"\n", xtal_correct); // DEBUG
            } else {
                /* tracking with low-pass filter */
                x = 1 / xtal_err_cpy;
                pthread_mutex_lock(&mx_xcorr);
                xtal_correct = xtal_correct - xtal_correct/XERR_FILT_COEF + x/XERR_FILT_COEF;
                pthread_mutex_unlock(&mx_xcorr);
                // fprintf(log_file,"%.18lf,\"track\"\n", xtal_correct); // DEBUG
            }
        }

        //printf("Time ref: %s, XTAL correct: %s (%.15lf)\n", ref_valid_local?"valid":"invalid", xtal_correct_ok?"valid":"invalid", xtal_correct); // DEBUG
    }
    MSG("\nINFO: End of validation thread\n");
}

/* -------------------------------------------------------------------------- */
/* --- THREAD 4: SPECTRAL SCAN ---------------------------------------------- */
void thread_spectral_scan(void) {
    int i, x;
    uint32_t freq_hz = spectral_scan_params.freq_hz_start;
    uint32_t freq_hz_stop = spectral_scan_params.freq_hz_start + spectral_scan_params.nb_chan * 200E3;
    int16_t levels[LGW_SPECTRAL_SCAN_RESULT_SIZE];
    uint16_t results[LGW_SPECTRAL_SCAN_RESULT_SIZE];
    struct timeval tm_start;
    lgw_spectral_scan_status_t status;
    bool spectral_scan_started = false;
    bool exit_thread = false;

    /* main loop task */
    while (!exit_sig && !quit_sig) {
        /* Pace the scan thread (1 sec min), and avoid waiting several seconds when exit */
        for (i = 0; i < (int)(spectral_scan_params.pace_s ? spectral_scan_params.pace_s : 1); i++) {
            if (exit_sig || quit_sig) {
                exit_thread = true;
                break;
            }
            wait_ms(1000);
        }
        if (exit_thread == true) {
            break;
        }

        spectral_scan_started = false;

        /* Start spectral scan */
        pthread_mutex_lock(&mx_concent);
        x = lgw_spectral_scan_start(freq_hz, spectral_scan_params.nb_scan);
        if (x != 0) {
            printf("ERROR: spectral scan start failed\n");
        }
        spectral_scan_started = true;
        pthread_mutex_unlock(&mx_concent);

        if (spectral_scan_started == true) {
            /* Wait for scan to be completed */
            status = LGW_SPECTRAL_SCAN_STATUS_UNKNOWN;
            timeout_start(&tm_start);
            do {
                /* handle timeout */
                if (timeout_check(tm_start, 2000) != 0) {
                    printf("ERROR: %s: TIMEOUT on Spectral Scan\n", __FUNCTION__);
                    break;  /* do while */
                }

                /* get spectral scan status */
                pthread_mutex_lock(&mx_concent);
                x = lgw_spectral_scan_get_status(&status);
                pthread_mutex_unlock(&mx_concent);
                if (x != 0) {
                    printf("ERROR: spectral scan status failed\n");
                    break; /* do while */
                }

                /* wait a bit before checking status again */
                wait_ms(10);
            } while (status != LGW_SPECTRAL_SCAN_STATUS_COMPLETED && status != LGW_SPECTRAL_SCAN_STATUS_ABORTED);

            if (status == LGW_SPECTRAL_SCAN_STATUS_COMPLETED) {
                /* Get spectral scan results */
                memset(levels, 0, sizeof levels);
                memset(results, 0, sizeof results);
                pthread_mutex_lock(&mx_concent);
                x = lgw_spectral_scan_get_results(levels, results);
                pthread_mutex_unlock(&mx_concent);
                if (x != 0) {
                    printf("ERROR: spectral scan get results failed\n");
                    continue; /* main while loop */
                }

                /* print results */
                printf("SPECTRAL SCAN RESULTS - %u Hz: ", freq_hz);
                for (i = 0; i < LGW_SPECTRAL_SCAN_RESULT_SIZE; i++) {
                    printf("%u ", results[i]);
                }
                printf("\n");
                printf("SPECTRAL SCAN LEVELS - %u Hz: ", freq_hz);
                for (i = 0; i < LGW_SPECTRAL_SCAN_RESULT_SIZE; i++) {
                    printf("%u ", levels[i]);
                }
                printf("\n");

                /* Next frequency to scan */
                freq_hz += 200000; /* 200kHz channels */
                if (freq_hz >= freq_hz_stop) {
                    freq_hz = spectral_scan_params.freq_hz_start;
                }
            } else if (status == LGW_SPECTRAL_SCAN_STATUS_ABORTED) {
                printf("INFO: %s: spectral scan has been aborted\n", __FUNCTION__);
            } else {
                printf("ERROR: %s: spectral scan status us unexpected 0x%02X\n", __FUNCTION__, status);
            }
        }
    }
    printf("\nINFO: End of Spectral Scan thread\n");
}

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main(int argc, char **argv) {

    /* return management variable */
    int i, j;







    // int **lmao;
    // int loopy = 0;

    // int *other;

    // lmao = (int**)calloc(4, sizeof(int*));
    // for (i = 0; i < 4; i++) {
    //     lmao[i] = (int*)calloc(4, sizeof(int));
    //     for (j = 0; j < 4; j++)
    //         lmao[i][j] = loopy++;
    // }

    // printf("What do the arrays say?\n");

    // other = (int*)realloc(lmao[1], sizeof(int) * 6);

    // for (i = 0; i < 6; i++) {
    //     lmao[1][i] = i;
    // }

    // if (other == NULL)
    //     printf("Realloc unsuccessful");

    // printf("New pointer is %p\n", other);
    // printf("Old pointer is %p\n", lmao[1]);

    // for (i = 0; i < 4; i++) {
    //     printf("Array %d has: ", i);
    //     if (i == 1) {
    //         for (j = 0; j < 6; j++) {
    //             printf("%d ", lmao[i][j]);
    //         }
    //     } else {
    //         for (j = 0; j < 4; j++) {
    //             printf("%d ", lmao[i][j]);
    //         }
    //     }
        
    //     printf("\n");
    // }

    // return EXIT_SUCCESS;


    /* configuration file related */
    const char defaut_conf_fname[] = JSON_CONF_DEFAULT;
    const char * conf_fname = defaut_conf_fname; /* pointer to a string we won't touch */

    /* threads */
    pthread_t thrid_listen;
    pthread_t thrid_gps;
    pthread_t thrid_valid;
    pthread_t thrid_spectral;

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

    /* configuration files management */
    if (access(conf_fname, R_OK) == 0) { /* if there is a global conf, parse it  */
        MSG("INFO: found configuration file %s, parsing it\n", conf_fname);
        i = parse_SX130x_configuration(conf_fname);
        if (i != 0) {
            exit(EXIT_FAILURE);
        }
        i = parse_gateway_configuration(conf_fname);
        if (i != 0) {
            exit(EXIT_FAILURE);
        }
        i = parse_debug_configuration(conf_fname);
        if (i != 0) {
            MSG("INFO: no debug configuration\n");
        }
    } else {
        MSG("ERROR: [main] failed to find any configuration file named %s\n", conf_fname);
        exit(EXIT_FAILURE);
    }

    /* Start GPS so it figures itself out quick */
    if (gps_tty_path[0] != '\0') { /* do not try to open GPS device if no path set */
        i = lgw_gps_enable(gps_tty_path, "ubx7", 0, &gps_tty_fd); /* HAL only supports u-blox 7 for now */
        if (i != LGW_GPS_SUCCESS) {
            printf("WARNING: [main] impossible to open %s for GPS sync (check permissions)\n", gps_tty_path);
            gps_enabled = false;
            gps_ref_valid = false;
        } else {
            printf("INFO: [main] TTY port %s open for GPS synchronization\n", gps_tty_path);
            gps_enabled = true;
            gps_ref_valid = false;
        }
    }

    /* starting the concentrator */
    if (start_sniffer())
        exit(EXIT_FAILURE);

    /* opening log file and writing CSV header*/
    time(&now_time);

    /* main listener for upstream */
    i = pthread_create(&thrid_listen, NULL, (void * (*)(void *))thread_listen, NULL);
    if (i != 0) {
        MSG("ERROR: [main] impossible to create listening thread\n");
        exit(EXIT_FAILURE);
    }

    /* Spectral scan auxiliary thread */
    if (spectral_scan_params.enable == true) {
        i = pthread_create(&thrid_spectral, NULL, (void * (*)(void *))thread_spectral_scan, NULL);
        if (i != 0) {
            MSG("ERROR: [main] impossible to create Spectral Scan thread\n");
            exit(EXIT_FAILURE);
        }
    }

    /* GPS thread management */
    if (gps_enabled == true) {
        i = pthread_create(&thrid_gps, NULL, (void * (*)(void *))thread_gps, NULL);
        if (i != 0) {
            MSG("ERROR: [main] impossible to create GPS thread\n");
            exit(EXIT_FAILURE);
        }
        i = pthread_create(&thrid_valid, NULL, (void * (*)(void *))thread_valid, NULL);
        if (i != 0) {
            MSG("ERROR: [main] impossible to create validation thread\n");
            exit(EXIT_FAILURE);
        }
    }

    /* configure signal handling */
    sigemptyset(&sigact.sa_mask);
    sigact.sa_flags = 0;
    sigact.sa_handler = sig_handler;
    sigaction(SIGQUIT, &sigact, NULL); /* Ctrl-\ */
    sigaction(SIGINT, &sigact, NULL); /* Ctrl-C */
    sigaction(SIGTERM, &sigact, NULL); /* default "kill" command */

    while (!exit_sig && !quit_sig) {
        // Sleep, then report once time is up
        wait_ms(1000 * stat_interval);
        pthread_mutex_lock(&mx_concent);

        if (radio_group_swapping) {
            if (stop_sniffer())
            exit(EXIT_FAILURE);

            radio_group_current++;
            radio_group_current %= RADIO_GROUP_COUNT;

            init_radio_group(radio_group_current);

            if (start_sniffer())
                exit(EXIT_FAILURE);
        }
        
        pthread_mutex_unlock(&mx_concent);
    }

    /* Get all of our main concentrator listening threads to close */
    i = pthread_join(thrid_listen, NULL);
    if (i != 0) {
        printf("ERROR: failed to join upstream thread with %d - %s\n", i, strerror(errno));
    }
    if (spectral_scan_params.enable == true) {
        i = pthread_join(thrid_spectral, NULL);
        if (i != 0) {
            printf("ERROR: failed to join Spectral Scan thread with %d - %s\n", i, strerror(errno));
        }
    }

    if (gps_enabled == true) {
        pthread_cancel(thrid_gps); /* don't wait for GPS thread, no access to concentrator board */
        pthread_cancel(thrid_valid); /* don't wait for validation thread, no access to concentrator board */

        i = lgw_gps_disable(gps_tty_fd);
        if (i == LGW_HAL_SUCCESS) {
            MSG("INFO: GPS closed successfully\n");
        } else {
            MSG("WARNING: failed to close GPS successfully\n");
        }
    }

    if (exit_sig) {
        /* clean up before leaving */
        stop_sniffer();
        stat_cleanup();
    }
    MSG("INFO: Exiting packet sniffer program\n");
    return EXIT_SUCCESS;
}

/* --- EOF ------------------------------------------------------------------ */