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

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a)   (sizeof(a) / sizeof((a)[0]))

/* message macros that utilise the wider logging function */
#define MSG_INFO(format, ...)   print_log("INFO: " format __VA_OPT__(,) __VA_ARGS__)
#define MSG_WARN(format, ...)   print_log("WARNING: " format __VA_OPT__(,) __VA_ARGS__)
#define MSG_ERR(format, ...)    print_log("ERROR: " format __VA_OPT__(,) __VA_ARGS__)
#define MSG_LOG(format, ...)    print_log("LOG: " format __VA_OPT__(,) __VA_ARGS__)

/* Logging macro function */
#define print_log(format, ...) {                                                                    \
    int i = 0;                                                                                      \
    log_file = fopen(log_file_name, "a");                                                           \
    time_t ltime = time(NULL);                                                                      \
    char *time_str = ctime(&ltime);                                                                 \
    time_str[strlen(time_str)-1] = '\0';                                                            \
    if (verbose) {                                                                                  \
        fprintf(stdout, format __VA_OPT__(,) __VA_ARGS__);                                          \
    }                                                                                               \
    fprintf(log_file, "%s - " format , time_str __VA_OPT__(,) __VA_ARGS__);                         \
    i = fclose(log_file);                                                                           \
    if (i < 0) {                                                                                    \
        printf("Failed to close log file\n");                                                       \
        sniffer_exit();                                                                             \
    }                                                                                               \
}                                                                                                   \

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#ifndef VERSION_STRING
    #define VERSION_STRING "undefined"
#endif

#define OPTION_ARGS         ":acdhv"

#define JSON_CONF_DEFAULT   "conf.json"

#define FILE_CPU_TEMP       "/sys/class/thermal/thermal_zone0/temp"
#define FILE_RAM_INFO       "/proc/meminfo"
#define FILE_WLAN0_STATS    "/proc/net/dev"

#define JSON_REPORT_SUFFIX  ".json"

#define JSON_REPORT_ED      "device"

/* JSON key fields for device and channel report information*/
#define JSON_TIME           "@timestamp"
#define JSON_TYPE           "type"
#define JSON_DEVADDR        "DevAddr"
#define JSON_SNR            "SNR"
#define JSON_RSSI           "RSSI"
#define JSON_TOA            "ToA"
#define JSON_ADR            "ADR"
#define JSON_MTYPE          "MType"
#define JSON_CRC            "CRC"
#define JSON_FCNT           "FCnt"
#define JSON_FREQ           "Freq"
#define JSON_SF             "SF"
#define JSON_FPORT          "FPort"
#define JSON_FRMLEN         "FRMLen"
#define JSON_APPEUI         "AppEui"
#define JSON_DEVEUI         "DevEui"

/* JSON key fields for gateway report information */
#define JSON_TMP_CPU        "temp_cpu"
#define JSON_TMP_CON        "temp_con"
#define JSON_RAM_TOTL       "ram_totl"
#define JSON_RAM_AVAL       "ram_aval"

#define JSON_TIME_LEN       80          /* Max length of the timestamp string, including null terminator */
#define JSON_DEVADDR_LEN    9           /* Max length of the device address string, including null terminator */
#define JSON_MTYPE_LEN      4           /* Max length of the message type string, including null terminator */
#define JSON_CRC_LEN        6           /* Max length of the CRC string, including null terminator */
#define JSON_FOPT_LEN       10           /* Max length of an Fopt argument, 0x prefix, max 5 bytes, CID and null terminator */

#define JSON_JR_DATA_LEN    17          /* Max length for two 64bit hex strings - used for AppEui and DevEui, including terminator */


#define MAX_FOPTS_FIELDS    15          /* Maximum number of frame options */

#define MS_CONV             1000        /* conversion define to go from seconds to milliseconds*/
#define UPLOAD_SLEEP        1           /* sleep time of the upload thread to check if its time to upload */
#define DEFAULT_INT_REPORT  900         /* default time interval (seconds) for report uploading */
#define DEFAULT_INT_LOG     1800        /* default time interval (seconds) for log usage */
#define DEFAULT_INT_STATS   4           /* default number of stats generated per log file */

#define SF_COUNT            6           /* Number of spreading factors to be used */ 
#define SF_BASE             7           /* Lowest SF (7->12) */
#define DEFAULT_GROUP_COUNT 2           /* Number of radio groups */
#define DEFAULT_GROUP       1           /* Default radio group */

#define BITRATE_DR0         250         /* Bitrate(bit/sec) for SF12@125KHz*/
#define BITRATE_DR1         440         /* Bitrate(bit/sec) for SF11@125KHz*/
#define BITRATE_DR2         980         /* Bitrate(bit/sec) for SF10@125KHz*/
#define BITRATE_DR3         1760        /* Bitrate(bit/sec) for SF9@125KHz*/
#define BITRATE_DR4         3125        /* Bitrate(bit/sec) for SF8@125KHz*/
#define BITRATE_DR5         5470        /* Bitrate(bit/sec) for SF7@125KHz*/

#define EXTRA_PREAMBLE      8           /* Bytes allocated to LoRa transmission preamble */
#define EXTRA_SYNCWORD      4.25        /* Bytes allocated to LoRa transmission synchronisation word */
#define EXTRA_PHDR          8           /* Bytes allocated to LoRa transmission PHDR and PHDR_CRC */
#define EXTRA_CRC           2           /* Bytes allocated to LoRa transmission CRC */

#define CID_UNKNOWN         "CID_UNKWN" /* Error string used for an unknown CID*/

/* defines for AUTH0 and HTTP POST curl-ing */
#define CURL_TARGET_DASH    0
#define CURL_TARGET_AUTH0   1

#define CURL_SUCCESS        0
#define CURL_RERUN          1
#define CURL_FAILURE        3

#define CURL_SYS_SHIFT      8

#define CURL_TIMEOUT_MIN    3
#define CURL_TIMEOUT_MAX    5
#define CURL_ERRORS_MIN     3
#define CURL_ERRORS_MAX     7

#define CURL_OUTPUT         "out.json"
#define CURL_PREFIX         "curl --connect-timeout 15 -o out.json -s -H \"Content-Type:application/json\""
#define CURL_TEST           "curl --connect-timeout 15 -s"

/* curl errors we actively deal with */
#define CURL_ERR_SUCCESS    0
#define CURL_ERR_NOCONNECT  7 // Do we really need this one???
#define CURL_ERR_TIMEOUT    28
#define CURL_ERR_CANTHANDLE -1
#define CURL_ERR_CODES      99

/* -------------------------------------------------------------------------- */
/* --- PRIVATE TYPES -------------------------------------------------------- */

/* end device report - structured like a data frame! */
typedef struct ed_report_s {
    /* Auxiliary metrics */
    char* timestamp;
    float freq;
    uint8_t sf;
    float snr;
    float rssi;
    float toa;
    /* Fields structured like a data frame! */
    char* mtype;
    char* devaddr;
    bool adr;
    bool ack;
    uint8_t foptslen;
    uint32_t fcnt;
    char** fopts;
    int fport;
    uint8_t frmlength;
    char* crc;
    /* Special JR request items */
    char* appEUI;
    char* devEUI;
    uint8_t size;
} ed_report_t;

/* struct for holding radio information configuration */
typedef struct if_info_s {
    uint8_t radio;
    int32_t freq_if;
} if_info_t;

/* message queue struct type */
struct entry {
    struct lgw_pkt_rx_s rx_pkt;
    STAILQ_ENTRY(entry) entries;
};

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES (GLOBAL) ------------------------------------------- */

/* statistics collection configuration variables */
static unsigned report_interval = DEFAULT_INT_REPORT;   /* time interval (in sec) at which reports are uploaded */
static unsigned log_interval = DEFAULT_INT_LOG;         /* time interval (in sec) at which new log files are used */
static unsigned stats_per_log = DEFAULT_INT_STATS;

/* signal handling variables */
struct sigaction sigact; /* SIGQUIT&SIGINT&SIGTERM signal handling */
static int exit_sig = 0; /* 1 -> application terminates cleanly (shut down hardware, close open files, etc) */
static int quit_sig = 0; /* 1 -> application terminates without shutting down the hardware */

/* STAILQ messaging head and initialisation */
static pthread_mutex_t mx_report_dev = PTHREAD_MUTEX_INITIALIZER; /* control access to the device message queue */
STAILQ_HEAD(stailhead, entry);
static struct stailhead head;

/* configuration variables needed by the application  */
static uint64_t lgwm = 0; /* LoRa gateway MAC address */

/* clock, log file, and statistics management */
static pthread_mutex_t mx_log = PTHREAD_MUTEX_INITIALIZER;  /* control access to the log file */
static int ed_reports_total = 0;                            /* statistics variables */
static uint32_t packets_caught = 0;                         /* Total packets caught */
static bool verbose = false;
static bool continuous = false;
static FILE * log_file = NULL;
static char log_file_name[128];

/* uploading files, auth0 authorisation and HTTP post variables */
static uint8_t failed_curls = 0;
static char file_client_key[80];   /* holds the client_key file string */
static char url_auth0[80];         /* auth0 url for token collection */
static char url_dash[80];          /* desired dashboard url */
static char auth_key[800];         /* holds the bearer token string */

/* hardware access control and correction */
static pthread_mutex_t mx_concent = PTHREAD_MUTEX_INITIALIZER; /* control access to the concentrator */

/* Gateway specificities */
static int8_t antenna_gain = 0;

static struct lgw_conf_debug_s debugconf;
static uint32_t nb_pkt_received_ref[16];

/* Interface type */
static lgw_com_type_t com_type = LGW_COM_USB;

/* Radio configuration structs */
static if_info_t if_info[LGW_MULTI_NB];
static bool radio_group_swapping;
static int radio_group_current; /* Current radio group in use */
static int radio_group_count;
static struct lgw_conf_rxrf_s **rfconf; /* Matrix of radio groups [group][radio config] */

/* JSON writing management and control */
/* the two sets of reports counters and mutexes should allow us to read and move the json encoded ED reports */
/* almost certainly a better way to do this but oh well */
static pthread_mutex_t mx_ed_report_0 = PTHREAD_MUTEX_INITIALIZER; /* control access to the ed_report_0 counter */
static pthread_mutex_t mx_ed_report_1 = PTHREAD_MUTEX_INITIALIZER; /* control access to the ed_report_1 counter */
static char report_string[100];
static int ed_reports_0 = 0;
static int ed_reports_1 = 0;
static int ed_uploads_0 = 0;
static int ed_uploads_1 = 0;

/* Curl failure prevention variables */
static int curl_failures = 0;
static int bad_file_count = 0;

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DECLARATION ---------------------------------------- */

/* General runtime functions */
static void usage(void);

static void sig_handler(int sigio);

/* Gateway statistic gathering functions (ram available and total)*/
static long line_get_first_number (char* line);

static long line_get_specific_number (char* line, uint8_t desired_index);

static float stat_get_ram_available (void);

static float stat_get_ram_total (void);

static float stat_get_temp_cpu (void);

static float stat_get_temp_lgw (void);

static int stat_get_wlan0_rx_tx (long *rx, long *tx);

/* Special functions for caught data frame parsing and handling */
static int fopts_get_mac_len (uint8_t cid);

static int write_cid_ed_report (ed_report_t* report, int index, char* string);

/* End device report writing object handlers */
static ed_report_t* create_ed_report(void);

static void write_ed_report(ed_report_t* report, struct lgw_pkt_rx_s *p, struct tm *xt, struct timespec *fetch_time);

static void reset_ed_report(ed_report_t* report);

static void destroy_ed_report(ed_report_t* report);

/* Report object encoding functions */
static void create_file_string(char* dest, char* file_type, int index_mutex, int index_file);

static void encode_ed_report(ed_report_t *info, int index_mutex, int index_file);

static void generate_sniffer_stats(void);

/* Auxilliary help functions */

static int sniffer_start(void);

static int sniffer_stop(void);

static void sniffer_exit(void);

/* Radio configuration functions */
static int init_radio_group(int group);

static void stat_cleanup(void);

/* Configuration parsing files */
static int parse_SX130x_configuration(const char * conf_file);

static int parse_gateway_configuration(const char * conf_file);

static int parse_debug_configuration(const char * conf_file);

static int parse_upload_configuration(const char * conf_file);

/* Log file interaction */
static void log_open (void);

/* curl HTTP post handling functions */
static int curl_read_system (int system_output);

static int curl_handle_timeout (char* url_to_check);

static int curl_handle_output (int curl_target);

static int curl_parse (JSON_Value *root_val, int curl_target);

static int curl_get_auth0 (void);

static int curl_upload_file (const char * upload_file);

static int save_unknown_response (const char* file_in);

/* threads */
void thread_listen(void);
void thread_gps(void);
void thread_valid(void);
void thread_spectral_scan(void);
void thread_encode(void);

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
 * Read a line given some string and return the first number found.
 * 
 * If used when reading from file ensure that the stream is not closed.
 * 
 * @param line  The string to search
 * @return      The number first found, else -1
*/
static long line_get_first_number (char* line) {

    long num = -1;

    while(*line) {
        if (isdigit(*line)) {
            num = strtol(line, &line, 10);
            break;
        }
        line++;
    }

    return num;
}

/**
 * Reads a line and returns a number at the specific index. This refers to whole numbers
 * seperated by white space.
 * 
 * NOTE: The line pointer is never reset to its original position...
 * 
 * @param line          The char pointer to shift
 * @param desired_index The desired number index (range of 0 to 255)
 * @return              -1 if the desired number couldnt be found, else the number
*/
static long line_get_specific_number (char* line, uint8_t desired_index) {

    long num = -1;
    uint8_t index = 0;
    bool is_digit = false;

    while(*line) {
        if (isdigit(*line)) {
            /* We have found a digit, are we at our desired index though? */
            if (index == desired_index) {
                num = strtol(line, &line, 10);
                break;
            }
            is_digit = true;
        } else {
            /* We have found something that isnt a digit, lets handle */
            if (is_digit) {
                is_digit = false;
                index++;
            }
        }
        line++;
    }

    return num;
}

/**
 * Returns the current system RAM availability and returns it in MiB.
 * 
 * @return  Float of the current available system RAM in MiB, otherwise 0 indicating failure
*/
static float stat_get_ram_available (void) {

    FILE* file = NULL;
    char* line = NULL;
    size_t len = 0;
    float available = 0;

    file = fopen(FILE_RAM_INFO, "r");

    if (file == NULL) {
        MSG_ERR("[stat_get_ram_available] Failed to open %s\n", FILE_RAM_INFO);
        return available;
    }

    /* Sets pointer position to the third line of the file */
    for (int i = 0; i < 3; i++)
        getline(&line, &len, file);

    available = (float)line_get_first_number(line);

    if (available == -1) {
        MSG_ERR("[stat_get_ram_available] Unable to get available RAM value\n");
        available = 0;
    } else {
        available = available / 1024; /* Convert to mibibytes */
    }

    fclose(file);

    return available;
}

/**
 * Returns the current total RAM availability and returns it in MiB.
 * 
 * @return  Float of the current total system RAM in MiB, otherwise 0 indicating failure
*/
static float stat_get_ram_total (void) {

    FILE* file = NULL;
    char* line = NULL;
    size_t len = 0;
    float available = 0;

    file = fopen(FILE_RAM_INFO, "r");

    if (file == NULL) {
        MSG_ERR("[stat_get_ram_total] Failed to open %s\n", FILE_RAM_INFO);
        return available;
    }

    getline(&line, &len, file); /* Get first line of file*/

    available = (float)line_get_first_number(line);

    if (available == -1) {
        MSG_ERR("[stat_get_ram_total] Unable to get total RAM value\n");
        available = 0;
    } else {
        available = available / 1024; /* Convert to mibibytes */
    }

    fclose(file);

    return available;
}

/**
 * Returns the current temperature of the CPU.
 * 
 * @return  Temperature of the CPU, else 0 on error
*/
static float stat_get_temp_cpu (void) {

    FILE* file = NULL;
    char* line = NULL;
    size_t len = 0;
    float temp = 0;

    file = fopen(FILE_CPU_TEMP, "r");

    if (file == NULL) {
        MSG_ERR("[stat_get_temp_cpu] Unable to open %s\n", FILE_CPU_TEMP);
        return temp;
    }

    getline(&line, &len, file); /* Get first line of file*/

    temp = (float)strtol(line, &line, 10) / 1000.0;

    fclose(file);

    return temp;
}

/**
 * Gets the temperature of the lora gateway concentrator card.
 * 
 * @return Returns the termpature as a float, 0 on error otherwise
*/
static float stat_get_temp_lgw (void) {

    int i;
    float temp = 0;

    pthread_mutex_lock(&mx_concent);
    i = lgw_get_temperature(&temp);
    pthread_mutex_unlock(&mx_concent);

    if (i == LGW_HAL_ERROR) {
        MSG_ERR("Failed to acquire concentrator temp\n");
        temp = 0;
    }

    return temp;
}

/**
 * Get the rx and tx stats of the wlan0 interface in bytes. Returned through pointers.
 * 
 * @param rx
 * @param tx
 * @return      0 on success, -1 on error
*/
static int stat_get_wlan0_rx_tx (long *rx, long *tx) {

    FILE* file = NULL;
    char* line = NULL;
    size_t len = 0;
    long rx_bytes = 0;
    long tx_bytes = 0;

    file = fopen(FILE_WLAN0_STATS, "r");

    if (file == NULL) {
        MSG_ERR("[stat_get_temp_cpu] Unable to open %s\n", FILE_WLAN0_STATS);
        return -1;
    }

    /* Sets pointer position to the third line of the file */
    for (int i = 0; i < 6; i++)
        getline(&line, &len, file);

    /* RX packets is at index 1, TX is at 9*/
    rx_bytes = line_get_specific_number(line, 1);

    if (rx_bytes == -1) {
        MSG_ERR("[stat_get_temp_cpu] Failed to get desired RX index\n");
        return -1;
    }

    tx_bytes = line_get_specific_number(line, 9);

    if (tx_bytes == -1) {
        MSG_ERR("[stat_get_temp_cpu] Failed to get desired TX index\n");
        return -1;
    }

    *rx = rx_bytes;
    *tx = tx_bytes;

    fclose(file);

    return 0;
}

/**
 * Returns the expected length of the MAC command data fields given some CID. Returns an 
 * error if the CID provides a value that cannot be handled (i.e. invalid CIDs 0 or 1, RFU,
 * or proprietary)
 * 
 * @param cid   The command identifier
 * @return      -1 if the CID
*/
static int fopts_get_mac_len (uint8_t cid) {

    static const uint8_t mac_len[] = {
        -1, -1, 2, 4, 1, 4, 2, 5, 1, 1, 5, -1, -1, 5, -1, -1, -1
    };

    if (cid > 0x80)
        return -1;

    return mac_len[cid];
}

/**
 * Write a CID entry to the ed_report struct.
 * Allocates space at the given index and copies the string in.
 * @param report    Report object pointer
 * @param index     Index to write to
 * @param string    String to enter
 * @return          0 on success, -1 otherwise
*/
static int write_cid_ed_report (ed_report_t* report, int index, char* string) {

    int ret = 0;

    report->fopts[index] = malloc(sizeof(char) * JSON_FOPT_LEN);

    if (report->fopts[index] == NULL) {
        MSG_ERR("[write_cid_ed_report] Unable to allocate memory to write CID\n");
        return -1;
    }

    ret = sprintf(report->fopts[index], string);

    if (ret < 0) {
        MSG_ERR("[write_cid_ed_report] Failed to sprintf CID string to e_report\n");
        return -1;
    }

    return 0;
}

/**
 * Create the appropriate string for a JSON file. User can define if the 
 * file is for a channel or device report, as well as its number.
 * 
 * @param dest          Char* to allocate the string to
 * @param file_type     prefix of the file type (i.e. JSON_REPORT_ED)
 * @param index_mutex   Index of the mutex to use
 * @param index_file    Index of the file
*/
static void create_file_string(char* dest, char* file_type, int index_mutex, int index_file) {

    char number[20];
    memset(dest, 0, 100);
    sprintf(number, "%d_%d", index_mutex, index_file);
    strcat(dest, file_type);
    strcat(dest, number);
    strcat(dest, JSON_REPORT_SUFFIX);
}

/**
 * Create (allocate memory) for an ED report object.
 * 
 * @return  Pointer to an ed_report_t object
*/
static ed_report_t* create_ed_report(void) {

    ed_report_t *ed_report =    (ed_report_t*)calloc(1, sizeof(ed_report_t));
    ed_report->timestamp =      (char*)calloc(JSON_TIME_LEN, sizeof(char));
    ed_report->devaddr =        (char*)calloc(JSON_DEVADDR_LEN, sizeof(char));
    ed_report->mtype =          (char*)calloc(JSON_MTYPE_LEN, sizeof(char));
    ed_report->crc =            (char*)calloc(JSON_CRC_LEN, sizeof(char));
    ed_report->fopts =          (char**)calloc(MAX_FOPTS_FIELDS, sizeof(char*));
    ed_report->appEUI =         (char*)calloc(JSON_JR_DATA_LEN, sizeof(char));
    ed_report->devEUI =         (char*)calloc(JSON_JR_DATA_LEN, sizeof(char));
    ed_report->foptslen = 0; // Foptslen on creation MUST be 0
    return ed_report;
}

/**
 * Fill an ed_report_t object with the appropriate data given the lgw_pkt_rx_s and timestamp.
 * 
 * @param report    Pointer to the ed_report_t object to fill
 * @param p         Pointer to the lgw_pkt_rx_s containing the incoming transmission data
 * @param timestamp Pointer to a tm struct timestamp object
 * 
 * @return          None
*/
static void write_ed_report(ed_report_t* report, struct lgw_pkt_rx_s *p, struct tm *xt, struct timespec *fetch_time) {

    /* airtime calculation variable */
    float airtime = 0;

    /* CID sprintf array */
    char number[JSON_FOPT_LEN];

    /* recieved packet variables that require a bit of shifting */
    uint64_t mote_mac_cmd = 0;
    uint64_t dev_eui, app_eui;
    uint32_t mote_addr = 0;
    int mac_len = 0;
    uint8_t mote_mhdr = 0;
    uint8_t mote_cid = 0;
    uint8_t offset = 0;

    /* Timestamp */
    sprintf(report->timestamp, "%04i-%02i-%02iT%02i:%02i:%02i.%03liZ",(xt->tm_year)+1900,(xt->tm_mon)+1,xt->tm_mday,xt->tm_hour,xt->tm_min,xt->tm_sec,(fetch_time->tv_nsec)/1000000); /* ISO 8601 format */
    
    /* MHDR and Message Type */
    mote_mhdr = p->payload[0];
    
    /* CRC status */
    switch(p->status) {
        case STAT_CRC_OK:       sprintf(report->crc, "OK");     break;
        case STAT_CRC_BAD:      sprintf(report->crc, "BAD");    break;
        case STAT_NO_CRC:       sprintf(report->crc, "NONE");   break;
        case STAT_UNDEFINED:    sprintf(report->crc, "UNDEF");  break;
        default:                sprintf(report->crc, "ERR");
    }

    /* General packet statistics - Freq, SF, SNR, RSSI, ToA, */
    report->freq = ((double)p->freq_hz / 1e6);

    

    report->sf = p->datarate;
    report->snr = p->snr;
    report->rssi = p->rssis;

    /* Time on air calculation */
    airtime = (p->size + EXTRA_PREAMBLE + EXTRA_SYNCWORD + EXTRA_PHDR + EXTRA_CRC) * 8;
    switch (p->datarate) {
        case DR_LORA_SF7:   airtime = airtime / BITRATE_DR5; break;
        case DR_LORA_SF8:   airtime = airtime / BITRATE_DR4; break;
        case DR_LORA_SF9:   airtime = airtime / BITRATE_DR3; break;
        case DR_LORA_SF10:  airtime = airtime / BITRATE_DR2; break;
        case DR_LORA_SF11:  airtime = airtime / BITRATE_DR1; break;
        case DR_LORA_SF12:  airtime = airtime / BITRATE_DR0; break;
        default:            MSG_ERR("Unknown spreading factor found");
    }
    report->toa = airtime * 1e3; // In ms 

    /* Join request case... very important */
    if (mote_mhdr >> 5 == 0b000) {
        // Special case for JR
        sprintf(report->mtype, "JR");

        // First 8 bytes are the APP EUI
        app_eui = (uint64_t)p->payload[1] | (uint64_t)p->payload[2] << 8 | (uint64_t)p->payload[3] << 16 | (uint64_t)p->payload[4] << 24 | (uint64_t)p->payload[5] << 32 | (uint64_t)p->payload[6] << 40 | (uint64_t)p->payload[7] << 48 | (uint64_t)p->payload[8] << 56;
        sprintf(report->appEUI, "%.16llx", app_eui);
        
        // Second 8 bytes are the Dev EUI
        dev_eui = (uint64_t)p->payload[9] | (uint64_t)p->payload[10] << 8 | (uint64_t)p->payload[11] << 16 | (uint64_t)p->payload[12] << 24 | (uint64_t)p->payload[13] << 32 | (uint64_t)p->payload[14] << 40 | (uint64_t)p->payload[15] << 48 | (uint64_t)p->payload[16] << 56;
        sprintf(report->devEUI, "%.16llx", dev_eui);

        // Then there are 2 DevNonce fields
        // Uploading this for a check - JR should be (at max) 19 bytes?
        report->frmlength = p->size;
        
        return;
    }

    switch(mote_mhdr >> 5) {
        // case 0b000 : sprintf(report->mtype, "JR");  break; // Not used as this results in a different message type
        // case 0b001 : sprintf(report->mtype, "JA");  break; // Not used as this results in a different message type
        case 0b010 : sprintf(report->mtype, "UDU"); break;
        case 0b011 : sprintf(report->mtype, "UDD"); break;
        case 0b100 : sprintf(report->mtype, "CDU"); break;
        case 0b101 : sprintf(report->mtype, "CDD"); break;
        case 0b110 : sprintf(report->mtype, "RFU"); break;
        case 0b111 : sprintf(report->mtype, "PRP"); break;
    }

    /* FHDR breakdown - DevAddr, FCtrl, FCnt, FOpts */
    /* DevAddr */
    mote_addr = p->payload[1] | p->payload[2] << 8 | p->payload[3] << 16 | p->payload[4] << 24;
    sprintf(report->devaddr, "%.8x", mote_addr);

    /* FCtrl - ADR, ACK, FOptsLen */
    report->adr = (p->payload[5] & 0x80) ? true : false;
    report->ack = (p->payload[5] & 0x20) ? true : false;
    report->foptslen = p->payload[5] & 0x0F;

    /* FCnt */
    report->fcnt = p->payload[6] | p->payload[7] << 8;

    /* FOpts - if any */
    // if (report->foptslen && p->status ==STAT_CRC_OK) {
    //     MSG_INFO("Device was %s\n", report->devaddr);
    //     MSG_INFO("ADR was %s\n", report->adr ? "TRUE" : "FALSE");
    //     MSG_INFO("ACK was %s\n", report->ack ? "TRUE" : "FALSE");
    //     MSG_INFO("FOptslen was %d\n", report->foptslen);
    //     MSG_INFO("FOPTS were: \n");

    //     for (int i = 0; i < report->foptslen; i++) {

    //         mote_cid = p->payload[8 + offset];
    //         sprintf(number, "0x%.2x", mote_cid);
    //         //DEBUG LINES
    //         MSG_INFO("%s\n", number);
    //         offset++;

    //         memset(number, 10, sizeof(char));
    //     }
    // } else if (p->status == STAT_CRC_OK && report->ack) {
    //     MSG_INFO("I noticed device %s sent an ACK...\n", report->devaddr);
    // }

    /* FPort and Foptslen */
    report->fport = (p->size == 8 + report->foptslen) ? -1 : p->payload[8 + offset];

    if (report->fport == -1) {
        report->frmlength = p->size - 8 - report->foptslen; // 8 is (7 FHDR + 1 MHDR)
    } else {
        report->frmlength = p->size - 8 - report->foptslen - 1; // 8 is (7 FHDR + 1 MHDR) and 1 is FPORT
    }

    // if (report->foptslen && p->status ==STAT_CRC_OK) {
    //     MSG_INFO("FPort was %d\n", report->fport);
    //     MSG_INFO("Payload original size was %d and FRM is %d\n", p->size, report->frmlength);
    //     MSG_INFO("\n");
    // }
}

/**
 * Set memory of pointers within ed_report_t object to 0.
 * 
 * @param report    Pointer to the ed_report_t object to clear
*/
static void reset_ed_report(ed_report_t* report) {

    /* Set all allocated memory sections to zero */
    memset(report->timestamp, 0, sizeof(char) * JSON_TIME_LEN);
    memset(report->devaddr, 0, sizeof(char) * JSON_DEVADDR_LEN);
    memset(report->mtype, 0, sizeof(char) * JSON_MTYPE_LEN);
    memset(report->crc, 0, sizeof(char) * JSON_CRC_LEN);
    memset(report->appEUI, 0, sizeof(char) * JSON_JR_DATA_LEN);
    memset(report->devEUI, 0, sizeof(char) * JSON_JR_DATA_LEN);

    /* Free the Fopt strings, they may not be needed for the next packet */
    for (int i = 0; i < report->foptslen; i++) {
        free((void*)report->fopts[i]);
    }
}

/**
 * Destroy the allocated memory associated with a ed_report_t object
 * 
 * @param report    Pointer to the ed_report_t object to free 
*/
static void destroy_ed_report(ed_report_t* report) {

    /* Free general strings */
    free((void*)report->timestamp);
    free((void*)report->devaddr);
    free((void*)report->mtype);
    free((void*)report->crc);
    free((void*)report->appEUI);
    free((void*)report->devEUI);

    /* Free the Fopt strings and its parent pointer */
    for (int i = 0; i < report->foptslen; i++) {
        free((void*)report->fopts[i]);
    }
    free((void*)report->fopts);

    /* Free the entire report */
    free((void*)report);
}

/**
 * Create a JSON report for a given end device information struct.
 * 
 * @param info          ed_report_t containing all relevant transmission information
 * @param index_mutex   Index of the mutex to use
 * @param index_file    Index of the file
*/
static void encode_ed_report(ed_report_t *info, int index_mutex, int index_file) {

    JSON_Value* root_value;
    JSON_Object* root_object;
    FILE* file;
    char* serialized_string = NULL;
    char report_string[100];

    create_file_string(report_string, JSON_REPORT_ED, index_mutex, index_file);

    file = fopen(report_string, "w");

    root_value = json_value_init_object();
    root_object = json_value_get_object(root_value);
    // Write the consistent fields first
    json_object_set_string(root_object, JSON_TIME,      info->timestamp);
    json_object_set_string(root_object, JSON_TYPE,      JSON_REPORT_ED);
    
    json_object_set_string(root_object, JSON_MTYPE,     info->mtype);
    json_object_set_string(root_object, JSON_CRC,       info->crc);
    json_object_set_number(root_object, JSON_FREQ,      info->freq);
    json_object_set_number(root_object, JSON_SF,        info->sf);
    json_object_set_number(root_object, JSON_RSSI,      info->rssi);
    json_object_set_number(root_object, JSON_TOA,       info->toa);
    json_object_set_number(root_object, JSON_FRMLEN,    info->frmlength);
    json_object_set_number(root_object, JSON_SNR,       info->snr);

    // If the message type is a JR
    if (!strcmp("JR", info->mtype)) {
        json_object_set_string(root_object, JSON_MTYPE,     info->mtype);
        json_object_set_string(root_object, JSON_CRC,       info->crc);
    } else {
        // If the message type is other
        json_object_set_number(root_object, JSON_FCNT,      info->fcnt);
        json_object_set_string(root_object, JSON_DEVADDR,   info->devaddr);
        json_object_set_boolean(root_object, JSON_ADR,      info->adr);
        json_object_set_number(root_object, JSON_FPORT,     info->fport);
    }
    serialized_string = json_serialize_to_string(root_value);
    fputs(serialized_string, file);

    json_free_serialized_string(serialized_string);
    json_value_free(root_value);
    fclose(file);
}

/**
 * Gather all stats relative to the sniffer and print them.
 *
 * Currently acquires:
 *  - Pi CPU temp
 *  - LGW concentrator temp
 *  - Total system ram
 *  - Total available system ram
*/
static void generate_sniffer_stats (void) {

    float temp_cpu, temp_con, ram_total, ram_available;
    long rx = 0;
    long tx = 0;

    temp_cpu = stat_get_temp_cpu();
    temp_con = stat_get_temp_lgw();
    ram_total = stat_get_ram_total();
    ram_available = stat_get_ram_available();

    stat_get_wlan0_rx_tx(&rx, &tx);

    MSG_INFO("Pi Temp: %fC\n", temp_cpu);
    MSG_INFO("LGW Temp: %fC\n", temp_con);
    MSG_INFO("Total RAM: %fMiB\n", ram_total);
    MSG_INFO("Available RAM %fMiB\n", ram_available);
    MSG_INFO("WLAN0 RX: %lu\n", rx);
    MSG_INFO("WLAN0 TX: %lu\n", tx);
    MSG_INFO("Total packets caught %lu\n", (unsigned long)packets_caught);
    MSG_INFO("Total packets uploaded %d\n", ed_reports_total);

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


/**
 * Cleanup function for allocated statistic memory.
 */
static void stat_cleanup(void) {

    int i;

    /* cleanup radio configuration */
    for (i = 0; i < radio_group_count; i++) {
        free(rfconf[i]);
    }
        
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
            MSG_ERR("invalid configuration for radio %i\n", i);
            return -1;
        } else {
            MSG_INFO("Group %d radio %d configured correctly\n", group, i);
        }
    }

    return 0;
}

static int parse_SX130x_configuration(const char * conf_file) {
    int i, j, number;
    char param_name[40]; /* used to generate variable parameter names */
    const char *str; /* used to store string value from JSON object */
    const char conf_obj_name[] = "SX130x_conf";
    JSON_Value *root_val = NULL;
    JSON_Value *val = NULL;
    JSON_Object *conf_obj = NULL;
    JSON_Object *conf_ts_obj;
    JSON_Object *conf_sx1261_obj = NULL;
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
        MSG_ERR("%s is not a valid JSON file\n", conf_file);
        exit(EXIT_FAILURE);
    }

    /* point to the gateway configuration object */
    conf_obj = json_object_get_object(json_value_get_object(root_val), conf_obj_name);
    if (conf_obj == NULL) {
        MSG_INFO("%s does not contain a JSON object named %s\n", conf_file, conf_obj_name);
        return -1;
    } else {
        MSG_INFO("%s does contain a JSON object named %s, parsing SX1302 parameters\n", conf_file, conf_obj_name);
    }

    /* set board configuration */
    memset(&boardconf, 0, sizeof boardconf); /* initialize configuration structure */
    str = json_object_get_string(conf_obj, "com_type");
    if (str == NULL) {
        MSG_ERR("com_type must be configured in %s\n", conf_file);
        return -1;
    } else if (!strncmp(str, "SPI", 3) || !strncmp(str, "spi", 3)) {
        boardconf.com_type = LGW_COM_SPI;
    } else if (!strncmp(str, "USB", 3) || !strncmp(str, "usb", 3)) {
        boardconf.com_type = LGW_COM_USB;
    } else {
        MSG_ERR("invalid com type: %s (should be SPI or USB)\n", str);
        return -1;
    }
    com_type = boardconf.com_type;
    str = json_object_get_string(conf_obj, "com_path");
    if (str != NULL) {
        strncpy(boardconf.com_path, str, sizeof boardconf.com_path);
        boardconf.com_path[sizeof boardconf.com_path - 1] = '\0'; /* ensure string termination */
    } else {
        MSG_ERR("com_path must be configured in %s\n", conf_file);
        return -1;
    }
    val = json_object_get_value(conf_obj, "lorawan_public"); /* fetch value (if possible) */
    if (json_value_get_type(val) == JSONBoolean) {
        boardconf.lorawan_public = (bool)json_value_get_boolean(val);
    } else {
        MSG_WARN("Data type for lorawan_public seems wrong, please check\n");
        boardconf.lorawan_public = false;
    }
    val = json_object_get_value(conf_obj, "clksrc"); /* fetch value (if possible) */
    if (json_value_get_type(val) == JSONNumber) {
        boardconf.clksrc = (uint8_t)json_value_get_number(val);
    } else {
        MSG_WARN("Data type for clksrc seems wrong, please check\n");
        boardconf.clksrc = 0;
    }
    val = json_object_get_value(conf_obj, "full_duplex"); /* fetch value (if possible) */
    if (json_value_get_type(val) == JSONBoolean) {
        boardconf.full_duplex = (bool)json_value_get_boolean(val);
    } else {
        MSG_WARN("Data type for full_duplex seems wrong, please check\n");
        boardconf.full_duplex = false;
    }
    MSG_INFO("com_type %s, com_path %s, lorawan_public %d, clksrc %d, full_duplex %d\n", (boardconf.com_type == LGW_COM_SPI) ? "SPI" : "USB", boardconf.com_path, boardconf.lorawan_public, boardconf.clksrc, boardconf.full_duplex);
    /* all parameters parsed, submitting configuration to the HAL */
    if (lgw_board_setconf(&boardconf) != LGW_HAL_SUCCESS) {
        MSG_ERR("Failed to configure board\n");
        return -1;
    }

    /* set antenna gain configuration */
    val = json_object_get_value(conf_obj, "antenna_gain"); /* fetch value (if possible) */
    if (val != NULL) {
        if (json_value_get_type(val) == JSONNumber) {
            antenna_gain = (int8_t)json_value_get_number(val);
        } else {
            MSG_WARN("Data type for antenna_gain seems wrong, please check\n");
            antenna_gain = 0;
        }
    }
    MSG_INFO("antenna_gain %d dBi\n", antenna_gain);

    /* set timestamp configuration */
    conf_ts_obj = json_object_get_object(conf_obj, "fine_timestamp");
    if (conf_ts_obj == NULL) {
        MSG_INFO("%s does not contain a JSON object for fine timestamp\n", conf_file);
    } else {
        val = json_object_get_value(conf_ts_obj, "enable"); /* fetch value (if possible) */
        if (json_value_get_type(val) == JSONBoolean) {
            tsconf.enable = (bool)json_value_get_boolean(val);
        } else {
            MSG_WARN("Data type for fine_timestamp.enable seems wrong, please check\n");
            tsconf.enable = false;
        }
        if (tsconf.enable == true) {
            str = json_object_get_string(conf_ts_obj, "mode");
            if (str == NULL) {
                MSG_ERR("fine_timestamp.mode must be configured in %s\n", conf_file);
                return -1;
            } else if (!strncmp(str, "high_capacity", 13) || !strncmp(str, "HIGH_CAPACITY", 13)) {
                tsconf.mode = LGW_FTIME_MODE_HIGH_CAPACITY;
            } else if (!strncmp(str, "all_sf", 6) || !strncmp(str, "ALL_SF", 6)) {
                tsconf.mode = LGW_FTIME_MODE_ALL_SF;
            } else {
                MSG_ERR("invalid fine timestamp mode: %s (should be high_capacity or all_sf)\n", str);
                return -1;
            }
            MSG_INFO("Configuring precision timestamp with %s mode\n", str);

            /* all parameters parsed, submitting configuration to the HAL */
            if (lgw_ftime_setconf(&tsconf) != LGW_HAL_SUCCESS) {
                MSG_ERR("Failed to configure fine timestamp\n");
                return -1;
            }
        } else {
            MSG_INFO("Configuring legacy timestamp\n");
        }
    }

    /* set SX1261 configuration */
    memset(&sx1261conf, 0, sizeof sx1261conf); /* initialize configuration structure */
    conf_sx1261_obj = json_object_get_object(conf_obj, "sx1261_conf"); /* fetch value (if possible) */
    if (conf_sx1261_obj == NULL) {
        MSG_INFO("no configuration for SX1261\n");
    } else {
        /* Global SX1261 configuration */
        str = json_object_get_string(conf_sx1261_obj, "spi_path");
        if (str != NULL) {
            strncpy(sx1261conf.spi_path, str, sizeof sx1261conf.spi_path);
            sx1261conf.spi_path[sizeof sx1261conf.spi_path - 1] = '\0'; /* ensure string termination */
        } else {
            MSG_INFO("SX1261 spi_path is not configured in %s\n", conf_file);
        }
        val = json_object_get_value(conf_sx1261_obj, "rssi_offset"); /* fetch value (if possible) */
        if (json_value_get_type(val) == JSONNumber) {
            sx1261conf.rssi_offset = (int8_t)json_value_get_number(val);
        } else {
            MSG_WARN("Data type for sx1261_conf.rssi_offset seems wrong, please check\n");
            sx1261conf.rssi_offset = 0;
        }

        /* all parameters parsed, submitting configuration to the HAL */
        if (lgw_sx1261_setconf(&sx1261conf) != LGW_HAL_SUCCESS) {
            MSG_ERR("Failed to configure the SX1261 radio\n");
            return -1;
        }
    }

    /* Radio group swapping configuration */
    val = json_object_dotget_value(conf_obj, "group_swapping");
    if (json_value_get_type(val) == JSONBoolean) {
        radio_group_swapping = (bool)json_value_get_boolean(val);
        MSG_INFO("Radio group swapping is %s\n", radio_group_swapping ? "enabled" : "disabled");
    } else {
        MSG_INFO("No group swapping configuration, assuming false\n");
    }

    val = json_object_dotget_value(conf_obj, "default_group");
    if (json_value_get_type(val) == JSONNumber) {
        radio_group_current = (int)json_value_get_number(val);
        MSG_INFO("Custom radio group %d selected\n", radio_group_current);
    } else {
        radio_group_current = DEFAULT_GROUP;
        MSG_INFO("Utilising default radio group %d\n", radio_group_current);
    }

    val = json_object_dotget_value(conf_obj, "radio_groups");
    if (json_value_get_type(val) == JSONNumber) {
        radio_group_count = (int)json_value_get_number(val);
        MSG_INFO("%d radio groups given\n", radio_group_count);
    } else {
        radio_group_count = DEFAULT_GROUP_COUNT;
        MSG_INFO("Utilising default radio group count %d\n", radio_group_count);
    }

    
    /* Allocate and initialise memory for the radio information structs and statistics */
    rfconf = (struct lgw_conf_rxrf_s**)calloc(radio_group_count, sizeof(struct lgw_conf_rxrf_s*));
    for (i = 0; i < LGW_RF_CHAIN_NB; i++) {
        rfconf[i] = (struct lgw_conf_rxrf_s*)calloc(LGW_RF_CHAIN_NB, sizeof(struct lgw_conf_rxrf_s));
    }
    
    /* set configuration for RF chains */
    number = 0;
    for (i = 0; i < radio_group_count; i++) {
        for (j = 0; j < LGW_RF_CHAIN_NB; j++) {
            snprintf(param_name, sizeof param_name, "radio_%d_%d", i, j); /* compose parameter path inside JSON structure */
            val = json_object_get_value(conf_obj, param_name); /* fetch value (if possible) */
            if (json_value_get_type(val) != JSONObject) {
                MSG_INFO("no configuration for group %d radio %d\n", i, j);
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
            if (rfconf[i][j].enable == false) { /* radio disabsled, nothing else to parse */
                MSG_INFO("Group %d radio %i disabled\n", i, j);
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
                    MSG_WARN("invalid radio type: %s (should be SX1255 or SX1257 or SX1250)\n", str);
                }
                snprintf(param_name, sizeof param_name, "radio_%d_%d.single_input_mode", i, j);
                val = json_object_dotget_value(conf_obj, param_name);
                if (json_value_get_type(val) == JSONBoolean) {
                    rfconf[i][j].single_input_mode = (bool)json_value_get_boolean(val);
                } else {
                    rfconf[i][j].single_input_mode = false;
                }

                MSG_INFO("Group %d radio %d enabled (type %s), center frequency %u, RSSI offset %f\n", i, j, str, rfconf[i][j].freq_hz, rfconf[i][j].rssi_offset);
            }
        }
    }

    /* initialise the specific radio group */
    if (number == LGW_RF_CHAIN_NB * radio_group_count) {
        MSG_ERR("No valid radio configurations given\n");
        return -1;
    } else {
        MSG_INFO("%d radios configured\n", number);
    }

    if (init_radio_group(radio_group_current)) {
        MSG_ERR("Failed to initialise radio group %d\n", i);
        return -1;
    }

    /* set configuration for demodulators */
    memset(&demodconf, 0, sizeof demodconf); /* initialize configuration structure */
    val = json_object_get_value(conf_obj, "chan_multiSF_All"); /* fetch value (if possible) */
    if (json_value_get_type(val) != JSONObject) {
        MSG_INFO("no configuration for LoRa multi-SF spreading factors enabling\n");
    } else {
        conf_demod_array = json_object_dotget_array(conf_obj, "chan_multiSF_All.spreading_factor_enable");
        if ((conf_demod_array != NULL) && ((size = json_array_get_count(conf_demod_array)) <= LGW_MULTI_NB)) {
            for (i = 0; i < (int)size; i++) {
                number = json_array_get_number(conf_demod_array, i);
                if (number < 5 || number > 12) {
                    MSG_WARN("failed to parse chan_multiSF_All.spreading_factor_enable (wrong value at idx %d)\n", i);
                    demodconf.multisf_datarate = 0xFF; /* enable all SFs */
                    break;
                } else {
                    /* set corresponding bit in the bitmask SF5 is LSB -> SF12 is MSB */
                    demodconf.multisf_datarate |= (1 << (number - 5));
                }
            }
        } else {
            MSG_WARN("failed to parse chan_multiSF_All.spreading_factor_enable\n");
            demodconf.multisf_datarate = 0xFF; /* enable all SFs */
        }
        /* all parameters parsed, submitting configuration to the HAL */
        if (lgw_demod_setconf(&demodconf) != LGW_HAL_SUCCESS) {
            MSG_ERR("invalid configuration for demodulation parameters\n");
            return -1;
        }
    }

    /* set configuration for Lora multi-SF channels (bandwidth cannot be set) */
    for (i = 0; i < LGW_MULTI_NB; ++i) {
        memset(&ifconf, 0, sizeof ifconf); /* initialize configuration structure */
        snprintf(param_name, sizeof param_name, "chan_multiSF_%i", i); /* compose parameter path inside JSON structure */
        val = json_object_get_value(conf_obj, param_name); /* fetch value (if possible) */
        if (json_value_get_type(val) != JSONObject) {
            MSG_INFO("no configuration for Lora multi-SF channel %i\n", i);
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
            MSG_INFO("Lora multi-SF channel %i disabled\n", i);
        } else  { /* Lora multi-SF channel enabled, will parse the other parameters */
            snprintf(param_name, sizeof param_name, "chan_multiSF_%i.radio", i);
            ifconf.rf_chain = (uint32_t)json_object_dotget_number(conf_obj, param_name);
            snprintf(param_name, sizeof param_name, "chan_multiSF_%i.if", i);
            ifconf.freq_hz = (int32_t)json_object_dotget_number(conf_obj, param_name);
            if_info[i].radio = ifconf.rf_chain;
            if_info[i].freq_if = ifconf.freq_hz;
            // TODO: handle individual SF enabling and disabling (spread_factor)
            MSG_INFO("Lora multi-SF channel %i>  radio %i, IF %i Hz, 125 kHz bw, SF 5 to 12\n", i, ifconf.rf_chain, ifconf.freq_hz);
        }
        /* all parameters parsed, submitting configuration to the HAL */
        if (lgw_rxif_setconf(i, &ifconf) != LGW_HAL_SUCCESS) {
            MSG_ERR("invalid configuration for Lora multi-SF channel %i\n", i);
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
        MSG_ERR("%s is not a valid JSON file\n", conf_file);
        exit(EXIT_FAILURE);
    }

    /* point to the gateway configuration object */
    conf_obj = json_object_get_object(json_value_get_object(root_val), conf_obj_name);
    if (conf_obj == NULL) {
        MSG_INFO("%s does not contain a JSON object named %s\n", conf_file, conf_obj_name);
        return -1;
    } else {
        MSG_INFO("%s does contain a JSON object named %s, parsing gateway parameters\n", conf_file, conf_obj_name);
    }

    /* gateway unique identifier (aka MAC address) (optional) */
    str = json_object_get_string(conf_obj, "gateway_ID");
    if (str != NULL) {
        sscanf(str, "%llx", &ull);
        lgwm = ull;
        MSG_INFO("gateway MAC address is configured to %016llX\n", ull);
    }

    /* get interval (in seconds) for uploading reports (optional) */
    val = json_object_get_value(conf_obj, "report_interval");
    if (val != NULL) {
        report_interval = (unsigned)json_value_get_number(val);
        MSG_INFO("report uploading interval is configured to %u seconds\n", report_interval);
    }

    /* get interval (in seconds) for changing log files (optional) */
    val = json_object_get_value(conf_obj, "log_interval");
    if (val != NULL) {
        log_interval = (unsigned)json_value_get_number(val);
        MSG_INFO("log creation is every %u seconds\n", log_interval);
    }

    /* get interval (in seconds) for changing log files (optional) */
    val = json_object_get_value(conf_obj, "stats_per_log");
    if (val != NULL) {
        stats_per_log = (unsigned)json_value_get_number(val);
        MSG_INFO("%u statistic generations per log file\n", stats_per_log);
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
        MSG_ERR("%s is not a valid JSON file\n", conf_file);
        exit(EXIT_FAILURE);
    }

    /* point to the gateway configuration object */
    conf_obj = json_object_get_object(json_value_get_object(root_val), conf_obj_name);
    if (conf_obj == NULL) {
        MSG_INFO("%s does not contain a JSON object named %s\n", conf_file, conf_obj_name);
        json_value_free(root_val);
        return -1;
    } else {
        MSG_INFO("%s does contain a JSON object named %s, parsing debug parameters\n", conf_file, conf_obj_name);
    }

    /* Get reference payload configuration */
    conf_array = json_object_get_array (conf_obj, "ref_payload");
    if (conf_array != NULL) {
        debugconf.nb_ref_payload = json_array_get_count(conf_array);
        MSG_INFO("got %u debug reference payload\n", debugconf.nb_ref_payload);

        for (i = 0; i < (int)debugconf.nb_ref_payload; i++) {
            conf_obj_array = json_array_get_object(conf_array, i);
            /* id */
            str = json_object_get_string(conf_obj_array, "id");
            if (str != NULL) {
                sscanf(str, "0x%08X", &(debugconf.ref_payload[i].id));
                MSG_INFO("reference payload ID %d is 0x%08X\n", i, debugconf.ref_payload[i].id);
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
        MSG_INFO("setting debug log file name to %s\n", debugconf.log_file_name);
    }

    /* Commit configuration */
    if (lgw_debug_setconf(&debugconf) != LGW_HAL_SUCCESS) {
        MSG_ERR("Failed to configure debug\n");
        json_value_free(root_val);
        return -1;
    }

    /* free JSON parsing data structure */
    json_value_free(root_val);
    return 0;
}

static int parse_upload_configuration(const char * conf_file) {

    const char conf_obj_name[] = "upload_conf";
    JSON_Object *conf_obj = NULL;
    JSON_Value *root_val = NULL;
    const char *str; /* pointer to sub-strings in the JSON data */

    root_val = json_parse_file_with_comments(conf_file);
    if (root_val == NULL) {
        MSG_ERR("%s is not a valid JSON file\n", conf_file);
        exit(EXIT_FAILURE);
    }

    /* point to the gateway configuration object */
    conf_obj = json_object_get_object(json_value_get_object(root_val), conf_obj_name);
    if (conf_obj == NULL) {
        MSG_INFO("%s does not contain a JSON object named %s\n", conf_file, conf_obj_name);
        json_value_free(root_val);
        return -1;
    } else {
        MSG_INFO("%s does contain a JSON object named %s, parsing debug parameters\n", conf_file, conf_obj_name);
    }

    /* Get auth0 log client key file */
    str = json_object_get_string(conf_obj, "client_key");
    if (str != NULL) {
        strncpy(file_client_key, str, sizeof file_client_key);
        file_client_key[sizeof file_client_key - 1] = '\0'; /* ensure string termination */
        MSG_INFO("auth0 client key json file is %s\n", file_client_key);
    }

    /* Get auth0 domain url */
    str = json_object_get_string(conf_obj, "client_domain");
    if (str != NULL) {
        strncpy(url_auth0, str, sizeof url_auth0);
        url_auth0[sizeof url_auth0 - 1] = '\0'; /* ensure string termination */
        MSG_INFO("auth0 url is %s\n", url_auth0);
    }

    /* Get dashboard endpoint URL */
    str = json_object_get_string(conf_obj, "dashboard_url");
    if (str != NULL) {
        strncpy(url_dash, str, sizeof url_dash);
        url_dash[sizeof url_dash - 1] = '\0'; /* ensure string termination */
        MSG_INFO("dashboard endpoint url is %s\n", url_dash);
    }
    
    json_value_free(root_val);
    return 0;
}

/**
 * Open new log file. This log file is written to by all "MSG_" logging functions,
 * regardless of verbose status.
*/
static void log_open (void) {

    struct timespec now_utc_time;
    char iso_date[40];
    
    clock_gettime(CLOCK_REALTIME, &now_utc_time);
    strftime(iso_date, ARRAY_SIZE(iso_date),"%Y%m%dT%H%M%SZ", gmtime(&now_utc_time.tv_sec)); /* format yyyymmddThhmmssZ */

    sprintf(log_file_name, "sniffer_log_%s.txt", iso_date);
    log_file = fopen(log_file_name, "a"); /* create log file to check if its possible */
    if (log_file == NULL) {
        printf("impossible to create log file %s\n", log_file_name);
        sniffer_exit();
    }

    fclose(log_file);

    MSG_INFO("Now writing to log file %s\n", log_file_name);
    return;
}

/**
 * Function used to specifically read the output given by a curl string run within the 
 * "system" function.
 * 
 * Curl error codes bizzarely shifted 8 places left by system output. Need to move
 * and then parse.
 * 
 * Returns the curl output if it succeeds or can be dealt with.
 * 
 * @param system_output The system code to analyse
 * @return              The curl return code that can be handled, either CURL_ERR_SUCCESS or CURL_ERR_TIMEOUT
*/
static int curl_read_system (int system_output) {

    int status;
    int curl_output = system_output >> CURL_SYS_SHIFT; // Get only 8 bits

    /* Check if the curl error code was something weird we can't handle */
    if (curl_output != CURL_ERR_SUCCESS && curl_output != CURL_ERR_TIMEOUT) {
        MSG_WARN("[uploader] Encountered curl error that cannot be dealth with\n");
        MSG_WARN("[uploader] Curl code %d (system return code %d)\n", curl_output, system_output);

        curl_failures++;

        if (curl_failures == CURL_ERRORS_MIN) {
            MSG_INFO("[uploader] Minimum curl failures hit. Closing and reopening VPN tun0\n");

            status = system("sudo ifconfig tun0 down");

            if (status) {
                MSG_ERR("[uploader] Failed to close ifconfig tun0\n");
                MSG_ERR("[uploader] Errno was %d\n", errno);
                sniffer_exit();
            }

            status = system("sudo ifconfig tun0 up");

            if (status) {
                MSG_ERR("[uploader] Failed to reopen ifconfig tun0\n");
                MSG_ERR("[uploader] Errno was %d\n", errno);
                sniffer_exit();
        }
        }

        /* Check if the curl has failed too many times sequentially */
        if (curl_failures > CURL_ERRORS_MAX) {
            MSG_ERR("[uploader] Reached maximum number of permitted curl failures (CURL_ERRORS_MAX). Exiting\n");
            sniffer_exit();
        }

        return CURL_ERR_CANTHANDLE;
    }

    curl_failures = 0;

    return curl_output;
}

/**
 * Checks for curl timeout, and now manages a failed counter. If the fail counter hits the minimum
 * value (CURL_TIMEOUT_MIN), the VPN tun0 will be closed and reopened. Should the fail counter exceed
 * the maximum value (CURL_TIMEOUT_MAX), the program will exit.
 * 
 * @param url_to_check  The url which the curl failed to reach
 * @return              -1 if the curls fail, 1 if a connection was restablished
*/
static int curl_handle_timeout (char* url_to_check) {

    int status, i;
    char curl_string[120];

    MSG_INFO("[uploader] Curl timeout occured after 15 seconds. Retrying connection to %s\n", url_to_check);

    /* First check the number of curl failures passes a certain threshold */
    if (failed_curls == CURL_TIMEOUT_MIN) {

        MSG_INFO("[uploader] Minimum curl failures hit. Closing and reopening VPN tun0\n");

        status = system("sudo ifconfig tun0 down");

        if (status) {
            MSG_ERR("[uploader] Failed to close ifconfig tun0\n");
            MSG_ERR("[uploader] Errno was %d\n", errno);
            sniffer_exit();
        }

        status = system("sudo ifconfig tun0 up");

        if (status) {
            MSG_ERR("[uploader] Failed to reopen ifconfig tun0\n");
            MSG_ERR("[uploader] Errno was %d\n", errno);
            sniffer_exit();
        }
    } else if (failed_curls > CURL_TIMEOUT_MAX) {
        MSG_ERR("[uploader] Max number of curl reattempts failed. Exiting\n");
        sniffer_exit();
    }

    /* Create our curl string to test */
    sprintf(curl_string, "%s %s", CURL_TEST, url_to_check);

    for (i = 0; i < 4; i++) {
        MSG_INFO("[uploader] Curl reestablish attempt %d\n", i);

        status = system((const char*)curl_string); // Run curl
        status = curl_read_system(status); // Parse system curl output

        /* Check to see if the curl successfully returns */
        if (status == CURL_ERR_SUCCESS) {
            MSG_INFO("[uploader] Curl connection reestablished\n");
            failed_curls = 0;
            return 1;
        }   
    }

    failed_curls++;

    MSG_WARN("[uploader] Failed to reestablish curl connection\n");
    MSG_WARN("[uploader] Failed curls now at %d\n", failed_curls);

    return -1;
}

/**
 * Check curl output of file upload.
 * 
 * If the file is empty, the curl was successful; if not investigate.
 * 
 * If response recieves unauthorized error, acquire new key, else exit.
 * 
 * @param curl_target   The specific curl type we need to parse based on. 
 * 
 * @return              -1 on failure, 0 on success, 1 for re-established connection during auth0
*/
static int curl_handle_output (int curl_target) {

    int i;
    FILE* fp_out;
    JSON_Value *root_val = NULL;
    
    fp_out = fopen(CURL_OUTPUT, "r");
    i = fgetc(fp_out);
    fclose(fp_out);

    /* Check if file is not empty */
    if (i != EOF) {
        /* Lets parse try to pass as JSON */
        root_val = json_parse_file(CURL_OUTPUT);

        if (root_val != NULL) {
            i = curl_parse(root_val, curl_target);
            if (i) return i;
        } else {
            /* Not JSON file found, lets save it */
            save_unknown_response(CURL_OUTPUT);
            save_unknown_response(report_string);
            //return 0;
        }
    }

    return 0;
}

/**
 * Specifically parse the curl output return based on the desired target.
 * Currently either CURL_TARGET_DASH or CURL_TARGET_AUTH0
 * 
 * @param root_val      The root value of the ideally JSON output file
 * @param curl_target   The curl type we are handling for
 * @return              -1 on failure, 0 on success, 1 in the case auth0 curl request re-establishes a connection
*/
static int curl_parse (JSON_Value *root_val, int curl_target) {

    int i;
    const char* str;

    if (curl_target == CURL_TARGET_DASH) {
        /* We are looking specifically for the message field */
        str = json_object_get_string(json_value_get_object(root_val), "message");

        if (str == NULL || strncmp(str, "Unauthorized", 12)) {
            /* Unknown JSON response from dash, lets save it */
            save_unknown_response(CURL_OUTPUT);
            save_unknown_response(report_string);
            //return -1;
        } else {
            MSG_INFO("[curl_parse] Received response {\"message\":\"Unauthorized\"}. Acquiring new key.\n");
            i = curl_get_auth0();
            if (i) return i; // Returns a special variable based on curl_get_auth0 function
        }

    } else if (curl_target == CURL_TARGET_AUTH0) {
        /* First check for errors */
        str = json_object_get_string(json_value_get_object(root_val), "error");

        if (str == NULL) {
            /* Check for NULL here or the strncmp breaks */    
        } else if (!strncmp(str, "access_denied", 13)) {
            /* Unknown JSON response from auth0, lets save it */
            MSG_ERR("[curl_parse] Auth0 Access denied. Check selected client secret json file.\n");
            sniffer_exit();
        }

        str = json_object_get_string(json_value_get_object(root_val), "access_token");

        if (str != NULL) {
            /* we have found a bearer key, lets save it!!!!! */
            memset(auth_key, 0, sizeof auth_key);
            strcpy(auth_key, str);
            MSG_INFO("[curl_parse] New AUTH key acquired\n");
        } else {
            /* Not JSON file found, lets save it */
            save_unknown_response(CURL_OUTPUT);
            save_unknown_response(report_string);
            //return -1;
        }
    }

    return 0;
}

/**
 * Complete auth0 request to acquirer bearer key for dashboard HTTP POST.
 * 
 * @return  -1 on failure, 1 on re-established connection AND getting new AUTH
*/
static int curl_get_auth0 (void) {

    int status;                 /* return variable */
    char curl_string[1500];     /* holds the full curl string */
    
    sprintf(curl_string, "%s -d @%s %s", CURL_PREFIX, file_client_key, url_auth0);

    status = system((const char*)curl_string);
    status = curl_read_system(status); // Parse system output

    if (status == CURL_ERR_SUCCESS) {
        /* Successful curl, lets see the output */
        curl_handle_output(CURL_TARGET_AUTH0);
    } else if (status == CURL_ERR_TIMEOUT) {
        /* Timeout status has returned, lets handle it and explore what went wrong */
        if (curl_handle_timeout(url_auth0)) {
            return -1;
        }
    } else {
        /* Unknown error, we are leaving */
        MSG_ERR("[uploader] During auth0 client request, System failed to run with exit code %d\n", status);
        sniffer_exit();
    }

    return 1;
}

/**
 * Curl POST upload end device and channel reports to dashboard.
 * 
 * Checks for a curl timeout and returns appropriately.
 * 
 * @param upload_file   File to upload.
 * 
 * @return              -1 on a curl failure, 0 on success, 1 if a curl connection is reestablished
*/
static int curl_upload_file (const char * upload_file) {

    int status;                 /* return variable */
    char curl_string[1500];     /* holds the full curl string */
    
    sprintf(curl_string, "%s -H \"Authorization: Bearer %s\" -d @%s %s", CURL_PREFIX, auth_key, upload_file, url_dash);

    status = system((const char*)curl_string);
    status = curl_read_system(status);

    if (status == CURL_ERR_SUCCESS) {
        /* Successful curl, lets see the output */

        //TODO: Need to investigate here next
        // Work on haing it so that we ignore atleast a few weird messages

        status = curl_handle_output(CURL_TARGET_DASH);

        return status;
        
        /* Lets handle any known statuses here */
    } else if (status == CURL_ERR_TIMEOUT) {
        /* Timeout status has returned, lets handle it and explore what went wrong */
        MSG_WARN("[curl_upload_file] Curl timeout detected. Handling\n");
        status = curl_handle_timeout(url_dash);
        return status;
    } else {
        /* Unknown curl error, we just skipping */
        return -1;
    }
    
    return 0;
}

/**
 * Special function for saving unknown curl responses.
 * 
 * @param file_in   String of the file to copy.
 * @return          -1 on failure, 0 on success
*/
static int save_unknown_response (const char* file_in) {

    ssize_t i, j;
    FILE* fp_in, *fp_out;
    char bad_file[100];

    sprintf(bad_file, "bad_file_%d.txt", bad_file_count++);

    MSG_WARN("[save_unknown_response] NON-JSON response received, attempting to save as %s\n", bad_file);

    fp_in = fopen(file_in, "r");
    fp_out = fopen(bad_file, "w");

    if (fp_in == NULL) {
        MSG_WARN("[save_unknown_response] Failed to open %s for reading. Errno was %d. Skipping copy\n", file_in, errno);
        return -1;
    } else if (fp_out == NULL) {
        MSG_WARN("[save_unknown_response] Failed to open %s for writing\n. Errno was %d. Skipping copy\n", bad_file, errno);
        return -1;
    }

    i = fgetc(fp_in);

    do {
        j = fputc(i, fp_out);

        if (j == EOF) {
            MSG_WARN("[save_unknown_response] Failed writing to %s\n. Errno was %d. Exiting copy\n", bad_file, errno);
            return -1;
        }

        i = fgetc(fp_in);
    } while (i != EOF);

    i =  fclose(fp_in);
    if (i == EOF)
        MSG_WARN("[save_unknown_response] Failed closing %s\n. Errno was %d\n", file_in, errno);
            
    i = fclose(fp_out);
    if (i == EOF)
        MSG_WARN("[save_unknown_response] Failed closing %s\n. Errno was %d\n", bad_file, errno);

    return 0;
}

/* -------------------------------------------------------------------------- */
/* --- THREAD 1.0: RECEIVING PACKETS ------------------------------------------ */
void thread_listen(void) {

    int i; /* loop and temporary variables */

    struct timespec sleep_time = {0, 3000000}; /* 3 ms */

    /* allocate memory for packet fetching and processing */
    struct lgw_pkt_rx_s rxpkt[16]; /* array containing up to 16 inbound packets metadata */
    int nb_pkt;

    /* struct for placing data into encoding queue */
    struct entry *pkt_encode;

    while (!exit_sig && !quit_sig) {

        /* fetch packets */
        pthread_mutex_lock(&mx_concent);
        nb_pkt = lgw_receive(ARRAY_SIZE(rxpkt), rxpkt);
        pthread_mutex_unlock(&mx_concent);
        
        if (nb_pkt == LGW_HAL_ERROR) {
            MSG_ERR("[listener] failed packet fetch, exiting\n");
            sniffer_exit();
        } else if (nb_pkt == 0) {
            clock_nanosleep(CLOCK_MONOTONIC, 0, &sleep_time, NULL); /* wait a short time if no packets */
            continue; // reestart loop and check again
        } else {
            pthread_mutex_lock(&mx_report_dev);
            for (i = 0; i < nb_pkt; ++i) {
                pkt_encode = (struct entry*)malloc(sizeof(struct entry));
                pkt_encode->rx_pkt = rxpkt[i];
                STAILQ_INSERT_TAIL(&head, pkt_encode, entries);
            }
            packets_caught += nb_pkt;
            pthread_mutex_unlock(&mx_report_dev);
        }
    }

    MSG_INFO("[listener] Packets caught: %lu\n", (unsigned long)packets_caught);
    MSG_INFO("[listener] End of listening thread\n");
}

/* -------------------------------------------------------------------------- */
/* --- THREAD 1.1: JSON encoding for device packet info --------------------- */
void thread_encode(void) {

    /* return holder variable */
    int i = 0;

    /* mutex interaction variables and counters */
    int mutex_current = 0;
    int ed_reports = 0;

    /* sleep managent value */
    struct timespec sleep_time = {0, 3000000}; /* 0 s, 3ms */

    /* Structs for traversing STAILQ*/
    struct entry *pkt_encode, *pkt_next;

    /* object for data encoding */
    ed_report_t *report = create_ed_report();

    /* timestamp variables */
    struct timespec pkt_utc_time;
    struct tm *xt;

    while (!exit_sig && !quit_sig) {

        pthread_mutex_lock(&mx_report_dev);

        mutex_current = 0; /* reset this, it will change below if necessary */
        i = pthread_mutex_trylock(&mx_ed_report_0);
        if (i != 0) {
            /* unable to get lock from above, lets get the other one */
            pthread_mutex_lock(&mx_ed_report_1);
            mutex_current = 1;
        }
        ed_reports = mutex_current ? ed_reports_1 : ed_reports_0;

        pkt_encode = STAILQ_FIRST(&head);

        while (pkt_encode != NULL) {

            /* Clear data in report object*/
            reset_ed_report(report);

            /* Acquire timestamp data */
            clock_gettime(CLOCK_REALTIME, &pkt_utc_time);
            xt = gmtime(&(pkt_utc_time.tv_sec));

            /* Write to report and encode t device json */
            write_ed_report(report, &pkt_encode->rx_pkt, xt, &pkt_utc_time);
            encode_ed_report(report, mutex_current, ed_reports++);

            if (mutex_current == 0) {
                ed_reports_0++;
            } else {
                ed_reports_1++;
            }

            /* traverse STAILQ and cleanup old queue entry */
            pkt_next = STAILQ_NEXT(pkt_encode, entries);
            STAILQ_REMOVE(&head, pkt_encode, entry, entries);
            free((void*)pkt_encode);
            pkt_encode = pkt_next;
        }

        if (mutex_current == 0) {
            pthread_mutex_unlock(&mx_ed_report_0);
        } else {
            pthread_mutex_unlock(&mx_ed_report_1);
        }

        pthread_mutex_unlock(&mx_report_dev);

        clock_nanosleep(CLOCK_MONOTONIC, 0, &sleep_time, NULL); /* wait a short time if no packets */
    }

    destroy_ed_report(report);
    MSG_INFO("End of encoding thread\n");
}

/* -------------------------------------------------------------------------- */
/* --- THREAD 1.11: Channel aggregate encoding and uploading JSONs ---------- */
void thread_upload(void) {
    
    time_t start, current;              /* Time management variables to ensure thread activates at the correct time*/
    char report_string[100];            /* File for holding file name creation */
    int success;                        /* Dummy return variables */
    int uploads = 0;                    /* Handler variable for temporary connection outages */
    int ed_reports = 0;                 /* Internal ed reports counter */
    int mutex_current = -1;             /* current mutex index handler */

    start = time(NULL);

    while (!exit_sig && !quit_sig) {

        wait_ms(MS_CONV * UPLOAD_SLEEP);
        current = time(NULL);

        /* check if upload interval time has elapsed */
        if (difftime(current, start) > report_interval) {
            MSG_INFO("[thread_upload] Upload timer expired. Beginning upload...\n");
            /* Acquire channel and log locks */
            pthread_mutex_lock(&mx_log);

            /* Need to acquire each uploading mutex and then upload */

            for (int z = 0; z < 2; z++) {
                /* MUTEX HANDLING */
                if (mutex_current == -1) {
                    /* Mutex is not set, choose whichever we can grab first */
                    success = pthread_mutex_trylock(&mx_ed_report_0);
                    if (success != 0) {
                        /* Couldn't get mutex 0, lets go mutex 1 */
                        success = pthread_mutex_lock(&mx_ed_report_1);
                        if (success) {
                            MSG_WARN("[thread_upload] Failed to acquire lock 1 with error after failing to get lock 0 %d?\n", success);
                            break;
                        } 
                        mutex_current = 1;
                    } else {
                        /* We got mutex 0, lets set out variables*/
                        mutex_current = 0;
                    }
                } else if (mutex_current == 0) {
                    /* Time to get mutex 0 */
                    success = pthread_mutex_lock(&mx_ed_report_0);
                    if (success) {
                        MSG_WARN("[thread_upload] Failed to acquire lock 0 with error %d?\n", success);
                        break;
                    } 
                } else if (mutex_current == 1) {
                    /* Time to get mutex 1 */
                    success = pthread_mutex_lock(&mx_ed_report_1);
                    if (success) {
                        MSG_WARN("[thread_upload] Failed to acquire lock 1 with error %d?\n", success);
                        break;
                    } 
                } else {
                    MSG_ERR("[thread_upload] Mutex_current incorrectly set... exiting.\n");
                    sniffer_exit();
                }

                ed_reports = mutex_current ? ed_reports_1 : ed_reports_0;
                uploads = mutex_current ? ed_uploads_1 : ed_uploads_0;

                MSG_INFO("[thread_upload] Utilising mutex %d\n", mutex_current);

                MSG_INFO("[thread_upload] Expecting %d reports uploads\n", ed_reports - uploads);
            
                /* Handle all ED reports generated */
                for (int i = uploads; i < ed_reports; i++) {

                    //MSG_INFO("[debug] Uploading for mutex %d at index %d with uploads %d\n", mutex_current, i, uploads);

                    create_file_string(report_string, JSON_REPORT_ED, mutex_current, i);

                    // MSG_INFO("[debug] Generated file string was %s\n", report_string);

                    success = curl_upload_file(report_string);

                    /* Check if there was a special curl return, either a timeout or a connection reestablish*/
                    if (success == -1) {
                        MSG_WARN("[thread_upload] Curl timeout occured. Waiting for next period.\n");
                        break;
                    } else if (success == 1) {
                        MSG_WARN("[thread_upload] Curl timeout fixed or new auth acquired. Repeating upload attempt.\n");
                        i--;
                        continue;
                    }

                    /* Remove the file to save space */
                    success = remove(report_string);

                    if (success) {
                        MSG_ERR("[thread_upload] Failed to remove file %s\n", report_string);
                    }

                    /* Increment our upload counter */
                    uploads++;
                }

                /* Log data to file */
                MSG_INFO("[thread_upload] Reports encoded: %d, uploaded: %d\n", ed_reports, uploads);

                /* reset upload count if all end device reports were uploaded */
                if (uploads == ed_reports) {

                    /* all data uploaded successfully, lets reset our variables */
                    uploads = 0;

                    if (!continuous) {
                        ed_reports_total += ed_reports;
                        ed_reports = 0;
                    }
                }

                if (mutex_current == 0) {
                    ed_uploads_0 = uploads;
                    ed_reports_0 = ed_reports;
                    pthread_mutex_unlock(&mx_ed_report_0);
                } else {
                    ed_uploads_1 = uploads;
                    ed_reports_1 = ed_reports;
                    pthread_mutex_unlock(&mx_ed_report_1);
                }

                mutex_current = mutex_current ? 0 : 1; /* If mutex is currently 0, go to 1, and vice versa */
            }

            mutex_current = -1; /* Set mutex current variable to its not set state */
            
            pthread_mutex_unlock(&mx_log);
            start = time(NULL);
        }
    }

    MSG_INFO("[uploader] ED reports uploaded total: %d\n", ed_reports_total);
    MSG_INFO("[uploader] End of uploading thread\n");
}

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main(int argc, char **argv) {

    /* return management variable */
    int i;

    /* configuration file related */
    const char defaut_conf_fname[] = JSON_CONF_DEFAULT;
    const char * conf_fname = defaut_conf_fname; /* pointer to a string we won't touch */

    unsigned sleep_time = 0;
    uint8_t sleep_counter = 0;

    /* deamonise handling variables */
    pid_t pid;
    bool daemonise = false;

    /* threads */
    pthread_t thrid_listen;
    pthread_t thrid_encode;
    pthread_t thrid_upload;

    /* message queue initialisation */
    STAILQ_INIT(&head);

    /* parse command line options */
    while( (i = getopt( argc, argv, OPTION_ARGS )) != -1 )
    {
        switch( i )
        {
        case 'a':
            printf("INFO: Keeping all logs...\n");
            continuous = true;
            break;

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
    log_open();

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
        i = parse_gateway_configuration(conf_fname);
        if (i != 0) {
            MSG_ERR("[main] No \"gateway_conf\" field in the chosen (or default) JSON\n");
            exit(EXIT_FAILURE);
        }
        i = parse_debug_configuration(conf_fname);
        if (i != 0) {
            MSG_INFO("[main] no debug configuration\n");
        }
        i =  parse_upload_configuration(conf_fname);
        if (i != 0) {
            MSG_ERR("[main] No \"upload_conf\" field in the chosen (or default) JSON\n");
            exit(EXIT_FAILURE);
        }
    } else {
        MSG_ERR("[main] failed to find any configuration file named %s\n", conf_fname);
        exit(EXIT_FAILURE);
    }

    /* Set our sleep time */
    sleep_time = log_interval / stats_per_log;

    /* starting the concentrator */
    if (sniffer_start()) {
        MSG_ERR("[main] Failed to start sniffer\n");
        exit(EXIT_FAILURE);
    }

    /* channel and gateway info encoding and uploading thread */
    i = pthread_create(&thrid_upload, NULL, (void * (*)(void *))thread_upload, NULL);
    if (i != 0) {
        MSG_ERR("[main] impossible to create uploading thread\n");
        sniffer_exit();
    }

    /* end device encoding thread */
    i = pthread_create(&thrid_encode, NULL, (void * (*)(void *))thread_encode, NULL);
    if (i != 0) {
        MSG_ERR("[main] impossible to create encoding thread\n");
        sniffer_exit();
    }

    /* main listener for upstream */
    i = pthread_create(&thrid_listen, NULL, (void * (*)(void *))thread_listen, NULL);
    if (i != 0) {
        MSG_ERR("[main] impossible to create listening thread\n");
        sniffer_exit();
    }

    /* configure signal handling */
    sigemptyset(&sigact.sa_mask);
    sigact.sa_flags = 0;
    sigact.sa_handler = sig_handler;
    sigaction(SIGQUIT, &sigact, NULL); /* Ctrl-\ */
    sigaction(SIGINT, &sigact, NULL); /* Ctrl-C */
    sigaction(SIGTERM, &sigact, NULL); /* default "kill" command */

    while (!exit_sig && !quit_sig) {
        /* Sleep, then create new log once time is up */
        wait_ms(MS_CONV * sleep_time);
        pthread_mutex_lock(&mx_log);
        pthread_mutex_lock(&mx_report_dev);

        /* close current log and open a new one only if no interrupt signals have been given */
        if (!exit_sig && !quit_sig) {
            if (sleep_counter == stats_per_log) {
                log_open();
                sleep_counter = 0;
            }
            

            generate_sniffer_stats(); // Get our lovely gateway info going
            sleep_counter++;
        }
        
        pthread_mutex_unlock(&mx_report_dev);
        pthread_mutex_unlock(&mx_log);
    }

    /* Get all of our main concentrator listening threads to close */
    i = pthread_join(thrid_listen, NULL);
    if (i != 0) {
        MSG_ERR("Failed to join LoRa listening upstream thread with %d - %s\n", i, strerror(errno));
    }

    /* Wait for ED encoding thread to end */
    i = pthread_join(thrid_encode, NULL);
    if (i != 0) {
        MSG_ERR("Failed to join ED encoding upstream thread with %d - %s\n", i, strerror(errno));
    }

    /* Wait for uploading thread to end */
    i = pthread_join(thrid_upload, NULL);
    if (i != 0) {
        MSG_ERR("Failed to join uploading upstream thread with %d - %s\n", i, strerror(errno));
    }

    if (exit_sig) {
        /* clean up before leaving */
        sniffer_stop();
        stat_cleanup();
    }

    /* message queue deinitialisation */
    STAILQ_INIT(&head);

    MSG_INFO("Successfully exited packet sniffer program\n");

    return EXIT_SUCCESS;
}

/* --- EOF ------------------------------------------------------------------ */
