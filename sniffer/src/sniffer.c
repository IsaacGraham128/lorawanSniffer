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
#include <unistd.h>     /* getopt access */
#include <stdlib.h>     /* atoi, malloc */
#include <errno.h>      /* error messages */
#include <math.h>       /* round */

#include <pthread.h>
#include <sys/queue.h>  /* STAILQ queue */

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
    time_t ltime = time(NULL);                                                                      \
    char *time_str = ctime(&ltime);                                                                 \
    time_str[strlen(time_str)-1] = '\0';                                                            \
    if (verbose) {                                                                                  \
        fprintf(stdout, format __VA_OPT__(,) __VA_ARGS__);                                          \
    }                                                                                               \
    fprintf(log_file, "%s - " format , time_str __VA_OPT__(,) __VA_ARGS__);                         \
}                                                                                                   \

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#ifndef VERSION_STRING
    #define VERSION_STRING "undefined"
#endif

#define OPTION_ARGS         "vhc:"

#define FILE_CPU_TEMP       "/proc/meminfo"
#define FILE_RAM_INFO       "/sys/class/thermal/thermal_zone0/temp"

#define JSON_CONF_DEFAULT   "conf.json"

#define RAM_INFO_FILE       "/proc/meminfo"

#define JSON_REPORT_SUFFIX  ".json"

#define JSON_FILE_ED        "ed_data_"
#define JSON_FILE_CH        "ch_data_"

#define JSON_REPORT_ED      "device"
#define JSON_REPORT_CH      "channel"
#define JSON_REPORT_GW      "gateway"

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
#define JSON_START          "StartTime"
#define JSON_END            "EndTime"
#define JSON_UTIL           "Util"
#define JSON_DEVSEEN        "DevSeen"
#define JSON_MSGTOTAL       "MsgTotal"
#define JSON_MSGUNIQ        "MsgUnique"
#define JSON_MSGFAIL        "MsgFail"

/* JSON key fields for gateway report information */
#define JSON_TMP_CPU        "temp_cpu"
#define JSON_TMP_CON        "temp_con"
#define JSON_RAM_TOTL       "ram_totl"
#define JSON_RAM_AVAL       "ram_aval"  
// what other fields could I get?
// rssi for the cell chip?
// 
// also need to make a log file

#define JSON_TIME_LEN       80          /* Max length of the timestamp string, including null terminator */
#define JSON_DEVADDR_LEN    9           /* Max length of the device address string, including null terminator */
#define JSON_MTYPE_LEN      4           /* Max length of the message type string, including null terminator */
#define JSON_CRC_LEN        6           /* Max length of the CRC string, including null terminator */

#define GPS_REF_MAX_AGE     30          /* maximum admitted delay in seconds of GPS loss before considering latest GPS sync unusable */
#define XERR_INIT_AVG       16          /* nb of measurements the XTAL correction is averaged on as initial value */
#define XERR_FILT_COEF      256         /* coefficient for low-pass XTAL error tracking */

#define MS_CONV             1000        /* conversion define to go from seconds to milliseconds*/
#define UPLOAD_SLEEP        1           /* sleep time of the upload thread to check if its time to upload */
#define GPS_SLEEP           1           /* sleep time of the GPS thread */
#define DEFAULT_INT_REPORT  900         /* default time interval (seconds) for report uploading */
#define DEFAULT_INT_LOG     1800        /* default time interval (seconds) for log usage */

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

#define INFO_ARRAY_DEFAULT  1          /* Initial space allocation for listening to devices */
#define INFO_ARRAY_SCALER   2          /* Used for realloc increase factor */
/* -------------------------------------------------------------------------- */
/* --- PRIVATE TYPES -------------------------------------------------------- */

/* basic device memory */
typedef struct lora_device_s {
    uint32_t        device_adr;     /* device address */
    uint32_t        fcnt;           /* device fcount */
} lora_device_t;

/* spectral scan */
typedef struct spectral_scan_s {
    bool enable;            /* enable spectral scan thread */
    uint32_t        freq_hz_start;  /* first channel frequency, in Hz */
    uint8_t         nb_chan;        /* number of channels to scan (200kHz between each channel) */
    uint16_t        nb_scan;        /* number of scan points for each frequency scan */
    uint32_t        pace_s;         /* number of seconds between 2 scans in the thread */
} spectral_scan_t;

/* end device report */
typedef struct ed_report_s {
    char* timestamp;
    char* devaddr;
    char* mtype;
    char* crc;
    float freq;
    uint8_t sf;
    uint32_t fcnt;
    float snr;
    float rssi;
    float toa;
    bool adr;
} ed_report_t;

/* channel report */
typedef struct ch_report_s {
    char* timestamp;
    char* start;
    char* end;
    float freq;
    uint8_t sf;
    float utilisation;
    uint32_t msg_total;
    uint32_t msg_unique;
    uint32_t msg_failed; 
} ch_report_t;

/* channel/sf report info structs */
typedef struct ch_info_s {
    float freq;
    uint8_t sf;
    struct timespec start_time;
    float total_airtime;
    lora_device_t* devices;
    lora_device_t* devices_tmp;
    uint32_t list_len;
    uint32_t device_count;
    uint32_t msg_total;
    uint32_t msg_unique;
    uint32_t msg_failed; 
} ch_info_t;

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
static unsigned report_interval = DEFAULT_INT_REPORT; /* time interval (in sec) at which reports are uploaded */
static unsigned log_interval = DEFAULT_INT_LOG; /* time interval (in sec) at which new log files are used */

/* signal handling variables */
struct sigaction sigact; /* SIGQUIT&SIGINT&SIGTERM signal handling */
static int exit_sig = 0; /* 1 -> application terminates cleanly (shut down hardware, close open files, etc) */
static int quit_sig = 0; /* 1 -> application terminates without shutting down the hardware */

/* STAILQ messaging head and initialisation */
static pthread_mutex_t mx_report_dev = PTHREAD_MUTEX_INITIALIZER; /* control access to the device message queue */
static pthread_mutex_t mx_report_ch = PTHREAD_MUTEX_INITIALIZER;  /* control access to the channel aggregate data */
STAILQ_HEAD(stailhead, entry);
static struct stailhead head;

/* configuration variables needed by the application  */
static uint64_t lgwm = 0; /* LoRa gateway MAC address */
// static char lgwm_str[17];

/* clock and log file management */
static pthread_mutex_t mx_log = PTHREAD_MUTEX_INITIALIZER; /* control access to the log file */
static bool verbose = false;
static FILE * log_file = NULL;
static char log_file_name[128];

/* hardware access control and correction */
// TODOD : What happens if this is made static??
pthread_mutex_t mx_concent = PTHREAD_MUTEX_INITIALIZER; /* control access to the concentrator */
static pthread_mutex_t mx_xcorr = PTHREAD_MUTEX_INITIALIZER; /* control access to the XTAL correction */
static bool xtal_correct_ok = false; /* set true when XTAL correction is stable enough */
static double xtal_correct = 1.0;

/* GPS configuration and synchronization */
static char gps_tty_path[64] = "\0"; /* path of the TTY port GPS is connected on */
static int gps_tty_fd = -1; /* file descriptor of the GPS TTY port */
static bool gps_enabled = true; /* is GPS enabled on that gateway ? */

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
static if_info_t if_info[LGW_MULTI_NB];
static bool radio_group_swapping;
static int radio_group_current; /* Current radio group in use */
static int radio_group_count;
static struct lgw_conf_rxrf_s **rfconf; /* Matrix of radio groups [group][radio config] */

/* Spectral Scan */
static spectral_scan_t spectral_scan_params = {
    .enable = false,
    .freq_hz_start = 0,
    .nb_chan = 0,
    .nb_scan = 0,
    .pace_s = 10
};

/* JSON writing management */
ch_info_t ch_report_info[LGW_MULTI_NB][SF_COUNT];
static char report_string[50];
static int ed_reports = 0;
static int ch_reports = 0;

/* JSOn statistics */
static int ed_reports_total = 0;
static int ch_reports_total = 0;



/* -------------------------------------------------------------------------- */
/* --- PRIVATE STATISTIC VARIABLES (GLOBAL) --------------------------------- */

// Need to delete
// uint16_t devices_seen;
// lora_device_t *devices;

// bool realloc_flag;
// uint8_t *confirmed_retransmissions;     /* List of retransmission numbers for confirmed messages */
// uint8_t *unconfirmed_retransmissions;   /* List of retransmission numbers for UNconfirmed messages */
// uint8_t *b_confirmed_retransmissions;   /* Realloc list of retransmission numbers for confirmed messages */
// uint8_t *b_unconfirmed_retransmissions; /* Realloc list of retransmission numbers for UNconfirmed messages */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DECLARATION ---------------------------------------- */

/* General runtime functions */
static void usage(void);

static void sig_handler(int sigio);

/* Gateway statistic gathering functions (ram available and total)*/
float get_ram_value(char* str);

float get_cpu_temp (void);

/* End device report writing object handlers */

static ed_report_t* create_ed_report(void);

static void write_ed_report(ed_report_t* report, struct lgw_pkt_rx_s *p, struct tm *xt, struct timespec *fetch_time);

static void reset_ed_report(ed_report_t* report);

static void destroy_ed_report(ed_report_t* report);

/* Channel report struct objects */

static void create_ch_report(void);

static void write_ch_report(ed_report_t* report, struct lgw_pkt_rx_s* pkt);

static void reset_ch_report(void);

static void destroy_ch_report(void);

/* Report object encoding functions */

static void create_file_string(char* file_type, int index);

static void encode_ed_report(ed_report_t *info, int index);

static void encode_ch_report(ch_report_t *info, int index);

static void create_gw_report(void);

static void create_all_channel_reports(void);

/* Auxilliary help functions*/

static uint8_t find_channel_no(uint32_t freq);

static int start_sniffer(void);

static int stop_sniffer(void);

/* Radio configuration functions */

static int init_radio_group(int group);

static void stat_cleanup(void);

/* Configuration parsing files */

static int parse_SX130x_configuration(const char * conf_file);

static int parse_gateway_configuration(const char * conf_file);

static int parse_debug_configuration(const char * conf_file);

/* Log file interaction */
static void log_open (void);

static void log_close (void);


/* Auxiliary GPS thread functions */

static void gps_process_sync(void);

static void gps_process_coords(void);

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
 * Get the ram utilisation values found in /proc/meminfo by passing in the current
 * file reading line. Should be used in conjunction with getline().
 * 
 * @param str   Pointer to the current file line
 * @return      The utilisation value in MiB
*/
float get_ram_value(char* str) {

    char* num;
    float value = 0;

    num = str;

    while (*num) {
        if (isdigit(*num)) {
            value = strtol(num, &num, 10);
            break;
        }
        num++;
    }

    return value;
}

/**
 * Get the current CPU temperature.
 * 
 * @return  The current CPU temperature in degrees.
*/
float get_cpu_temp (void) {

    FILE* stream;
    char *line, *num;
    size_t len;
    float value = 0;

    stream = fopen(FILE_CPU_TEMP, "r");

    getline(&line, &len, stream);
    num = line;
    value = (float)strtol(num, &num, 10) / 1000.0;

    fclose(stream);

    return value;
}

/**
 * Create the appropriate string for a JSON file. User can define if the 
 * file is for a channel or device report, as well as its number. String is placed into
 * "report_string" variable.
 * 
 * @param file_type Either JSON_REPORT_ED or JSON_REPORT_CH
 * @param index     The number of the file
*/
static void create_file_string(char* file_type, int index) {

    char number[20];
    memset(report_string, 0, 50);
    sprintf(number, "%d", index);
    strcat(report_string, file_type);
    strcat(report_string, number);
    strcat(report_string, JSON_REPORT_SUFFIX);
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

    /* recieved packet variables */
    uint32_t mote_addr = 0;
    uint8_t  mote_mhdr = 0;

    /* Timestamp */
    sprintf(report->timestamp, "%04i-%02i-%02iT%02i:%02i:%02i.%03liZ",(xt->tm_year)+1900,(xt->tm_mon)+1,xt->tm_mday,xt->tm_hour,xt->tm_min,xt->tm_sec,(fetch_time->tv_nsec)/1000000); /* ISO 8601 format */
    /* MHDR and Message Type */
    mote_mhdr = p->payload[0];
    mote_addr = p->payload[1] | p->payload[2] << 8 | p->payload[3] << 16 | p->payload[4] << 24;

    sprintf(report->devaddr, "%x", mote_addr);

    switch(mote_mhdr >> 5) {
        case 0b000 : sprintf(report->mtype, "JR");  break;
        case 0b001 : sprintf(report->mtype, "JA");  break;
        case 0b010 : sprintf(report->mtype, "UDU"); break;
        case 0b011 : sprintf(report->mtype, "UDD"); break;
        case 0b100 : sprintf(report->mtype, "CDU"); break;
        case 0b101 : sprintf(report->mtype, "CDD"); break;
        case 0b110 : sprintf(report->mtype, "RFU"); break;
        case 0b111 : sprintf(report->mtype, "PRP"); break;
    }

    /* CRC status */
    switch(p->status) {
        case STAT_CRC_OK:       sprintf(report->crc, "OK");     break;
        case STAT_CRC_BAD:      sprintf(report->crc, "BAD");    break;
        case STAT_NO_CRC:       sprintf(report->crc, "NONE");   break;
        case STAT_UNDEFINED:    sprintf(report->crc, "UNDEF");  break;
        default:                sprintf(report->crc, "ERR");
    }

    /* General packet statistics - Freq, SF, FCnt, SNR, RSSI, ToA, ADR */
    report->freq = ((double)p->freq_hz / 1e6);
    report->sf = p->datarate;
    report->snr = p->snr;
    report->fcnt = p->payload[6] | p->payload[7] << 8;
    report->rssi = -1 * p->rssis;

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
    report->adr = (p->payload[5] & 0x80) ? true : false;
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
}

/**
 * Destroy the allocated memory associated with a ed_report_t object
 * 
 * @param report    Pointer to the ed_report_t object to free 
*/
static void destroy_ed_report(ed_report_t* report) {

    free((void*)report->timestamp);
    free((void*)report->devaddr);
    free((void*)report->mtype);
    free((void*)report->crc);
    free((void*)report);
}

/**
 * Create a JSON report for a given end device information struct.
 * 
 * @param info  ed_report_t containing all relevant transmission information
 * @param index Number of the report
*/
static void encode_ed_report(ed_report_t *info, int index) {

    JSON_Value* root_value;
    JSON_Object* root_object;
    FILE* file;
    char* serialized_string = NULL;

    create_file_string(JSON_REPORT_ED, index);

    file = fopen(report_string, "w");

    root_value = json_value_init_object();
    root_object = json_value_get_object(root_value);
    json_object_set_string(root_object, JSON_TIME,      info->timestamp);
    json_object_set_string(root_object, JSON_TYPE,      JSON_REPORT_ED);
    json_object_set_string(root_object, JSON_DEVADDR,   info->devaddr);
    json_object_set_string(root_object, JSON_MTYPE,     info->mtype);
    json_object_set_string(root_object, JSON_CRC,       info->crc);
    json_object_set_number(root_object, JSON_FREQ,      info->freq);
    json_object_set_number(root_object, JSON_SF,        info->sf);
    json_object_set_number(root_object, JSON_FCNT,      info->fcnt);
    json_object_set_number(root_object, JSON_SNR,       info->snr);
    json_object_set_number(root_object, JSON_RSSI,      info->rssi);
    json_object_set_number(root_object, JSON_TOA,       info->toa);
    json_object_set_boolean(root_object, JSON_ADR,      info->adr);
    
    serialized_string = json_serialize_to_string(root_value);
    //serialized_string = json_serialize_to_string_pretty(root_value);
    fputs(serialized_string, file);

    json_free_serialized_string(serialized_string);
    json_value_free(root_value);
    fclose(file);
}

/**
 * Create a JSON report for a given channel and spreading factor information struct.
 * 
 * @param info  ch_report_t containing all relevant channel activity information
 * @param index Number of the report
*/
static void encode_ch_report(ch_report_t *info, int index) {

    JSON_Value* root_value;
    JSON_Object* root_object;
    FILE* file;
    char* serialized_string = NULL;

    create_file_string(JSON_REPORT_CH, index);

    file = fopen(report_string, "w");

    root_value = json_value_init_object();
    root_object = json_value_get_object(root_value);
    json_object_set_string(root_object, JSON_TIME,      info->timestamp);
    json_object_set_string(root_object, JSON_TYPE,      JSON_REPORT_CH);
    json_object_set_string(root_object, JSON_START,     info->start);
    json_object_set_string(root_object, JSON_END,       info->end);
    json_object_set_number(root_object, JSON_FREQ,      info->freq);
    json_object_set_number(root_object, JSON_SF,        info->sf);
    json_object_set_number(root_object, JSON_UTIL,      info->utilisation);
    json_object_set_number(root_object, JSON_MSGTOTAL,  info->msg_total);
    json_object_set_number(root_object, JSON_MSGUNIQ,   info->msg_unique);
    json_object_set_number(root_object, JSON_MSGFAIL,   info->msg_failed);

    serialized_string = json_serialize_to_string(root_value);
    //serialized_string = json_serialize_to_string_pretty(root_value);
    fputs(serialized_string, file);

    json_free_serialized_string(serialized_string);
    json_value_free(root_value);
    fclose(file);
}

/**
 * Create a JSON report for the current gateway statistics.
*/
static void create_gw_report (void) {

    struct timespec fetch_time;
    struct tm *xt;
    JSON_Value *root_value;
    JSON_Object *root_object;
    FILE* file;
    size_t len;
    char *serialized_string = NULL;
    char *line;
    char *timestamp = (char*)malloc(sizeof(char) * JSON_TIME_LEN);

    /* Variables for utilisation and statistics */
    int i;
    float temp_cpu, temp_con;
    uint16_t ram_total, ram_available;
    
    temp_cpu = 0;
    //temp_cpu = get_cpu_temp();
    i = lgw_get_temperature(&temp_con);

    if (i == LGW_HAL_ERROR) {
        MSG_ERR("Failed to acquire concentrator temp\n");
        temp_con = 0;
    }

    /* Acquire ram utilisation values */
    file = fopen(RAM_INFO_FILE,"r");
    getline(&line, &len, file);                 /* get the total system ram*/
    ram_total = get_ram_value(line) / 1024;     /* convert to mibi bytes */
    getline(&line, &len, file);                 /* skip this line, we dont care about free */   
    getline(&line, &len, file);                 /* get the available system ram */   
    ram_available = get_ram_value(line) / 1024; /* convert to mibi bytes */
    fclose(file);

    /* Now continue log information as usual */
    clock_gettime(CLOCK_REALTIME, &fetch_time);
    xt = gmtime(&(fetch_time.tv_sec));
    sprintf(timestamp, "%04i-%02i-%02iT%02i:%02i:%02i.%03liZ",(xt->tm_year)+1900,(xt->tm_mon)+1,xt->tm_mday,xt->tm_hour,xt->tm_min,xt->tm_sec,(fetch_time.tv_nsec)/1000000); /* ISO 8601 format */

    create_file_string(JSON_REPORT_GW, 0);

    file = fopen(report_string, "w");

    root_value = json_value_init_object();
    root_object = json_value_get_object(root_value);
    json_object_set_string(root_object, JSON_TIME,      timestamp);
    json_object_set_string(root_object, JSON_TYPE,      JSON_REPORT_GW);
    json_object_set_number(root_object, JSON_TMP_CPU,   temp_cpu);
    json_object_set_number(root_object, JSON_TMP_CON,   temp_con);
    json_object_set_number(root_object, JSON_RAM_TOTL,  ram_total);
    json_object_set_number(root_object, JSON_RAM_AVAL,  ram_available);
    
    serialized_string = json_serialize_to_string(root_value);
    fputs(serialized_string, file);

    json_free_serialized_string(serialized_string);
    json_value_free(root_value);
    fclose(file);

    free((void*)timestamp);
}

/**
 * Create all channel report json files based of the ch_report_info matrix.
*/
static void create_all_channel_reports(void) {

    struct timespec fetch_time;
    struct tm *xt;
    ch_info_t *ch_info;
    ch_report_t *ch_report;
    char *start_time;
    char *end_time;
    float utilisation;
    
    start_time = (char*)malloc(sizeof(char) * JSON_TIME_LEN);
    end_time = (char*)malloc(sizeof(char) * JSON_TIME_LEN);
    
    ch_report = (ch_report_t*)calloc(1, sizeof(ch_report_t));

    clock_gettime(CLOCK_REALTIME, &fetch_time);
    xt = gmtime(&(fetch_time.tv_sec));
    sprintf(end_time, "%04i-%02i-%02iT%02i:%02i:%02i.%03liZ",(xt->tm_year)+1900,(xt->tm_mon)+1,xt->tm_mday,xt->tm_hour,xt->tm_min,xt->tm_sec,(fetch_time.tv_nsec)/1000000); /* ISO 8601 format */

    for (int i = 0; i < LGW_MULTI_NB; i++) {
        for (int j = 0; j < SF_COUNT; j++) {
            if (ch_report_info[i][j].device_count) {
                ch_info = &ch_report_info[i][j];

                xt = gmtime(&(ch_info->start_time.tv_sec));
                sprintf(start_time, "%04i-%02i-%02iT%02i:%02i:%02i.%03liZ",(xt->tm_year)+1900,(xt->tm_mon)+1,xt->tm_mday,xt->tm_hour,xt->tm_min,xt->tm_sec,(fetch_time.tv_nsec)/1000000); /* ISO 8601 format */

                ch_report->timestamp = end_time;
                ch_report->start = start_time;
                ch_report->end = end_time;

                ch_report->freq = ch_info->freq;
                ch_report->sf = ch_info->sf;

                ch_report->msg_total = ch_info->msg_total;
                ch_report->msg_unique = ch_info->msg_unique;
                ch_report->msg_failed = ch_info->msg_failed;

                utilisation = ch_info->total_airtime / (float)(fetch_time.tv_sec - ch_info->start_time.tv_sec);

                ch_report->utilisation = utilisation * 1e2; // Move up to percentile

                encode_ch_report(ch_report, ch_reports++);
            }
        }
    }

    free((void*)start_time);
    free((void*)end_time);
    free((void*)ch_report);
}

/**
 * Create all of the necessary channel report information structs used to store
 * information necessary to generating the channel report info
*/
static void create_ch_report (void) {

    ch_info_t *ch_info;
    uint32_t radio_0_freq, radio_1_freq;
    struct timespec fetch_time;

    clock_gettime(CLOCK_REALTIME, &fetch_time);
    
    radio_0_freq = rfconf[radio_group_current][0].freq_hz;
    radio_1_freq = rfconf[radio_group_current][1].freq_hz;

    for (int i = 0; i < LGW_MULTI_NB; i++) {
        for (int j = 0; j < SF_COUNT; j++) {
            // Get ch report info struct and assign to pointer for nicer looking code
            ch_info = &ch_report_info[i][j];
            
            // Clear the memory and set the variables
            // Could alternatively use a memset?
            ch_info->start_time = fetch_time;
            ch_info->devices = (lora_device_t*)calloc(INFO_ARRAY_DEFAULT, sizeof(lora_device_t));
            ch_info->list_len = INFO_ARRAY_DEFAULT;
            ch_info->freq = if_info[i].radio ? radio_1_freq : radio_0_freq;
            ch_info->freq = (ch_info->freq + if_info[i].freq_if) / 1e6;
            ch_info->sf = (SF_BASE + j);
            ch_info->total_airtime = 0;
            ch_info->device_count = 0;
            ch_info->msg_total = 0;
            ch_info->msg_unique = 0;
            ch_info->msg_failed = 0;
        }
    }
}

/**
 * Update channel report struct given a recieved packet.
 * 
 * @param   report  The encoded packet information
 * @param   p       The raw packet information
*/
static void write_ch_report (ed_report_t* report, struct lgw_pkt_rx_s* p) {

    ch_info_t *ch_info;
    lora_device_t *ch_device;
    bool dev_found = false;
    uint32_t mote_addr;

    for (int i = 0; i < LGW_MULTI_NB; i++) {
        if (report->freq == ch_report_info[i][0].freq) {

            ch_info = &ch_report_info[i][report->sf - SF_BASE];

            /* Get device address */
            mote_addr = p->payload[1] | p->payload[2] << 8 | p->payload[3] << 16 | p->payload[4] << 24;

            /* increment channel airtime */
            ch_info->total_airtime += report->toa / 1e3; // Convert to seconds

            /* find if the device already exists in this ch reports memory */
            for (uint32_t g = 0; g < ch_info->device_count + 1; g++) {
                if (ch_info->devices[g].device_adr == mote_addr) {
                    ch_device = &ch_info->devices[g];
                    dev_found = true;
                    break;
                }
            }

            /* increment our message counters */
            ch_info->msg_total++;
            if (p->status != STAT_CRC_OK) {
                ch_info->msg_failed++;
            }

            /* handle counters and device listing memory */
            if (dev_found) {
                if (report->fcnt != ch_device->fcnt) {
                    ch_device->fcnt = report->fcnt;
                    ch_info->msg_unique++;
                }
                dev_found = false; // set to false for next device analysis
            } else {
                ch_info->devices[ch_info->device_count].device_adr = mote_addr;
                ch_info->devices[ch_info->device_count].fcnt = report->fcnt;
                ch_info->device_count++;

                /* check list size, realloc if we have filled the buffer */
                if (ch_info->device_count == ch_info->list_len) {
                    ch_info->list_len *= INFO_ARRAY_SCALER;
                    ch_info->devices_tmp = (lora_device_t*)realloc(ch_info->devices, ch_info->list_len * sizeof(lora_device_t));
                    if (ch_info->devices_tmp == NULL) {
                        MSG_ERR("Realloc failed\n");
                        exit(EXIT_FAILURE);
                    }
                    ch_info->devices = ch_info->devices_tmp;
                }
            }

            break;
        }
    }
}

/**
 * Assign new timestamp and (if messages were caught) clear associated statistic memory. 
 * Use when entering a new listening period.
*/
static void reset_ch_report (void) {

    ch_info_t *ch_info;
    struct timespec fetch_time;

    clock_gettime(CLOCK_REALTIME, &fetch_time);

    for (int i = 0; i < LGW_MULTI_NB; i++) {
        for (int j = 0; j < SF_COUNT; j++) {

            ch_info = &ch_report_info[i][j];

            /* Set new timestamp */
            ch_info->start_time = fetch_time;

            /* If device count higher than 0, clear all tracking fields and clear array memory */
            if (ch_info->device_count) {
                ch_info->devices = (lora_device_t*)calloc(ch_info->list_len, sizeof(lora_device_t));
                ch_info->total_airtime = 0;
                ch_info->device_count = 0;
                ch_info->msg_total = 0;
                ch_info->msg_unique = 0;
                ch_info->msg_failed = 0;
            }
        }
    }
}

/**
 * Cleanup allocated memory given to each ch_info piece
*/
static void destroy_ch_report (void) {

    for (int i = 0; i < LGW_MULTI_NB; i++) {
        for (int j = 0; j < SF_COUNT; j++) {
            free((void*)ch_report_info[i][j].devices);
        }
    }
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
static int stop_sniffer(void) {

    int i;
    i = lgw_stop();
    if (i == LGW_HAL_SUCCESS) {
        MSG_INFO("concentrator stopped successfully\n");
    } else {
        MSG_WARN("failed to stop concentrator successfully\n");
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

        /* Spectral Scan configuration */
        conf_scan_obj = json_object_get_object(conf_sx1261_obj, "spectral_scan"); /* fetch value (if possible) */
        if (conf_scan_obj == NULL) {
            MSG_INFO("no configuration for Spectral Scan\n");
        } else {
            val = json_object_get_value(conf_scan_obj, "enable"); /* fetch value (if possible) */
            if (json_value_get_type(val) == JSONBoolean) {
                /* Enable background spectral scan thread in packet forwarder */
                spectral_scan_params.enable = (bool)json_value_get_boolean(val);
            } else {
                MSG_WARN("Data type for spectral_scan.enable seems wrong, please check\n");
            }
            if (spectral_scan_params.enable == true) {
                /* Enable the sx1261 radio hardware configuration to allow spectral scan */
                sx1261conf.enable = true;
                MSG_INFO("Spectral Scan with SX1261 is enabled\n");

                /* Get Spectral Scan Parameters */
                val = json_object_get_value(conf_scan_obj, "freq_start"); /* fetch value (if possible) */
                if (json_value_get_type(val) == JSONNumber) {
                    spectral_scan_params.freq_hz_start = (uint32_t)json_value_get_number(val);
                } else {
                    MSG_WARN("Data type for spectral_scan.freq_start seems wrong, please check\n");
                }
                val = json_object_get_value(conf_scan_obj, "nb_chan"); /* fetch value (if possible) */
                if (json_value_get_type(val) == JSONNumber) {
                    spectral_scan_params.nb_chan = (uint8_t)json_value_get_number(val);
                } else {
                    MSG_WARN("Data type for spectral_scan.nb_chan seems wrong, please check\n");
                }
                val = json_object_get_value(conf_scan_obj, "nb_scan"); /* fetch value (if possible) */
                if (json_value_get_type(val) == JSONNumber) {
                    spectral_scan_params.nb_scan = (uint16_t)json_value_get_number(val);
                } else {
                    MSG_WARN("Data type for spectral_scan.nb_scan seems wrong, please check\n");
                }
                val = json_object_get_value(conf_scan_obj, "pace_s"); /* fetch value (if possible) */
                if (json_value_get_type(val) == JSONNumber) {
                    spectral_scan_params.pace_s = (uint32_t)json_value_get_number(val);
                } else {
                    MSG_WARN("Data type for spectral_scan.pace_s seems wrong, please check\n");
                }
            }
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
        MSG_INFO("statistics display interval is configured to %u seconds\n", log_interval);
    }

    /* GPS module TTY path (optional) */
    str = json_object_get_string(conf_obj, "gps_tty_path");
    if (str != NULL) {
        strncpy(gps_tty_path, str, sizeof gps_tty_path);
        gps_tty_path[sizeof gps_tty_path - 1] = '\0'; /* ensure string termination */
        MSG_INFO("GPS serial port path is configured to \"%s\"\n", gps_tty_path);
    }

    /* get reference coordinates */
    val = json_object_get_value(conf_obj, "ref_latitude");
    if (val != NULL) {
        reference_coord.lat = (double)json_value_get_number(val);
        MSG_INFO("Reference latitude is configured to %f deg\n", reference_coord.lat);
    }
    val = json_object_get_value(conf_obj, "ref_longitude");
    if (val != NULL) {
        reference_coord.lon = (double)json_value_get_number(val);
        MSG_INFO("Reference longitude is configured to %f deg\n", reference_coord.lon);
    }
    val = json_object_get_value(conf_obj, "ref_altitude");
    if (val != NULL) {
        reference_coord.alt = (short)json_value_get_number(val);
        MSG_INFO("Reference altitude is configured to %i meters\n", reference_coord.alt);
    }

    /* Gateway GPS coordinates hardcoding (aka. faking) option */
    val = json_object_get_value(conf_obj, "fake_gps");
    if (json_value_get_type(val) == JSONBoolean) {
        gps_fake_enable = (bool)json_value_get_boolean(val);
        if (gps_fake_enable == true) {
            MSG_INFO("fake GPS is enabled\n");
        } else {
            MSG_INFO("fake GPS is disabled\n");
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

static void log_open (void) {

    struct timespec now_utc_time;
    char iso_date[40];
    
    clock_gettime(CLOCK_REALTIME, &now_utc_time);
    strftime(iso_date, ARRAY_SIZE(iso_date),"%Y%m%dT%H%M%SZ", gmtime(&now_utc_time.tv_sec)); /* format yyyymmddThhmmssZ */

    sprintf(log_file_name, "sniffer_log_%s.txt", iso_date);
    log_file = fopen(log_file_name, "a"); /* create log file, append if file already exist */
    if (log_file == NULL) {
        MSG_ERR("impossible to create log file %s\n", log_file_name);
        exit(EXIT_FAILURE);
    }

    // i = fprintf(log_file, "\"gateway ID\",\"node MAC\",\"UTC timestamp\",\"us count\",\"frequency\",\"RF chain\",\"RX chain\",\"status\",\"size\",\"modulation\",\"bandwidth\",\"datarate\",\"coderate\",\"RSSI\",\"SNR\",\"payload\"\n");
    // if (i < 0) {
    //     MSG_ERR("impossible to write to log file %s\n", log_file_name);
    //     exit(EXIT_FAILURE);
    // }

    MSG_INFO("Now writing to log file %s\n", log_file_name);
    return;
}

static void log_close (void) {

    int i;

    i = fclose(log_file);

    if (i < 0) {
        MSG_ERR("Failed to close log file\n");
        exit(EXIT_FAILURE);
    }
}

/* -------------------------------------------------------------------------- */
/* --- THREAD 1: RECEIVING PACKETS ------------------------------------------ */
void thread_listen(void) {

    int i; /* loop and temporary variables */

    struct timespec sleep_time = {0, 3000000}; /* 3 ms */
    // struct timespec sleep_time = {0, 5000000000}; /* 5000 ms */

    /* counting variables */
    unsigned long pkt_in_log = 0;

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
            MSG_ERR("failed packet fetch, exiting\n");
            exit(EXIT_FAILURE);
        } else if (nb_pkt == 0) {
            clock_nanosleep(CLOCK_MONOTONIC, 0, &sleep_time, NULL); /* wait a short time if no packets */
            continue; // restart loop and check again
        } else {
            pthread_mutex_lock(&mx_report_dev);
            for (i = 0; i < nb_pkt; ++i) {
                pkt_encode = (struct entry*)malloc(sizeof(struct entry));
                pkt_encode->rx_pkt = rxpkt[i];
                STAILQ_INSERT_TAIL(&head, pkt_encode, entries);
            }
            pthread_mutex_unlock(&mx_report_dev);
        }
    }

    MSG_INFO("Packets caught: %ld\n", pkt_in_log);
    MSG_INFO("End of listening thread\n");
}

/* -------------------------------------------------------------------------- */
/* --- THREAD 1.1: JSON encoding for device packet info --------------------- */
void thread_encode(void) {

    /* General return variable holder */
    int i = 0;

    /* sleep managent value */
    struct timespec sleep_time = {0, 3000000}; /* 0 s, 3ms */

    /* Structs for traversing STAILQ*/
    struct entry *pkt_encode, *pkt_next;

    /* object for data encoding */
    ed_report_t *report = create_ed_report();

    /* gps handling variables */
    struct tref local_ref; /* time reference used for UTC <-> timestamp conversion */
    bool gps_ok = false;

    /* timestamp variables until GPS is acquired */
    struct timespec pkt_utc_time;
    struct tm *xt;

    while (!exit_sig && !quit_sig) {

        pthread_mutex_lock(&mx_report_dev);
        pthread_mutex_lock(&mx_report_ch);

        if ((STAILQ_EMPTY(&head) == false) && (gps_enabled == true)) {
            pthread_mutex_lock(&mx_timeref);
            local_ref = time_reference_gps;
            gps_ok = gps_ref_valid;
            pthread_mutex_unlock(&mx_timeref);
        } else {
            gps_ok = false;
        }

        pkt_encode = STAILQ_FIRST(&head);

        while (pkt_encode != NULL) {

            MSG_INFO("[encoder] Got a packet time to encode!\n");

            /* Clear data in report object*/
            reset_ed_report(report);

            /* Acquire timestamp data */
            if (gps_ok) {
                i = lgw_cnt2utc(local_ref, pkt_encode->rx_pkt.count_us, &pkt_utc_time);
                if (i != LGW_GPS_SUCCESS) {
                    clock_gettime(CLOCK_REALTIME, &pkt_utc_time);
                }
            } else {
                clock_gettime(CLOCK_REALTIME, &pkt_utc_time);
            }
            xt = gmtime(&(pkt_utc_time.tv_sec));

            /* Write to report and encode t device json */
            write_ed_report(report, &pkt_encode->rx_pkt, xt, &pkt_utc_time);
            encode_ed_report(report, ed_reports++);

            //MSG_INFO("Report number %d encoded for this time frame\n", ed_reports);

            /* Update the appropriate channel aggregate info */
            write_ch_report(report, &pkt_encode->rx_pkt);

            /* traverse STAILQ and cleanup old queue entry */
            pkt_next = STAILQ_NEXT(pkt_encode, entries);
            STAILQ_REMOVE(&head, pkt_encode, entry, entries);
            free((void*)pkt_encode);
            pkt_encode = pkt_next;
        }
        pthread_mutex_unlock(&mx_report_ch);
        pthread_mutex_unlock(&mx_report_dev);

        clock_nanosleep(CLOCK_MONOTONIC, 0, &sleep_time, NULL); /* wait a short time if no packets */
    }

    destroy_ed_report(report);
    MSG_INFO("End of encoding thread\n");
}

/* -------------------------------------------------------------------------- */
/* --- THREAD 1.11: Channel aggregate encoding and uploading JSONs ---------- */
void thread_upload(void) {

    time_t start, current;

    /* Create the channel report to be used */ 
    create_ch_report();

    start = time(NULL);

    while (!exit_sig && !quit_sig) {

        wait_ms(MS_CONV * UPLOAD_SLEEP);
        current = time(NULL);

        /* check if upload interval time has elapsed */
        if (difftime(current, start) > report_interval) {
            MSG_INFO("[upload] creating logs before uploading\n");
            /* Acquire channel and log locks */
            pthread_mutex_lock(&mx_report_ch);
            pthread_mutex_lock(&mx_log);
            
            /* Generate all reports for channels */
            create_all_channel_reports();

            /* Generate gateway statistic report and write to log */
            create_gw_report();


            /* Upload reports */

            /* Delete reports */

            /* Log data to file */
            ed_reports_total += ed_reports;
            ch_reports_total += ch_reports;

            /* Cleanup of any data structures to prep for next upload */
            reset_ch_report();
            ed_reports = 0;
            ch_reports = 0;
            
            pthread_mutex_unlock(&mx_log);
            pthread_mutex_unlock(&mx_report_ch);

            start = time(NULL);
        }
    }

    destroy_ch_report();
    MSG_INFO("End of uploading thread\n");
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
        MSG_WARN("[gps] could not get GPS time from GPS\n");
        return;
    }

    /* get timestamp captured on PPM pulse  */
    pthread_mutex_lock(&mx_concent);
    i = lgw_get_trigcnt(&trig_tstamp);
    pthread_mutex_unlock(&mx_concent);
    if (i != LGW_HAL_SUCCESS) {
        MSG_WARN("[gps] failed to read concentrator timestamp\n");
        return;
    }

    /* try to update time reference with the new GPS time & timestamp */
    pthread_mutex_lock(&mx_timeref);
    i = lgw_gps_sync(&time_reference_gps, trig_tstamp, utc, gps_time);
    pthread_mutex_unlock(&mx_timeref);
    if (i != LGW_GPS_SUCCESS) {
        MSG_WARN("[gps] GPS out of sync, keeping previous time reference\n");
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
    ssize_t nb_char;
    size_t wr_idx = 0;     /* pointer to end of chars in buffer */
    size_t rd_idx;
    size_t frame_end_idx;
    size_t frame_size;

    /* variables for PPM pulse GPS synchronization */
    enum gps_msg latest_msg; /* keep track of latest NMEA message parsed */

    /* initialize some variables before loop */
    memset(serial_buff, 0, sizeof serial_buff);

    while (!exit_sig && !quit_sig) {
        
        rd_idx = 0;
        frame_end_idx = 0;

        /* blocking non-canonical read on serial port */
        nb_char = read(gps_tty_fd, serial_buff + wr_idx, LGW_GPS_MIN_MSG_SIZE);
        if (nb_char <= 0) {
            MSG_WARN("[gps] read() returned value %zd\n", nb_char);
            continue;
        }
        wr_idx += (size_t)nb_char;

        /*******************************************
         * Scan buffer for UBX/NMEA sync chars and *
         * attempt to decode frame if one is found *
         *******************************************/
        while (rd_idx < wr_idx) {
            frame_size = 0;

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
                        MSG_WARN("[gps] could not get a valid message from GPS (no time)\n");
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
    MSG_INFO("End of GPS thread\n");
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
                xtal_correct_ok = true;
                pthread_mutex_unlock(&mx_xcorr);
                ++init_cpt;
            } else {
                /* tracking with low-pass filter */
                x = 1 / xtal_err_cpy;
                pthread_mutex_lock(&mx_xcorr);
                xtal_correct = xtal_correct - xtal_correct/XERR_FILT_COEF + x/XERR_FILT_COEF;
                pthread_mutex_unlock(&mx_xcorr);
            }
        }
    }
    MSG_INFO("End of validation thread\n");
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
                // printf("SPECTRAL SCAN RESULTS - %u Hz: ", freq_hz);
                // for (i = 0; i < LGW_SPECTRAL_SCAN_RESULT_SIZE; i++) {
                //     printf("%u ", results[i]);
                // }
                // printf("\n");


                //TODO : Uncomment above and delete this print loop
                // printf("SPECTRAL SCAN LEVELS - %u Hz: ", freq_hz);
                // for (i = 0; i < LGW_SPECTRAL_SCAN_RESULT_SIZE; i++) {
                //     printf("LEVEL: %u RESULT: %u\n", levels[i], results[i]);
                // }
                // printf("\n");

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
    int i;

    /* configuration file related */
    const char defaut_conf_fname[] = JSON_CONF_DEFAULT;
    const char * conf_fname = defaut_conf_fname; /* pointer to a string we won't touch */

    /* threads */
    pthread_t thrid_listen;
    pthread_t thrid_encode;
    pthread_t thrid_upload;
    pthread_t thrid_gps;
    pthread_t thrid_valid;
    pthread_t thrid_spectral;

    /* message queue initialisation */
    STAILQ_INIT(&head);

    

    /* parse command line options */
    while( (i = getopt( argc, argv, OPTION_ARGS )) != -1 )
    {
        switch( i )
        {
        case 'v':
        verbose =  true;
        break;

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

    /* opening log file */
    log_open();

    // log_open();

    // MSG_ERR("this is a test of sorts\n");

    // MSG_WARN("this is a test with %d numbers!%d\n", 2, 1);

    // log_close();

    // return EXIT_SUCCESS;

    /* configuration files management */
    if (access(conf_fname, R_OK) == 0) { /* if there is a global conf, parse it  */
        MSG_INFO("found configuration file %s, parsing it\n", conf_fname);
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
            MSG_INFO("no debug configuration\n");
        }
    } else {
        MSG_ERR("[main] failed to find any configuration file named %s\n", conf_fname);
        exit(EXIT_FAILURE);
    }

    /* Start GPS so it figures itself out quick */
    if (gps_tty_path[0] != '\0') { /* do not try to open GPS device if no path set */
        i = lgw_gps_enable(gps_tty_path, "ubx7", 0, &gps_tty_fd); /* HAL only supports u-blox 7 for now */
        if (i != LGW_GPS_SUCCESS) {
            MSG_WARN("impossible to open %s for GPS sync (check permissions)\n", gps_tty_path);
            gps_enabled = false;
            gps_ref_valid = false;
        } else {
            MSG_INFO("TTY port %s open for GPS synchronization\n", gps_tty_path);
            gps_enabled = true;
            gps_ref_valid = false;
        }
    }

    /* starting the concentrator */
    if (start_sniffer())
        exit(EXIT_FAILURE);

    /* channel and gateway info encoding and uploading thread */
    i = pthread_create(&thrid_upload, NULL, (void * (*)(void *))thread_upload, NULL);
    if (i != 0) {
        MSG_ERR("[main] impossible to create uploading thread\n");
        exit(EXIT_FAILURE);
    }

    /* end device encoding thread */
    i = pthread_create(&thrid_encode, NULL, (void * (*)(void *))thread_encode, NULL);
    if (i != 0) {
        MSG_ERR("[main] impossible to create encoding thread\n");
        exit(EXIT_FAILURE);
    }

    /* main listener for upstream */
    i = pthread_create(&thrid_listen, NULL, (void * (*)(void *))thread_listen, NULL);
    if (i != 0) {
        MSG_ERR("[main] impossible to create listening thread\n");
        exit(EXIT_FAILURE);
    }

    /* Spectral scan auxiliary thread */
    if (spectral_scan_params.enable == true) {
        i = pthread_create(&thrid_spectral, NULL, (void * (*)(void *))thread_spectral_scan, NULL);
        if (i != 0) {
            MSG_ERR("[main] impossible to create Spectral Scan thread\n");
            exit(EXIT_FAILURE);
        }
    }

    /* GPS thread management */
    if (gps_enabled == true) {
        i = pthread_create(&thrid_gps, NULL, (void * (*)(void *))thread_gps, NULL);
        if (i != 0) {
            MSG_ERR("[main] impossible to create GPS thread\n");
            exit(EXIT_FAILURE);
        }
        i = pthread_create(&thrid_valid, NULL, (void * (*)(void *))thread_valid, NULL);
        if (i != 0) {
            MSG_ERR("[main] impossible to create validation thread\n");
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
        /* Sleep, then create new log once time is up */
        wait_ms(MS_CONV * log_interval);
        pthread_mutex_lock(&mx_log);

        /* close current log and open a new one only if no interrupt signals have been given */
        if (!exit_sig && !quit_sig) {
            log_close();
            log_open();
        }
        
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

    if (spectral_scan_params.enable == true) {
        i = pthread_join(thrid_spectral, NULL);
        if (i != 0) {
            MSG_ERR("Failed to join Spectral Scan thread with %d - %s\n", i, strerror(errno));
        }
    }

    if (gps_enabled == true) {
        pthread_cancel(thrid_gps); /* don't wait for GPS thread, no access to concentrator board */
        pthread_cancel(thrid_valid); /* don't wait for validation thread, no access to concentrator board */

        i = lgw_gps_disable(gps_tty_fd);
        if (i == LGW_HAL_SUCCESS) {
            MSG_INFO("GPS closed successfully\n");
        } else {
            MSG_WARN("failed to close GPS successfully\n");
        }
    }

    if (exit_sig) {
        /* clean up before leaving */
        stop_sniffer();
        stat_cleanup();
    }

    /* message queue deinitialisation */
    STAILQ_INIT(&head);

    MSG_INFO("Exiting packet sniffer program\n");

    /* close the log file */
    log_close();

    return EXIT_SUCCESS;
}

/* --- EOF ------------------------------------------------------------------ */