
#ifndef MODESDECODE_H
#define MODESDECODE_H

#include "globals.h"
#include "modesMessage.h"
#include "anet.h"
#include "rtl-sdr.h"

extern "C" {
#include <sys/times.h>
}

namespace modes {
  struct client;
}

namespace modesDecode {

/* Program global state. */
struct MMODES {
    /* Internal state */
    pthread_t reader_thread;
    pthread_mutex_t data_mutex;     /* Mutex to synchronize buffer access. */
    pthread_cond_t data_cond;       /* Conditional variable associated. */
    clock_t time;                   /* time stamp when the IQ samples get 
                                       copied into the data buffer */
    struct tms cpu_time;            /* time stamp when the IQ samples get 
                                       copied into the data buffer */
    unsigned char *data;            /* Raw IQ samples buffer */
    uint16_t *magnitude;            /* Magnitude vector */
    uint32_t data_len;              /* Buffer length. */
    int fd;                         /* --ifile or --rfile option file descriptor. */
    int data_ready;                 /* Data ready to be processed. */
    uint32_t *icao_cache;           /* Recently seen ICAO addresses cache. */
    uint16_t *maglut;               /* I/Q -> Magnitude lookup table. */
    int exit;                       /* Exit from the main loop when true. */

    /* RTLSDR */
    int dev_index;
    int gain;
    int enable_agc;
    rtlsdr_dev_t *dev;
    int freq;

    /* Networking */
    char aneterr[ANET_ERR_LEN];
  struct modes::client *clients[MODES_NET_MAX_FD]; /* Our clients. */
    int maxfd;                      /* Greatest fd currently active. */
    int sbsos;                      /* SBS output listening socket. */
    int ros;                        /* Raw output listening socket. */
    int ris;                        /* Raw input listening socket. */
    int https;                      /* HTTP listening socket. */

    /* Configuration */
    char *ifilename;                /* Input form file, --ifile option. */
    char *rfilename;                /* Input form file, --rfile option. */
    int fix_errors;                 /* Single bit error correction if true. */
    int check_crc;                  /* Only display messages with good CRC. */
    int raw;                        /* Raw output format. */
    int debug;                      /* Debugging mode. */
    int net;                        /* Enable networking. */
    int net_only;                   /* Enable just networking. */
    int net_output_sbs_port;        /* SBS output TCP port. */
    int net_output_raw_port;        /* Raw output TCP port. */
    int net_input_raw_port;         /* Raw input TCP port. */
    int net_http_port;              /* HTTP port. */
    int interactive;                /* Interactive mode */
    int interactive_rows;           /* Interactive mode: max number of rows. */
    int interactive_ttl;            /* Interactive mode: TTL before deletion. */
    int stats;                      /* Print stats at exit in --ifile mode. */
    int onlyaddr;                   /* Print only ICAO addresses. */
    int metric;                     /* Use metric units. */
    int aggressive;                 /* Aggressive detection algorithm. */

    /* Interactive mode */
  struct modeSMessage::aircraft *aircrafts;
    long interactive_last_update;  /* Last screen update in milliseconds */

    /* Statistics */
    long stat_valid_preamble;
    long stat_demodulated;
    long stat_goodcrc;
    long stat_badcrc;
    long stat_fixed;
    long stat_single_bit_fix;
    long stat_two_bits_fix;
    long stat_http_requests;
    long stat_sbs_connections;
    long stat_out_of_phase;
};

 extern struct MMODES Modes;

 void modesInitConfig(void);
 void modesInit(void) ;
 void modesInitRTLSDR(void);

 /* Detect a Mode S messages inside the magnitude buffer pointed by 'm' and of
  * size 'mlen' bytes. Every detected Mode S message is convert it into a
  * stream of bits and passed to the function to display it. */
 void detectModeS(const clock_t* time, uint16_t *m, uint32_t mlen);

 /* Decode a raw Mode S message demodulated as a stream of bytes by
  * detectModeS(), and split it into fields populating a modesMessage
  * structure. */
 void decodeModesMessage(struct modeSMessage::modesMessage *mm, 
                         unsigned char *msg);

 /* When a new message is available, because it was decoded from the
  * RTL device, file, or received in the TCP input port, or any other
  * way we can receive a decoded message, we call this function in order
  * to use the message.
  *
  * Basically this function passes a raw message to the upper layers for
  * further processing and visualization. */ 
 void useModesMessage(const clock_t* time, struct modeSMessage::modesMessage *mm);

 /* Turn I/Q samples pointed by Modes.data into the magnitude vector
  * pointed by Modes.magnitude. */
 void computeMagnitudeVector(void);

 /* We read data using a thread, so the main thread only handles decoding
  * without caring about data acquisition. */
 void *readerThreadEntryPoint(void *arg);




} // namespace


#endif
