#ifndef DUMP1090_GLOBALS_H
#define DUMP1090_GLOBALS_H


namespace modesDecode {

static const  int MODES_DEFAULT_RATE        =2000000;
static const  int MODES_DEFAULT_FREQ        =1090000000;
static const  int MODES_DEFAULT_WIDTH       =1000;
static const  int MODES_DEFAULT_HEIGHT      =700;
static const  int MODES_ASYNC_BUF_NUMBER    =12;
static const unsigned int MODES_DATA_LEN    =(16*16384);   /* 256k */
static const int MODES_AUTO_GAIN            =-100 ;        /* Use automatic gain. */
static const int MODES_MAX_GAIN             =999999;       /* Use max available gain. */

static const  int MODES_PREAMBLE_US         =8;       /* microseconds */
static const  int MODES_LONG_MSG_BITS       =112;
static const  int MODES_SHORT_MSG_BITS      =56;
static const  int MODES_FULL_LEN            =(MODES_PREAMBLE_US+MODES_LONG_MSG_BITS);
static const  int MODES_LONG_MSG_BYTES      =(112/8);
static const  int MODES_SHORT_MSG_BYTES     =(56/8);

static const  int MODES_ICAO_CACHE_LEN      =1024; /* Power of two required. */
static const unsigned int MODES_ICAO_CACHE_TTL =60;   /* Time to live of cached addresses. */
static const  int MODES_UNIT_FEET   =0;
static const  int MODES_UNIT_METERS =1;

static const int MODES_DEBUG_DEMOD      =(1<<0);
static const int MODES_DEBUG_DEMODERR   =(1<<1);
static const int MODES_DEBUG_BADCRC     =(1<<2);
static const int MODES_DEBUG_GOODCRC    =(1<<3);
static const int MODES_DEBUG_NOPREAMBLE =(1<<4);
static const int MODES_DEBUG_NET        =(1<<5);
static const int MODES_DEBUG_JS         =(1<<6);

/* When debug is set to MODES_DEBUG_NOPREAMBLE, the first sample must be
 * at least greater than a given level for us to dump the signal. */
static const int MODES_DEBUG_NOPREAMBLE_LEVEL =25;

static const int MODES_INTERACTIVE_REFRESH_TIME =500 ;     /* Milliseconds */
static const int MODES_INTERACTIVE_ROWS =15;               /* Rows on screen */
static const int MODES_INTERACTIVE_TTL =60;                /* TTL before being removed */

static const int MODES_NET_MAX_FD          =1024;
static const int MODES_NET_OUTPUT_SBS_PORT =30003;
static const int MODES_NET_OUTPUT_RAW_PORT =30002;
static const int MODES_NET_INPUT_RAW_PORT  =30001;
static const int MODES_NET_HTTP_PORT       =8080;
static const int MODES_CLIENT_BUF_SIZE     =1024;
static const int MODES_NET_SNDBUF_SIZE     =(1024*64);

static const int MODES_SQUAWK              = 1000; /* decimal notation - but meant octal*/

}

#endif
