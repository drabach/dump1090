/* Mode1090, a Mode S messages decoder for RTLSDR devices.
 *
 * Copyright (C) 2012 by Salvatore Sanfilippo <antirez@gmail.com>
 * Modified (code reorganized, radar output and time stamps added) (C) 2013 by A. Bach
 *
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 * 
 *  *  Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  *  Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cerrno>
#include <cmath>
#include <cctype>

extern "C" {
#include <pthread.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/time.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/stat.h>
}

#include "rtl-sdr.h"
#include "anet.h"

#include "globals.h"
#include "modesDecode.h"
#include "modesMessage.h" 



/* ============================== Snip mode ================================= */

/* Get raw IQ samples and filter everything is < than the specified level
 * for more than 256 samples in order to reduce example file size. */
void snipMode(int level) {
    int i, q;
    long c = 0;

    while ((i = getchar()) != EOF && (q = getchar()) != EOF) {
        if (abs(i-127) < level && abs(q-127) < level) {
            c++;
            if (c > modesDecode::MODES_PREAMBLE_US*4) continue;
        } else {
            c = 0;
        }
        putchar(i);
        putchar(q);
    }
}

/* ============================= Networking =================================
 * Note: here we disregard any kind of good coding practice in favor of
 * extreme simplicity, that is:
 *
 * 1) We only rely on the kernel buffers for our I/O without any kind of
 *    user space buffering.
 * 2) We don't register any kind of event handler, from time to time a
 *    function gets called and we accept new connections. All the rest is
 *    handled via non-blocking I/O and manually pulling clients to see if
 *    they have something new to share with us when reading is needed.
 */

/* Networking "stack" initialization. */
void modesInitNet(void) {
  static const int no_services = 4;

    struct {
        char *descr;
        int *socket;
        int port;
    } services[no_services] = {
        {(char*)"Raw TCP output", 
         &modesDecode::Modes.ros, modesDecode::Modes.net_output_raw_port},
        {(char*)"Raw TCP input", 
         &modesDecode::Modes.ris, modesDecode::Modes.net_input_raw_port},
        {(char*)"HTTP server", 
         &modesDecode::Modes.https, modesDecode::Modes.net_http_port},
        {(char*)"Basestation TCP output", 
         &modesDecode::Modes.sbsos, modesDecode::Modes.net_output_sbs_port}
    };

    ::memset(modesDecode::Modes.clients,0,sizeof(modesDecode::Modes.clients));
    modesDecode::Modes.maxfd = -1;

    signal(SIGPIPE, SIG_IGN);

    for (int j = 0; j < no_services; j++) {
        int s = anetTcpServer(modesDecode::Modes.aneterr, services[j].port, NULL);
        if (s == -1) {
            ::fprintf(stderr, "Error opening the listening port %d (%s): %s\n",
                services[j].port, services[j].descr, strerror(errno));
            ::exit(1);
        }
        anetNonBlock(modesDecode::Modes.aneterr, s);
        *services[j].socket = s;
    }
}

/* ================================ Help ==================================== */
void showHelp(void) {
    ::printf(
"--device-index <index>   Select RTL device (default: 0).\n"
"--gain <db>              Set gain (default: max gain. Use -100 for auto-gain).\n"
"--enable-agc             Enable the Automatic Gain Control (default: off).\n"
"--freq <hz>              Set frequency (default: 1090 Mhz).\n"
"--ifile <filename>       Read binary data from file (use '-' for stdin).\n"
"--rfile <filename>       Read raw data from file (use '-' for stdin).\n"
"--interactive            Interactive mode refreshing data on screen.\n"
"--interactive-rows <num> Max number of rows in interactive mode (default: 15).\n"
"--interactive-ttl <sec>  Remove from list if idle for <sec> (default: 60).\n"
"--raw                    Show only messages hex values.\n"
"--net                    Enable networking.\n"
"--net-only               Enable just networking, no RTL device or file used.\n"
"--net-ro-port <port>     TCP listening port for raw output (default: 30002).\n"
"--net-ri-port <port>     TCP listening port for raw input (default: 30001).\n"
"--net-http-port <port>   HTTP server port (default: 8080).\n"
"--net-sbs-port <port>    TCP listening port for BaseStation format output (default: 30003).\n"
"--no-fix                 Disable single-bits error correction using CRC.\n"
"--no-crc-check           Disable messages with broken CRC (discouraged).\n"
"--aggressive             More CPU for more messages (two bits fixes, ...).\n"
"--stats                  With --ifile print stats at exit. No other output.\n"
"--onlyaddr               Show only ICAO addresses (testing purposes).\n"
"--metric                 Use metric units (meters, km/h, ...).\n"
"--snip <level>           Strip IQ file removing samples < level.\n"
"--debug <flags>          Debug mode (verbose), see README for details.\n"
"--help                   Show this help.\n"
"\n"
"Debug mode flags: d = Log frames decoded with errors\n"
"                  D = Log frames decoded with zero errors\n"
"                  c = Log frames with bad CRC\n"
"                  C = Log frames with good CRC\n"
"                  p = Log frames with bad preamble\n"
"                  n = Log network debugging info\n"
"                  j = Log frames to frames.js, loadable by debug.html.\n"
    );
}


/* ================================ Main ==================================== */

int main(int argc, char **argv) {
    int j;

    /* Set sane defaults. */
    modesDecode::modesInitConfig();

    /* Parse the command line options */
    for (j = 1; j < argc; j++) {
        int more = j+1 < argc; /* There are more arguments. */

        if (!::strcmp(argv[j],"--device-index") && more) {
            modesDecode::Modes.dev_index = atoi(argv[++j]);
        } else if (!::strcmp(argv[j],"--gain") && more) {
            modesDecode::Modes.gain = atof(argv[++j])*10; /* Gain is in tens of DBs */
        } else if (!::strcmp(argv[j],"--enable-agc")) {
            modesDecode::Modes.enable_agc++;
        } else if (!::strcmp(argv[j],"--freq") && more) {
            modesDecode::Modes.freq = strtoll(argv[++j],NULL,10);
        } else if (!::strcmp(argv[j],"--ifile") && more) {
            modesDecode::Modes.ifilename = strdup(argv[++j]);
        } else if (!::strcmp(argv[j],"--rfile") && more) {
            modesDecode::Modes.rfilename = strdup(argv[++j]);
        } else if (!::strcmp(argv[j],"--no-fix")) {
            modesDecode::Modes.fix_errors = 0;
        } else if (!::strcmp(argv[j],"--no-crc-check")) {
            modesDecode::Modes.check_crc = 0;
        } else if (!::strcmp(argv[j],"--raw")) {
            modesDecode::Modes.raw = 1;
        } else if (!::strcmp(argv[j],"--net")) {
            modesDecode::Modes.net = 1;
        } else if (!::strcmp(argv[j],"--net-only")) {
            modesDecode::Modes.net = 1;
            modesDecode::Modes.net_only = 1;
        } else if (!::strcmp(argv[j],"--net-ro-port") && more) {
            modesDecode::Modes.net_output_raw_port = atoi(argv[++j]);
        } else if (!::strcmp(argv[j],"--net-ri-port") && more) {
            modesDecode::Modes.net_input_raw_port = atoi(argv[++j]);
        } else if (!::strcmp(argv[j],"--net-http-port") && more) {
            modesDecode::Modes.net_http_port = atoi(argv[++j]);
        } else if (!::strcmp(argv[j],"--net-sbs-port") && more) {
            modesDecode::Modes.net_output_sbs_port = atoi(argv[++j]);
        } else if (!::strcmp(argv[j],"--onlyaddr")) {
            modesDecode::Modes.onlyaddr = 1;
        } else if (!::strcmp(argv[j],"--metric")) {
            modesDecode::Modes.metric = 1;
        } else if (!::strcmp(argv[j],"--aggressive")) {
            modesDecode::Modes.aggressive++;
        } else if (!::strcmp(argv[j],"--interactive")) {
            modesDecode::Modes.interactive = 1;
        } else if (!::strcmp(argv[j],"--interactive-rows")) {
            modesDecode::Modes.interactive_rows = atoi(argv[++j]);
        } else if (!::strcmp(argv[j],"--interactive-ttl")) {
            modesDecode::Modes.interactive_ttl = atoi(argv[++j]);
        } else if (!::strcmp(argv[j],"--debug") && more) {
            char *f = argv[++j];
            while(*f) {
                switch(*f) {
                case 'D': modesDecode::Modes.debug |= modesDecode::MODES_DEBUG_DEMOD; break;
                case 'd': modesDecode::Modes.debug |= modesDecode::MODES_DEBUG_DEMODERR; break;
                case 'C': modesDecode::Modes.debug |= modesDecode::MODES_DEBUG_GOODCRC; break;
                case 'c': modesDecode::Modes.debug |= modesDecode::MODES_DEBUG_BADCRC; break;
                case 'p': modesDecode::Modes.debug |= modesDecode::MODES_DEBUG_NOPREAMBLE; break;
                case 'n': modesDecode::Modes.debug |= modesDecode::MODES_DEBUG_NET; break;
                case 'j': modesDecode::Modes.debug |= modesDecode::MODES_DEBUG_JS; break;
                default:
                    ::fprintf(stderr, "Unknown debugging flag: %c\n", *f);
                    ::exit(1);
                    break;
                }
                f++;
            }
        } else if (!::strcmp(argv[j],"--stats")) {
            modesDecode::Modes.stats = 1;
        } else if (!::strcmp(argv[j],"--snip") && more) {
            snipMode(atoi(argv[++j]));
            ::exit(0);
        } else if (!::strcmp(argv[j],"--help")) {
            showHelp();
            ::exit(0);
        } else {
            ::fprintf(stderr,
                "Unknown or not enough arguments for option '%s'.\n\n",
                argv[j]);
            showHelp();
            ::exit(1);
        }
    }

    /* Initialization */
    modesDecode::modesInit();
    if (modesDecode::Modes.net_only) {
        ::fprintf(stderr,"Net-only mode, no RTL device or file open.\n");
    } else if ((modesDecode::Modes.ifilename == NULL) && 
               (modesDecode::Modes.rfilename == NULL)) {
        modesDecode::modesInitRTLSDR();
    } else {
      if ((modesDecode::Modes.ifilename &&
           modesDecode::Modes.ifilename[0] == '-' && 
           modesDecode::Modes.ifilename[1] == '\0') ||
          (modesDecode::Modes.rfilename &&
           modesDecode::Modes.rfilename[0] == '-' && 
           modesDecode::Modes.rfilename[1] == '\0') ) {
        modesDecode::Modes.fd = STDIN_FILENO;
        } else if (modesDecode::Modes.ifilename != NULL)
          {
            if ((modesDecode::Modes.fd = 
                 ::open(modesDecode::Modes.ifilename,O_RDONLY)) == -1) 
              {
                perror("Opening binary data file");
                ::exit(1);
              }
          } else if (modesDecode::Modes.rfilename != NULL)
          {
            if ((modesDecode::Modes.fd = 
                 ::open(modesDecode::Modes.rfilename,O_RDONLY)) == -1) 
              {
                perror("Opening raw data file");
                ::exit(1);
              }
          }
      }
    if (modesDecode::Modes.net) modesInitNet();

    /* If the user specifies --net-only, just run in order to serve network
     * clients without reading data from the RTL device. */
    while (modesDecode::Modes.net_only) {
      modeSMessage::backgroundTasks();
        usleep(100000);
    }

    /* Create the thread that will read the data from the device. */
    ::pthread_create(&modesDecode::Modes.reader_thread, NULL, 
                     modesDecode::readerThreadEntryPoint, NULL);

    ::pthread_mutex_lock(&modesDecode::Modes.data_mutex);
    while(1) {
        if (!modesDecode::Modes.data_ready) {
            ::pthread_cond_wait(&modesDecode::Modes.data_cond,
                                &modesDecode::Modes.data_mutex);
            continue;
        }
        modesDecode::computeMagnitudeVector();

        /* Signal to the other thread that we processed the available data
         * and we want more (useful for --ifile). */
        modesDecode::Modes.data_ready = 0;
        ::pthread_cond_signal(&modesDecode::Modes.data_cond);

        /* Process data after releasing the lock, so that the capturing
         * thread can read data while we perform computationally expensive
         * stuff * at the same time. (This should only be useful with very
         * slow processors). */
        ::pthread_mutex_unlock(&modesDecode::Modes.data_mutex);
        modesDecode::detectModeS(&modesDecode::Modes.time,
                                 modesDecode::Modes.magnitude, 
                                 modesDecode::Modes.data_len/2);
        modeSMessage::backgroundTasks();
        ::pthread_mutex_lock(&modesDecode::Modes.data_mutex);
        if (modesDecode::Modes.exit) break;
    }

    /* If --ifile and --stats were given, print statistics. */
    if (modesDecode::Modes.stats && modesDecode::Modes.ifilename) {
        ::printf("%ld valid preambles\n", modesDecode::Modes.stat_valid_preamble);
        ::printf("%ld demodulated again after phase correction\n",
            modesDecode::Modes.stat_out_of_phase);
        ::printf("%ld demodulated with zero errors\n",
            modesDecode::Modes.stat_demodulated);
        ::printf("%ld with good crc\n", modesDecode::Modes.stat_goodcrc);
        ::printf("%ld with bad crc\n", modesDecode::Modes.stat_badcrc);
        ::printf("%ld errors corrected\n", modesDecode::Modes.stat_fixed);
        ::printf("%ld single bit errors\n", modesDecode::Modes.stat_single_bit_fix);
        ::printf("%ld two bits errors\n", modesDecode::Modes.stat_two_bits_fix);
        ::printf("%ld total usable messages\n",
            modesDecode::Modes.stat_goodcrc + modesDecode::Modes.stat_fixed);
    }

    ::rtlsdr_close(modesDecode::Modes.dev);
    return 0;
}
