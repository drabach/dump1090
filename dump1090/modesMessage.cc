
#include "modesMessage.h"
#include "modesDecode.h"
#include "Client.h"

#include <cstring>
#include <cstdio>
#include <cmath>

extern "C" {
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
}


namespace modeSMessage {

long mstime(void) {
    struct timeval tv;
    long mst;

    ::gettimeofday(&tv, NULL);
    mst = ((long)tv.tv_sec)*1000;
    mst += tv.tv_usec/1000;
    return mst;
}

/* ============================= Utility functions ========================== */
/* Always positive MOD operation, used for CPR decoding. */
int cprModFunction(int a, int b) {
    int res = a % b;
    if (res < 0) res += b;
    return res;
}

/* The NL function uses the precomputed table from 1090-WP-9-14 */
int cprNLFunction(double lat) {
    if (lat < 10.47047130) return 59;
    if (lat < 14.82817437) return 58;
    if (lat < 18.18626357) return 57;
    if (lat < 21.02939493) return 56;
    if (lat < 23.54504487) return 55;
    if (lat < 25.82924707) return 54;
    if (lat < 27.93898710) return 53;
    if (lat < 29.91135686) return 52;
    if (lat < 31.77209708) return 51;
    if (lat < 33.53993436) return 50;
    if (lat < 35.22899598) return 49;
    if (lat < 36.85025108) return 48;
    if (lat < 38.41241892) return 47;
    if (lat < 39.92256684) return 46;
    if (lat < 41.38651832) return 45;
    if (lat < 42.80914012) return 44;
    if (lat < 44.19454951) return 43;
    if (lat < 45.54626723) return 42;
    if (lat < 46.86733252) return 41;
    if (lat < 48.16039128) return 40;
    if (lat < 49.42776439) return 39;
    if (lat < 50.67150166) return 38;
    if (lat < 51.89342469) return 37;
    if (lat < 53.09516153) return 36;
    if (lat < 54.27817472) return 35;
    if (lat < 55.44378444) return 34;
    if (lat < 56.59318756) return 33;
    if (lat < 57.72747354) return 32;
    if (lat < 58.84763776) return 31;
    if (lat < 59.95459277) return 30;
    if (lat < 61.04917774) return 29;
    if (lat < 62.13216659) return 28;
    if (lat < 63.20427479) return 27;
    if (lat < 64.26616523) return 26;
    if (lat < 65.31845310) return 25;
    if (lat < 66.36171008) return 24;
    if (lat < 67.39646774) return 23;
    if (lat < 68.42322022) return 22;
    if (lat < 69.44242631) return 21;
    if (lat < 70.45451075) return 20;
    if (lat < 71.45986473) return 19;
    if (lat < 72.45884545) return 18;
    if (lat < 73.45177442) return 17;
    if (lat < 74.43893416) return 16;
    if (lat < 75.42056257) return 15;
    if (lat < 76.39684391) return 14;
    if (lat < 77.36789461) return 13;
    if (lat < 78.33374083) return 12;
    if (lat < 79.29428225) return 11;
    if (lat < 80.24923213) return 10;
    if (lat < 81.19801349) return 9;
    if (lat < 82.13956981) return 8;
    if (lat < 83.07199445) return 7;
    if (lat < 83.99173563) return 6;
    if (lat < 84.89166191) return 5;
    if (lat < 85.75541621) return 4;
    if (lat < 86.53536998) return 3;
    if (lat < 87.00000000) return 2;
    else return 1;
}

int cprNFunction(double lat, int isodd) {
    int nl = cprNLFunction(lat) - isodd;
    if (nl < 1) nl = 1;
    return nl;
}

double cprDlonFunction(double lat, int isodd) {
    return 360.0 / cprNFunction(lat, isodd);
}

/* This algorithm comes from:
 * http://www.lll.lu/~edward/edward/adsb/DecodingADSBposition.html.
 *
 *
 * A few remarks:
 * 1) 131072 is 2^17 since CPR latitude and longitude are encoded in 17 bits.
 */
void decodeCPR(struct modeSMessage::aircraft *a) {
    const double AirDlat0 = 360.0 / 60.0;
    const double AirDlat1 = 360.0 / 59.0;
    double lat0 = a->even_cprlat;
    double lat1 = a->odd_cprlat;
    double lon0 = a->even_cprlon;
    double lon1 = a->odd_cprlon;

    /* Compute the Latitude Index "j" */
    int j = ::floor(((59.0*lat0 - 60.0*lat1) / 131072.0) + 0.5);
    double rlat0 = AirDlat0 * (cprModFunction(j,60) + lat0 / 131072.0);
    double rlat1 = AirDlat1 * (cprModFunction(j,59) + lat1 / 131072.0);

    if (rlat0 >= 270.0) rlat0 -= 360.0;
    if (rlat1 >= 270.0) rlat1 -= 360.0;

    /* Check that both are in the same latitude zone, or abort. */
    if (cprNLFunction(rlat0) != cprNLFunction(rlat1)) return;

    /* Compute ni and the longitude index m */
    if (a->even_cprtime > a->odd_cprtime) {
        /* Use even packet. */
        int ni = cprNFunction(rlat0,0);
        int m = ::floor((((lon0 * (cprNLFunction(rlat0)-1)) -
                          (lon1 *  cprNLFunction(rlat0)  )    ) / 131072.0) + 0.5);
        a->lon = cprDlonFunction(rlat0,0) * (cprModFunction(m,ni)+lon0/131072.0);
        a->lat = rlat0;
    } else {
        /* Use odd packet. */
        int ni = cprNFunction(rlat1,1);
        int m = ::floor((((lon0 * (cprNLFunction(rlat1)-1)) -
                          (lon1 *  cprNLFunction(rlat1)  )    ) / 131072.0) + 0.5);
        a->lon = cprDlonFunction(rlat1,1) * (cprModFunction(m,ni)+lon1/131072.0);
        a->lat = rlat1;
    }
    
    if (a->lon >= 360.0) a->lon -= 360.0;
}

/* Return the aircraft with the specified address, or NULL if no aircraft
 * exists with this address. */
struct aircraft *interactiveFindAircraft(uint32_t addr) {
  struct aircraft *a = modesDecode::Modes.aircrafts;

    while(a) {
        if (a->addr == addr) return a;
        a = a->next;
    }
    return NULL;
}


/* Return a new aircraft structure for the interactive mode linked list
 * of aircrafts. */
struct modeSMessage::aircraft *interactiveCreateAircraft(uint32_t addr) {
  struct modeSMessage::aircraft *a = (struct aircraft*)::malloc(sizeof(*a));

    a->addr = addr;
    ::snprintf(a->hexaddr,sizeof(a->hexaddr),"%06x",(int)addr);
    a->flight[0] = '\0';
    a->identity = 0;
    a->altitude = 0;
    a->rateOfClimb = 0;
    a->vel[0] = a->vel[1] = 0;
    a->speed = 0;
    a->heading = 0;
    a->odd_cprlat = 0;
    a->odd_cprlon = 0;
    a->odd_cprtime = 0;
    a->even_cprlat = 0;
    a->even_cprlon = 0;
    a->even_cprtime = 0;
    a->lat = 0;
    a->lon = 0;
    a->seen = ::time(NULL);
    a->messages = 0;
    a->next = NULL;
    return a;
}


/* On error free the client, collect the structure, adjust maxfd if needed. */
void modesFreeClient(int fd) {
  ::close(fd);
  ::free(modesDecode::Modes.clients[fd]);
    modesDecode::Modes.clients[fd] = NULL;

    if (modesDecode::Modes.debug & modesDecode::MODES_DEBUG_NET)
        ::printf("Closing client %d\n", fd);

    /* If this was our maxfd, rescan the full clients array to check what's
     * the new max. */
    if (modesDecode::Modes.maxfd == fd) {
        int j;

        modesDecode::Modes.maxfd = -1;
        for (j = 0; j < modesDecode::MODES_NET_MAX_FD; j++) {
            if (modesDecode::Modes.clients[j]) modesDecode::Modes.maxfd = j;
        }
    }
}



/* Send the specified message to all clients listening for a given service. */
void modesSendAllClients(int service, void *msg, int len) {
    int j;
    struct modes::client *c;

    for (j = 0; j <= modesDecode::Modes.maxfd; j++) {
        c = modesDecode::Modes.clients[j];
        if (c && c->service == service) {
            int nwritten = write(j, msg, len);
            if (nwritten != len) {
                modesFreeClient(j);
            }
        }
    }
}


void sendSync(void)
  {
    static int counter = 0;
    static struct tms cpu_time;

    if (counter%1024 == 0)
      {
        counter = 1;
        time_t global_time = ::time(NULL);
        long  msTime = mstime();
        clock_t relative_time = ::times(&cpu_time);

        char msg[225];
        msg[224] = '\0';
        ::snprintf(msg, 223, "SYNC %lds %ldms %ldtks:\n", 
                   global_time, msTime, relative_time);

        modesSendAllClients(modesDecode::Modes.ros, msg, ::strlen(msg));
      }
    else counter++;
  }


/* Write raw output to TCP clients. */
 void modesSendRawOutput(const clock_t *time, struct modeSMessage::modesMessage *mm) {
   sendSync();

    char msg[224], *p = msg;
    ::snprintf(p, 223, "%ld*", *time);
    p = ::strstr(p, "*"); p++;
    for (int j = 0; j < mm->msgbits/8; j++) {
        ::sprintf(p, "%02X", mm->msg[j]);
        p += 2;
    }
    *p++ = ';';
    *p++ = '\n';
    modesSendAllClients(modesDecode::Modes.ros, msg, p-msg);
}


/* Write SBS output to TCP clients. */
void modesSendSBSOutput(struct modeSMessage::modesMessage *mm, struct aircraft *a) {
    char msg[256], *p = msg;
    int emergency = 0, ground = 0, alert = 0, spi = 0;

    if (mm->msgtype == 4 || mm->msgtype == 5 || mm->msgtype == 21) {
        /* Node: identity is calculated/kept in base10 but is actually
         * octal (07500 is represented as 7500) */
        if (mm->identity == 7500 || mm->identity == 7600 ||
            mm->identity == 7700) emergency = -1;
        if (mm->fs == 1 || mm->fs == 3) ground = -1;
        if (mm->fs == 2 || mm->fs == 3 || mm->fs == 4) alert = -1;
        if (mm->fs == 4 || mm->fs == 5) spi = -1;
    }

    if (mm->msgtype == 0) {
        p += ::sprintf(p, "MSG,5,,,%02X%02X%02X,,,,,,,%d,,,,,,,,,,",
        mm->aa1, mm->aa2, mm->aa3, mm->altitude);
    } else if (mm->msgtype == 4) {
        p += ::sprintf(p, "MSG,5,,,%02X%02X%02X,,,,,,,%d,,,,,,,%d,%d,%d,%d",
        mm->aa1, mm->aa2, mm->aa3, mm->altitude, alert, emergency, spi, ground);
    } else if (mm->msgtype == 5) {
        p += ::sprintf(p, "MSG,6,,,%02X%02X%02X,,,,,,,,,,,,,%d,%d,%d,%d,%d",
        mm->aa1, mm->aa2, mm->aa3, mm->identity, alert, emergency, spi, ground);
    } else if (mm->msgtype == 11) {
        p += ::sprintf(p, "MSG,8,,,%02X%02X%02X,,,,,,,,,,,,,,,,,",
        mm->aa1, mm->aa2, mm->aa3);
    } else if (mm->msgtype == 17 && mm->metype == 4) {
        p += ::sprintf(p, "MSG,1,,,%02X%02X%02X,,,,,,%s,,,,,,,,0,0,0,0",
        mm->aa1, mm->aa2, mm->aa3, mm->flight);
    } else if (mm->msgtype == 17 && mm->metype >= 9 && mm->metype <= 18) {
        if (a->lat == 0 && a->lon == 0)
            p += ::sprintf(p, "MSG,3,,,%02X%02X%02X,,,,,,,%d,,,,,,,0,0,0,0",
            mm->aa1, mm->aa2, mm->aa3, mm->altitude);
        else
            p += ::sprintf(p, "MSG,3,,,%02X%02X%02X,,,,,,,%d,,,%1.5f,%1.5f,,,"
                            "0,0,0,0",
            mm->aa1, mm->aa2, mm->aa3, mm->altitude, a->lat, a->lon);
    } else if (mm->msgtype == 17 && mm->metype == 19 && mm->mesub == 1) {
        int vr = (mm->vert_rate_sign==0?1:-1) * (mm->vert_rate-1) * 64;

        p += ::sprintf(p, "MSG,4,,,%02X%02X%02X,,,,,,,,%d,%d,,,%i,,0,0,0,0",
        mm->aa1, mm->aa2, mm->aa3, a->speed, a->heading, vr);
    } else if (mm->msgtype == 21) {
        p += ::sprintf(p, "MSG,6,,,%02X%02X%02X,,,,,,,,,,,,,%d,%d,%d,%d,%d",
        mm->aa1, mm->aa2, mm->aa3, mm->identity, alert, emergency, spi, ground);
    } else {
        return;
    }

    *p++ = '\n';
    modesSendAllClients(modesDecode::Modes.sbsos, msg, p-msg);
}



/* Receive new messages and populate the interactive mode with more info. */
struct modeSMessage::aircraft*
interactiveReceiveData(struct modeSMessage::modesMessage *mm) {
    uint32_t addr;
    struct aircraft *a, *aux;
    time_t now = ::time(NULL);

    if (modesDecode::Modes.check_crc && mm->crcok == 0) return NULL;
    addr = (mm->aa1 << 16) | (mm->aa2 << 8) | mm->aa3;

    /* Loookup our aircraft or create a new one. */
    a = interactiveFindAircraft(addr);
    if (!a) {
        a = interactiveCreateAircraft(addr);
        a->next = modesDecode::Modes.aircrafts;
        modesDecode::Modes.aircrafts = a;
    } else {
        /* If it is an already known aircraft, move it on head
         * so we keep aircrafts ordered by received message time.
         *
         * However move it on head only if at least one second elapsed
         * since the aircraft that is currently on head sent a message,
         * othewise with multiple aircrafts at the same time we have an
         * useless shuffle of positions on the screen. */
      if (0 && modesDecode::Modes.aircrafts != a && (now - a->seen) >= 1) {
            aux = modesDecode::Modes.aircrafts;
            while(aux->next != a) aux = aux->next;
            /* Now we are a node before the aircraft to remove. */
            aux->next = aux->next->next; /* removed. */
            /* Add on head */
            a->next = modesDecode::Modes.aircrafts;
            modesDecode::Modes.aircrafts = a;
        }
    }

    a->seen = now;
    a->messages++;

    if (mm->msgtype == 0 || mm->msgtype == 4 || mm->msgtype == 20) {
        a->altitude = mm->altitude;
    } else if (mm->msgtype == 4 || mm->msgtype == 5 || mm->msgtype == 21) {
      a->identity = mm->identity;
    } else if (mm->msgtype == 17) {
        if (mm->metype >= 1 && mm->metype <= 4) {
            memcpy(a->flight, mm->flight, sizeof(a->flight));
        } else if (mm->metype >= 9 && mm->metype <= 18) {
            a->altitude = mm->altitude;
            if (mm->fflag) {
                a->odd_cprlat = mm->raw_latitude;
                a->odd_cprlon = mm->raw_longitude;
                a->odd_cprtime = mstime();
            } else {
                a->even_cprlat = mm->raw_latitude;
                a->even_cprlon = mm->raw_longitude;
                a->even_cprtime = mstime();
            }
            /* If the two data is less than 10 seconds apart, compute
             * the position. */
            if (abs(a->even_cprtime - a->odd_cprtime) <= 10000) {
              decodeCPR(a);
            }
        } else if (mm->metype == 19) {
            if (mm->mesub == 1 || mm->mesub == 2) {
                a->speed = mm->velocity;
                a->heading = mm->heading;
                a->rateOfClimb = (mm->vert_rate_sign==0?1:-1) * (mm->vert_rate-1) * 64;
            }
        }
    }
    return a;
}

/* Show the currently captured interactive data on screen. */
void interactiveShowData(void) {
    struct aircraft *a = modesDecode::Modes.aircrafts;
    time_t now = ::time(NULL);
    int count = 0;

#ifndef RADAR_OUTPUT
    char progress[4];
    progress[3] = '\0';
    ::memset(progress,' ',3);
    progress[::time(NULL)%3] = '.';

    ::printf("\x1b[H\x1b[2J");    /* Clear the screen */
    ::printf(
"Hex    Flight   Altitude  Speed   Lat       Lon       Head  Messages Seen %s\n"
"--------------------------------------------------------------------------------\n",
        progress);
#endif

    while(a && count < modesDecode::Modes.interactive_rows) {
        int altitude = a->altitude, speed = a->speed;

        /* Convert units to metric if --metric was specified. */
        if (modesDecode::Modes.metric) {
            altitude /= 3.2828;
            speed *= 1.852;
        }

#ifndef RADAR_OUTPUT
        ::printf("%-6s %-8s %-9d %-7d %-7.03f   %-7.03f   %-3d   %-9ld %d sec\n",
            a->hexaddr, a->flight, altitude, speed,
            a->lat, a->lon, a->heading, a->messages,
            (int)(now - a->seen));
#else
        int trackLabel1Len = 
          ::strlen(a->flight)+::strlen(a->hexaddr)+2;
        char* trackLabel1 = new char[trackLabel1Len];
        trackLabel1[trackLabel1Len-1] = '\0';
        if (a->identity && a->identity != modesDecode::MODES_SQUAWK)
          ::snprintf(trackLabel1, trackLabel1Len, "%s %04d",
                     a->flight ? a->flight: "", a->identity);
        else if (a->flight[0] != '\0')
          ::snprintf(trackLabel1, trackLabel1Len, "%s",
                     a->flight ? a->flight: "");
        else ::snprintf(trackLabel1, trackLabel1Len, "%s", a->hexaddr);

        char* trackLabel2 = new char[16];
        trackLabel2[15] = '\0';
        ::snprintf(trackLabel2, 15, "h%3d|%d s%d",
                   altitude/100, a->rateOfClimb, speed);

        char* trackLabel3 = NULL;

        ::printf ("%09lu,TRK,%d,%d,%10.6f,%11.6f,%3d,%3d,%3d,%3d,%s,\"%s\","
                  "\"%s\",\"%s\"\n", 
        (now%8640)*1000, 0, a->addr%2048, 
        a->lat, a->lon, altitude/100, 
        a->heading, speed, a->rateOfClimb, "",
        trackLabel1 ? trackLabel1 : "", 
        trackLabel2 ? trackLabel2 : "", 
        trackLabel3 ? trackLabel3 : "");

        if (a->lat > 1e-7 || a->lon > 1e-7) 
          ::printf ("%09lu,PLT,%d,%10.6f,%11.6f\n", 
                    (now%8640)*1000, 0, 
                    a->lat, a->lon);

        delete [] trackLabel1;
        delete [] trackLabel2;
#endif
        a = a->next;
        count++;
    }
}

/* When in interactive mode If we don't receive new nessages within
 * MODES_INTERACTIVE_TTL seconds we remove the aircraft from the list. */
void interactiveRemoveStaleAircrafts(void) {
    struct modeSMessage::aircraft *a = modesDecode::Modes.aircrafts;
    struct modeSMessage::aircraft *prev = NULL;
    time_t now = ::time(NULL);

    while(a) {
        if ((now - a->seen) > modesDecode::Modes.interactive_ttl) {
            struct modeSMessage::aircraft *next = a->next;
            /* Remove the element from the linked list, with care
             * if we are removing the first element. */
            ::free(a);
            if (!prev)
                modesDecode::Modes.aircrafts = next;
            else
                prev->next = next;
            a = next;
        } else {
            prev = a;
            a = a->next;
        }
    }
}

/* This function gets called from time to time when the decoding thread is
 * awakened by new data arriving. This usually happens a few times every
 * second. */
void modesAcceptClients(void) {
    int fd, port;
    unsigned int j;
    struct modes::client *c;
    int services[4];

    services[0] = modesDecode::Modes.ros;
    services[1] = modesDecode::Modes.ris;
    services[2] = modesDecode::Modes.https;
    services[3] = modesDecode::Modes.sbsos;

    for (j = 0; j < sizeof(services)/sizeof(int); j++) {
        fd = anetTcpAccept(modesDecode::Modes.aneterr, services[j], NULL, &port);
        if (fd == -1) continue;

        if (fd >= modesDecode::MODES_NET_MAX_FD) {
            close(fd);
            return; /* Max number of clients reached. */
        }

        anetNonBlock(modesDecode::Modes.aneterr, fd);
        c = (struct modes::client*)::malloc(sizeof(*c));
        c->service = services[j];
        c->fd = fd;
        c->buflen = 0;
        modesDecode::Modes.clients[fd] = c;
        anetSetSendBuffer(modesDecode::Modes.aneterr, fd,
                          modesDecode::MODES_NET_SNDBUF_SIZE);

        if (modesDecode::Modes.maxfd < fd) 
          modesDecode::Modes.maxfd = fd;
        if (services[j] == modesDecode::Modes.sbsos) 
          modesDecode::Modes.stat_sbs_connections++;

        j--; /* Try again with the same listening port. */

        if (modesDecode::Modes.debug & modesDecode::MODES_DEBUG_NET)
            ::printf("Created new client %d\n", fd);
    }
}


/* Read data from clients. This function actually delegates a lower-level
 * function that depends on the kind of service (raw, http, ...). */
void modesReadFromClients(void) {
    int j;
    struct modes::client *c;

    for (j = 0; j <= modesDecode::Modes.maxfd; j++) {
        if ((c = modesDecode::Modes.clients[j]) == NULL) continue;
        if (c->service == modesDecode::Modes.ris)
          modes::modesReadFromClient(c,(char*)"\n",modes::decodeHexMessage);
        else if (c->service == modesDecode::Modes.https)
          modes::modesReadFromClient(c,(char*)"\r\n\r\n",modes::handleHTTPRequest);
    }
}


/* This function is called a few times every second by main in order to
 * perform tasks we need to do continuously, like accepting new clients
 * from the net, refreshing the screen in interactive mode, and so forth. */
void backgroundTasks(void) {
    if (modesDecode::Modes.net) {
        modesAcceptClients();
        modesReadFromClients();
        interactiveRemoveStaleAircrafts();
    }

    /* Refresh screen when in interactive mode. */
    long msTime = mstime();
    if (modesDecode::Modes.interactive == 1&&
        (msTime - modesDecode::Modes.interactive_last_update) >
        modesDecode::MODES_INTERACTIVE_REFRESH_TIME)
    {
        interactiveRemoveStaleAircrafts();
        interactiveShowData();
        modesDecode::Modes.interactive_last_update = msTime;
    }
}


} // namespace

