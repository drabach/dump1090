#include "Client.h"
#include "modesMessage.h"
#include "modesDecode.h"

#include <string>
#include <cstdio>
#include <cstring>
#include <cerrno>

extern "C" {
#include <fcntl.h>
#include <sys/stat.h>
}

namespace modes {

  const static std::string MODES_CONTENT_TYPE_HTML = 
    std::string("text/html;charset=utf-8");
  const static std::string MODES_CONTENT_TYPE_JSON = 
    std::string("application/json;charset=utf-8");

  int hexDigitVal(int c);
  char *aircraftsToJson(int *len);

  int handleHTTPRequest(struct client *c)
  {
    char hdr[512];
    int clen, hdrlen;
    int httpver, keepalive;
    char *p, *url, *content;
    const char *ctype;
  
    if (modesDecode::Modes.debug & modesDecode::MODES_DEBUG_NET)
      ::printf("\nHTTP request: %s\n", c->buf);
  
    /* Minimally parse the request. */
    httpver = (::strstr(c->buf, "HTTP/1.1") != NULL) ? 11 : 10;
    if (httpver == 10) {
      /* HTTP 1.0 defaults to close, unless otherwise specified. */
      keepalive = ::strstr(c->buf, "Connection: keep-alive") != NULL;
    } else if (httpver == 11) {
      /* HTTP 1.1 defaults to keep-alive, unless close is specified. */
      keepalive = ::strstr(c->buf, "Connection: close") == NULL;
    }
  
    /* Identify he URL. */
    p = ::strchr(c->buf,' ');
    if (!p) return 1; /* There should be the method and a space... */
    url = ++p; /* Now this should point to the requested URL. */
    p = ::strchr(p, ' ');
    if (!p) return 1; /* There should be a space before HTTP/... */
    *p = '\0';
  
    if (modesDecode::Modes.debug & modesDecode::MODES_DEBUG_NET) {
      ::printf("\nHTTP keep alive: %d\n", keepalive);
      ::printf("HTTP requested URL: %s\n\n", url);
    }
  
    /* Select the content to send, we have just two so far:
     * "/" -> Our google map application.
     * "/data.json" -> Our ajax request to update aircrafts. */
    if (::strstr(url, "/data.json")) {
      content = aircraftsToJson(&clen);
      ctype = MODES_CONTENT_TYPE_JSON.c_str();
    } else {
      struct stat sbuf;
      int fd = -1;
    
      if (::stat("gmap.html",&sbuf) != -1 &&
          (fd = ::open("gmap.html",O_RDONLY)) != -1)
        {
          content = (char*)::malloc(sbuf.st_size);
          if (::read(fd,content,sbuf.st_size) == -1) {
            ::snprintf(content,sbuf.st_size,"Error reading from file: %s",
                       ::strerror(errno));
          }
          clen = sbuf.st_size;
        } 
      else 
        {
          char buf[128];
          
          clen = ::snprintf(buf,sizeof(buf),"Error opening HTML file: %s",
                            ::strerror(errno));
          content = ::strdup(buf);
        }

      if (fd != -1) ::close(fd);
      ctype = MODES_CONTENT_TYPE_HTML.c_str();
    }
  
    /* Create the header and send the reply. */
    hdrlen = ::snprintf(hdr, sizeof(hdr),
                        "HTTP/1.1 200 OK\r\n"
                        "Server: Dump1090\r\n"
                        "Content-Type: %s\r\n"
                        "Connection: %s\r\n"
                        "Content-Length: %d\r\n"
                        "\r\n",
                        ctype,
                        keepalive ? "keep-alive" : "close",
                        clen);
  
    if (modesDecode::Modes.debug & modesDecode::MODES_DEBUG_NET)
      ::printf("HTTP Reply header:\n%s", hdr);
  
    /* Send header and content. */
    if (::write(c->fd, hdr, hdrlen) == -1 ||
        ::write(c->fd, content, clen) == -1)
      {
        ::free(content);
        return 1;
      }
    ::free(content);
    modesDecode::Modes.stat_http_requests++;
    return !keepalive;
  }


  /* Return a description of aircrafts in json. */
  char *aircraftsToJson(int *len) {
    struct modeSMessage::aircraft *a = modesDecode::Modes.aircrafts;
    int buflen = 1024; /* The initial buffer is incremented as needed. */
    char *buf = (char*)::malloc(buflen), *p = buf;
    int l;

    l = ::snprintf(p,buflen,"[\n");
    p += l; buflen -= l;
    while(a) {
      int altitude = a->altitude, speed = a->speed;

      /* Convert units to metric if --metric was specified. */
      if (modesDecode::Modes.metric) {
        altitude /= 3.2828;
        speed *= 1.852;
      }

      if (a->lat != 0 && a->lon != 0) {
        l = ::snprintf(p,buflen,
                       "{\"hex\":\"%s\", \"flight\":\"%s\", \"lat\":%f, "
                       "\"lon\":%f, \"altitude\":%d, \"track\":%d, "
                       "\"speed\":%d},\n",
                       a->hexaddr, a->flight, a->lat, a->lon, a->altitude, a->heading,
                       a->speed);
        p += l; buflen -= l;
        /* Resize if needed. */
        if (buflen < 256) {
          int used = p-buf;
          buflen += 1024; /* Our increment. */
          buf = (char*)::realloc(buf,used+buflen);
          p = buf+used;
        }
      }
      a = a->next;
    }
    /* Remove the final comma if any, and closes the json array. */
    if (*(p-2) == ',') {
      *(p-2) = '\n';
      p--;
      buflen++;
    }
    l = ::snprintf(p,buflen,"]\n");
    p += l; buflen -= l;

    *len = p-buf;
    return buf;
  }


  int decodeHexMessage(struct client *c)  
  {
    char *hex = c->buf;
    int l = ::strlen(hex), j;
    unsigned char msg[modesDecode::MODES_LONG_MSG_BYTES];
    struct modeSMessage::modesMessage mm;

    /* Remove spaces on the left and on the right. */
    while(l && ::isspace(hex[l-1])) {
      hex[l-1] = '\0';
      l--;
    }
    while(::isspace(*hex)) {
      hex++;
      l--;
    }

    /* find the * character */
    char* delim = strstr(hex, "*");
    if (delim == NULL) return 1;
    delim[0] = '\0';

    /* read the time */
    clock_t time = 0l;
    sscanf(hex, "%ld", &time);
    delim[0] = '*';
    hex = delim;
    l -= (delim-hex);

    /* Turn the message into binary. */
    if (l < 2 || hex[0] != '*' || hex[l-1] != ';') return 0;
    hex++; l-=2; /* Skip * and ; */
    if (l > modesDecode::MODES_LONG_MSG_BYTES*2) return 0; 
    /* Too long message... broken. */
    for (j = 0; j < l; j += 2) {
      int high = hexDigitVal(hex[j]);
      int low = hexDigitVal(hex[j+1]);

      if (high == -1 || low == -1) return 0;
      msg[j/2] = (high<<4) | low;
    }
    modesDecode::decodeModesMessage(&mm,msg);
    modesDecode::useModesMessage(&time, &mm);
    return 0;
  }

  void modesReadFromClient(struct client *c, char *sep,
                           int(*handler)(struct client *))
  {
    while(1) {
      int left = modesDecode::MODES_CLIENT_BUF_SIZE - c->buflen;
      int nread = ::read(c->fd, c->buf+c->buflen, left);
      int fullmsg = 0;
      int i;
      char *p;

      if (nread <= 0) {
        if (nread == 0 || errno != EAGAIN) {
          /* Error, or end of file. */
          modeSMessage::modesFreeClient(c->fd);
        }
        break; /* Serve next client. */
      }
      c->buflen += nread;

      /* Always null-term so we are free to use strstr() */
      c->buf[c->buflen] = '\0';

      /* If there is a complete message there must be the separator 'sep'
       * in the buffer, note that we full-scan the buffer at every read
       * for simplicity. */
      while ((p = strstr(c->buf, sep)) != NULL) {
        i = p - c->buf; /* Turn it as an index inside the buffer. */
        c->buf[i] = '\0'; /* Te handler expects null terminated strings. */
        /* Call the function to process the message. It returns 1
         * on error to signal we should close the client connection. */
        if (handler(c)) {
          modeSMessage::modesFreeClient(c->fd);
          return;
        }
        /* Move what's left at the start of the buffer. */
        i += strlen(sep); /* The separator is part of the previous msg. */
        ::memmove(c->buf,c->buf+i,c->buflen-i);
        c->buflen -= i;
        c->buf[c->buflen] = '\0';
        /* Maybe there are more messages inside the buffer.
         * Start looping from the start again. */
        fullmsg = 1;
      }
      /* If our buffer is full discard it, this is some badly
       * formatted shit. */
      if (c->buflen == modesDecode::MODES_CLIENT_BUF_SIZE) {
        c->buflen = 0;
        /* If there is garbage, read more to discard it ASAP. */
        continue;
      }
      /* If no message was decoded process the next client, otherwise
       * read more data from the same client. */
      if (!fullmsg) break;
    }
  }

  /* Turn an hex digit into its 4 bit decimal value.
   * Returns -1 if the digit is not in the 0-F range. */
  int hexDigitVal(int c) {
    c = tolower(c);
    if (c >= '0' && c <= '9') return c-'0';
    else if (c >= 'a' && c <= 'f') return c-'a'+10;
    else return -1;
  }


}
