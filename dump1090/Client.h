#ifndef CLIENT_H
#define CLIENT_H

#include "modesDecode.h"

namespace modes {

  /* Structure used to describe a modes client. */
  struct client {
    int fd;         /* File descriptor. */
    int service;    /* TCP port the client is connected to. */
    char buf[modesDecode::MODES_CLIENT_BUF_SIZE+1];    /* Read buffer. */
    int buflen;                         /* Amount of data on buffer. */
  };

  /* Get an HTTP request header and write the response to the client.
   * Again here we assume that the socket buffer is enough without doing
   * any kind of userspace buffering.
   *
   * Returns 1 on error to signal the caller the client connection should
   * be closed. */
  int handleHTTPRequest(struct client *c);

  /* This function decodes a string representing a Mode S message in
   * raw hex format like: *8D4B969699155600E87406F5B69F;
   * The string is supposed to be at the start of the client buffer
   * and null-terminated.
   * 
   * The message is passed to the higher level layers, so it feeds
   * the selected screen output, the network output and so forth.
   * 
   * If the message looks invalid is silently discarded.
   *
   * The function always returns 0 (success) to the caller as there is
   * no case where we want broken messages here to close the client
   * connection. */
  int decodeHexMessage(struct client *c);

  /* This function polls the clients using read() in order to receive new
   * messages from the net.
   *
   * The message is supposed to be separated by the next message by the
   * separator 'sep', that is a null-terminated C string.
   *
   * Every full message received is decoded and passed to the higher layers
   * calling the function 'handler'.
   *
   * The handler returns 0 on success, or 1 to signal this function we
   * should close the connection with the client in case of non-recoverable
   * errors. */
  void modesReadFromClient(struct client *c, char *sep,
                           int(*handler)(struct client *));

} // namespace


#endif
