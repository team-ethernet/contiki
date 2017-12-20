#ifndef GPRS_A6_H
#define GPRS_A6_H

typedef enum {
  GPRS_CONN_CONNECTED,
  GPRS_CONN_SOCKET_CLOSED,
  GPRS_CONN_SOCKET_TIMEDOUT,
  GPRS_CONN_ABORTED,
  GPRS_CONN_DATA_SENT
} gprs_conn_event_t;

process_event_t a6at_gprs_init;
process_event_t a6at_gprs_activate;
process_event_t a6at_gprs_connection;

#define GPRS_MAX_SEND_LEN 1024
#define GPRS_MAX_RECV_LEN 1024

#define GPRS_MAX_APN_LEN 32

struct gprs_context {
  uint8_t active; /* Context active? */
  char *pdptype; /* allowed values: "IP" and "IPV6" */
  char apn[GPRS_MAX_APN_LEN+1];
};

#define GPRS_MAX_CONNECTION 1
struct gprs_connection {
  struct gprs_context *context;
  const char *proto;
  const char *ipaddr;
  uint16_t port;
  void *callback; 
  struct tcp_socket_gprs *socket;
};

typedef int (* gprs_callback_t)(struct gprs_connection *, int);

process_event_t sc16is_input_event;
process_event_t at_match_event;

process_event_t a6at_gprs_init;
//process_event_t a6at_gprs_activate;
process_event_t a6at_gprs_connect;
process_event_t a6at_gprs_send;
process_event_t a6at_gprs_close;

void
gprs_init();

int
gprs_set_context(struct gprs_context *gcontext, char *pdptype, char *apn);

struct gprs_connection *
gprs_connection(const char *proto, const char *ipaddr, uint16_t port, struct tcp_socket_gprs *socket);

void
gprs_send(struct tcp_socket_gprs *socket);

int
gprs_register(struct gprs_connection *gconn,
              struct tcp_socket_gprs *socket,
              void *callback);

int
gprs_unregister(struct gprs_connection *gconn);

void
gprs_close(struct tcp_socket_gprs *socket);
#endif /* GPRS_A6_H */
