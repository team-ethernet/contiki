#ifndef GPRS_A6_H
#define GPRS_A6_H


#define GPRS_MAX_SEND_LEN 1024
//#define GPRS_MAX_SEND_LEN 512
#define GPRS_MAX_RECV_LEN 1024

/* How long to wait for APN registration (sec) */
#define GPRS_APN_REGISTER_TIMEOUT 60
/* How long to wait between each attempt (sec) */
#define GPRS_APN_REGISTER_REATTEMPT 10

#define GPRS_DEBUG

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

#define GPRS_MAX_APN_LEN 32
struct gprs_context {
  uint8_t active; /* Context active? */
  char *pdptype; /* allowed values: "IP" and "IPV6" */
  char apn[GPRS_MAX_APN_LEN+1];
};

struct gprs_connection; /* Forward declaration */
typedef int (* gprs_callback_t)(struct gprs_connection *, int);
typedef void (* gprs_data_callback_t)(struct gprs_connection *gprsconn,
                                      void *callback_arg,
                                      const uint8_t *input_data_ptr,
                                      int input_data_len);
typedef void (* gprs_event_callback_t)(struct gprs_connection *gprsconn,
                                       void *callback_arg,
                                       gprs_conn_event_t event);

#define GPRS_MAX_CONNECTION 1
struct gprs_connection {
  struct gprs_context *context;
  uint8_t reserved;
  uint8_t connectionid;
  const char *proto;
  const char *ipaddr;
  uint16_t port;
  void *callback; 
  struct tcp_socket_gprs *socket;
  void *callback_arg;
  gprs_data_callback_t input_callback;
  gprs_event_callback_t event_callback;
  uint8_t *input_data_ptr;
  uint8_t *output_data_ptr;

  uint16_t input_data_len;
  uint16_t output_data_len;
};
#define GPRS_CONNECTION_RESERVED(GC) ((GC)->reserved)
#define GPRS_CONNECTION_RELEASE(GC) (GC)->reserved = 0
#define GPRS_CONNECTION_RESERVE(GC) (GC)->reserved = 1    

struct gprs_status {
  enum {
    GPRS_STATE_NONE,
    GPRS_STATE_IDLE,
    GPRS_STATE_REGISTERED,
    GPRS_STATE_ACTIVE
  } state;

#if NETSTACK_CONF_WITH_IPV6
  char ipaddr[sizeof("::ffff:255.255.255.255")];
#else
  char ipaddr[sizeof("255.255.255.255")];  
#endif /* NETSTACK_CONF_WITH_IPV6 */
  int8_t rssi;

  enum int8_t {
    GPRS_MODULE_UNNKOWN = -1,
    GPRS_MODULE_A6 = 6,
    GPRS_MODULE_A7,
    GPRS_MODULE_SIM7020E
  } module;

  double longi;
  double lat;
  double speed;
  double course;
};

struct gprs_statistics {
  unsigned int at_timeouts;
  unsigned int at_errors;
  unsigned int at_retries;
  unsigned int resets;
  unsigned int connections;     /* Connections succeeded */
  unsigned int connfailed;      /* Connections failed */
} gprs_statistics;

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

struct gprs_connection *
alloc_gprs_connection();

int
gprs_set_context(struct gprs_context *gcontext, char *pdptype, char *apn);

struct gprs_connection *
gprs_connection(struct gprs_connection *gprsconn, const char *proto, const char *ipaddr,
                uint16_t port, struct tcp_socket_gprs *socket);

void
gprs_send(struct gprs_connection *gprsconn);

#if 0
int
gprs_register(struct gprs_connection *gconn,
              struct tcp_socket_gprs *socket,
              void *callback);
#endif
int
gprs_register(struct gprs_connection *gconn,
              void *callback_arg,
              void *callback,
              gprs_data_callback_t input_callback,
              gprs_event_callback_t event_callback);

int
gprs_unregister(struct gprs_connection *gconn);

void
gprs_close(struct tcp_socket_gprs *socket);

struct gprs_status *gprs_status();
#endif /* GPRS_A6_H */
