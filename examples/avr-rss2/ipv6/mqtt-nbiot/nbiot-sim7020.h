#ifndef NBIOT_7020_H
#define NBIOT_7020_H


#define GPRS_MAX_SEND_LEN 1024
//#define GPRS_MAX_SEND_LEN 512
#define GPRS_MAX_RECV_LEN 1024

#define GRPS_APN_REGISTER_TIMEOUT 60
#define GRPS_APN_REGISTER_REATTEMPT 10

struct nbiot_status {
  enum {
    STATE_NONE, 
    STATE_IDLE,
    STATE_REGISTERED,
    STATE_ACTIVE
  } state;
#if NETSTACK_CONF_WITH_IPV6
  char ipaddr[sizeof("::ffff:255.255.255.255")];
#else
  char ipaddr[sizeof("255.255.255.255")];  
#endif /* NETSTACK_CONF_WITH_IPV6 */
  int8_t rssi;

  enum {
    MODULE_UNNKOWN = -1,
    MODULE_A6 = 6,
    MODULE_A7,
    MODULE_SIM7020E,
  } module;

  double longi;
  double lat;
  double speed;
  double course;
};

#endif /* NBIOT_7020_H */
