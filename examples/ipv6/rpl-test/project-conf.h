#ifndef __PROJECT_CONF_H__
#define __PROJECT_CONF_H__

#define RPL_CONF_WITH_STORING 1

#ifndef RPL_CONF_WITH_STORING
#undef RPL_NS_CONF_LINK_NUM
#define RPL_NS_CONF_LINK_NUM 40 /* Number of links maintained at the root. Can be set to 0 at non-root nodes. */
#undef UIP_CONF_MAX_ROUTES
#define UIP_CONF_MAX_ROUTES 0 /* No need for routes */
#undef RPL_CONF_MOP
#define RPL_CONF_MOP RPL_MOP_NON_STORING /* Mode of operation*/
#endif /* WITH_NON_STORING */


/* Netstack layers */
#if 0
#define NETSTACK_CONF_MAC         csma_driver
#define NETSTACK_CONF_RDC         contikimac_driver
#else
#define NETSTACK_CONF_RDC         nullrdc_driver
#define NETSTACK_CONF_MAC         nullmac_driver
#endif

#define NETSTACK_CONF_FRAMER      framer_802154

#undef UIP_CONF_TCP
#define UIP_CONF_TCP 0
#undef QUEUEBUF_CONF_NUM
#define QUEUEBUF_CONF_NUM 3
#undef RPL_NS_CONF_LINK_NUM
#define RPL_NS_CONF_LINK_NUM  8
#undef NBR_TABLE_CONF_MAX_NEIGHBORS
#define NBR_TABLE_CONF_MAX_NEIGHBORS 8
#undef UIP_CONF_ND6_SEND_NA
#define UIP_CONF_ND6_SEND_NA 0
#undef SICSLOWPAN_CONF_FRAG
#define SICSLOWPAN_CONF_FRAG 0

#define RPL_CONF_STATS 1

#define IEEE802154_CONF_PANID 0xFEED
#define CHANNEL_CONF_802_15_4 25


#endif /* __PROJECT_CONF_H__ */
