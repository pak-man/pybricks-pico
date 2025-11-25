#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

// ============================================================================
// Pybricks Hub-specific lwIP Configuration
// ============================================================================

// Use polling mode (better for real-time motor control + BLE)
#define NO_SYS                          1
#define LWIP_SOCKET                     0
#define LWIP_NETCONN                    0
#define LWIP_NETIF_API                  0

// ============================================================================
// Memory configuration
// ============================================================================

#define MEM_LIBC_MALLOC                 0
#define MEM_ALIGNMENT                   4
#define MEM_SIZE                        8000        // Reduced for small system
#define MEMP_NUM_TCP_PCB                8           // Max TCP connections
#define MEMP_NUM_TCP_PCB_LISTEN         4
#define MEMP_NUM_UDP_PCB                8
#define MEMP_NUM_PBUF                   16
#define PBUF_POOL_SIZE                  16
#define LWIP_PBUF_FROM_CUSTOM_POOLS     0

// ============================================================================
// Protocol features
// ============================================================================

// Core protocols
#define LWIP_ARP                        1
#define LWIP_ETHERNET                   1
#define LWIP_ICMP                       1
#define LWIP_RAW                        1
#define LWIP_DHCP                       1
#define LWIP_AUTOIP                     0
#define LWIP_DNS                        1
#define LWIP_UDP                        1
#define LWIP_TCP                        1

// Advanced TCP
#define TCP_MSS                         1460
#define TCP_WND                         (4 * TCP_MSS)
#define TCP_SND_BUF                     (4 * TCP_MSS)
#define TCP_SND_QUEUELEN                ((4 * (TCP_SND_BUF) + (TCP_MSS - 1)) / (TCP_MSS))
#define LWIP_TCP_KEEPALIVE              1

// IGMP for multicast
#define LWIP_IGMP                       1

// SNTP for time sync
#define LWIP_SNTP                       1
#define SNTP_SET_SYSTEM_TIME_US(sec, us) do { } while(0)

// HTTP server
#define LWIP_HTTPD                      1
#define LWIP_HTTPD_CGI                  1
#define LWIP_HTTPD_SSI                  1
#define LWIP_HTTPD_DYNAMIC_HEADERS      1
#define LWIP_HTTPD_CUSTOM_FILES         1
#define HTTPD_SERVER_PORT               80

// ============================================================================
// Statistics and debugging
// ============================================================================

#define LWIP_STATS                      1
#define LWIP_STATS_DISPLAY              1

// Disable debug by default (enable for troubleshooting)
#define LWIP_DEBUG                      0
#define ETHARP_DEBUG                    LWIP_DBG_OFF
#define NETIF_DEBUG                     LWIP_DBG_OFF
#define PBUF_DEBUG                      LWIP_DBG_OFF
#define API_LIB_DEBUG                   LWIP_DBG_OFF
#define API_MSG_DEBUG                   LWIP_DBG_OFF
#define SOCKETS_DEBUG                   LWIP_DBG_OFF
#define ICMP_DEBUG                      LWIP_DBG_OFF
#define INET_DEBUG                      LWIP_DBG_OFF
#define IP_DEBUG                        LWIP_DBG_OFF
#define IP_REASS_DEBUG                  LWIP_DBG_OFF
#define RAW_DEBUG                       LWIP_DBG_OFF
#define MEM_DEBUG                       LWIP_DBG_OFF
#define MEMP_DEBUG                      LWIP_DBG_OFF
#define SYS_DEBUG                       LWIP_DBG_OFF
#define TCP_DEBUG                       LWIP_DBG_OFF
#define TCP_INPUT_DEBUG                 LWIP_DBG_OFF
#define TCP_OUTPUT_DEBUG                LWIP_DBG_OFF
#define TCP_RTO_DEBUG                   LWIP_DBG_OFF
#define TCP_CWND_DEBUG                  LWIP_DBG_OFF
#define TCP_WND_DEBUG                   LWIP_DBG_OFF
#define TCP_FR_DEBUG                    LWIP_DBG_OFF
#define TCP_QLEN_DEBUG                  LWIP_DBG_OFF
#define TCP_RST_DEBUG                   LWIP_DBG_OFF
#define UDP_DEBUG                       LWIP_DBG_OFF
#define TCPIP_DEBUG                     LWIP_DBG_OFF
#define SLIP_DEBUG                      LWIP_DBG_OFF
#define DHCP_DEBUG                      LWIP_DBG_OFF

// ============================================================================
// Performance tuning
// ============================================================================

#define LWIP_CHECKSUM_ON_COPY           1
#define LWIP_NETIF_TX_SINGLE_PBUF       1

// Reduced reassembly for memory savings
#define MEMP_NUM_REASSDATA              4
#define IP_REASS_MAX_PBUFS              10
#define IP_REASS_MAXAGE                 3

// ============================================================================
// Threading and callbacks (polling mode)
// ============================================================================

#define LWIP_TCPIP_CORE_LOCKING         0
#define LWIP_COMPAT_MUTEX               0
#define LWIP_ALLOW_MEM_FREE_FROM_OTHER_CONTEXT 0

// ============================================================================
// IPv6 (optional - enable if needed)
// ============================================================================

#define LWIP_IPV6                       0
#define LWIP_IPV6_DHCP6                 0

// ============================================================================
// Compatibility with Pico SDK
// ============================================================================

#ifndef PICO_CYW43_ARCH_POLL
#define PICO_CYW43_ARCH_POLL            1
#endif

#endif /* _LWIPOPTS_H */
