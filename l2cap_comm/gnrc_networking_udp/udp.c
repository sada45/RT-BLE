/*
 * Copyright (C) 2015-17 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       Demonstrating the sending and receiving of UDP data
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Martine Lenders <m.lenders@fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>
#include <inttypes.h>
#include "net/gnrc.h"
#include "net/gnrc/ipv6.h"
#include "net/gnrc/netif.h"
#include "net/gnrc/netif/hdr.h"
#include "net/gnrc/udp.h"
#include "net/gnrc/pktdump.h"
#include "timex.h"
#include "utlist.h"
#include "xtimer.h"
#include <sys/time.h>
#include "debug.h"

char str[2000];
static gnrc_netreg_entry_t server = GNRC_NETREG_ENTRY_INIT_PID(GNRC_NETREG_DEMUX_CTX_ALL,
                                                               KERNEL_PID_UNDEF);


static void send(char *addr_str, char *port_str, char *data, unsigned int num,
                 unsigned int delay)
{
   // puts("Enter send()!\n");
    gnrc_netif_t *netif = NULL;
    char *iface;
    uint16_t port;
    ipv6_addr_t addr;

    iface = ipv6_addr_split_iface(addr_str);//拆分IPv6地址+接口说明符，返回的是接口号，如果未指定接口，则为NULL。
    if ((!iface) && (gnrc_netif_numof() == 1)) {
        //puts("udp.c iface is null\n");//
        netif = gnrc_netif_iter(NULL);//遍历所有的网络接口，返回一个网络接口
    }
    else if (iface) {
        netif = gnrc_netif_get_by_pid(atoi(iface));//根据PID获取网络接口
    }

    /* parse destination address */
    if (ipv6_addr_from_str(&addr, addr_str) == NULL) {
        puts("Error: unable to parse destination address");
        return;
    }
    /* parse port */
    port = atoi(port_str);
    if (port == 0) {
        puts("Error: unable to parse destination port");
        return;
    }

    for (unsigned int i = 0; i < num; i++) {
        gnrc_pktsnip_t *payload, *udp, *ip;
        unsigned payload_size;
        /* allocate payload */
        DEBUG("Point 1\n");
        payload = gnrc_pktbuf_add(NULL, data, strlen(data), GNRC_NETTYPE_UNDEF);
        DEBUG("Point 2\n");
        if (payload == NULL) {
            puts("Error: unable to copy data to packet buffer");
            return;
        }
        /* store size for output */
        payload_size = (unsigned)payload->size;
        /* allocate UDP header, set source port := destination port */
        udp = gnrc_udp_hdr_build(payload, port, port);
        if (udp == NULL) {
            puts("Error: unable to allocate UDP header");
            gnrc_pktbuf_release(payload);
            return;
        }
        /* allocate IPv6 header */
        ip = gnrc_ipv6_hdr_build(udp, NULL, &addr);
        if (ip == NULL) {
            puts("Error: unable to allocate IPv6 header");
            gnrc_pktbuf_release(udp);
            return;
        }
        /* add netif header, if interface was given */
        if (netif != NULL) {
            gnrc_pktsnip_t *netif_hdr = gnrc_netif_hdr_build(NULL, 0, NULL, 0);

            gnrc_netif_hdr_set_netif(netif_hdr->data, netif);
            ip = gnrc_pkt_prepend(ip, netif_hdr);
        }
        /* send packet */
       // puts("Start to send!\n");//add by ljm
        if (!gnrc_netapi_dispatch_send(GNRC_NETTYPE_UDP, GNRC_NETREG_DEMUX_CTX_ALL, ip)) {
            puts("Error: unable to locate UDP thread");
            gnrc_pktbuf_release(ip);
            return;
        }
        /* access to `payload` was implicitly given up with the send operation above
         * => use temporary variable for output */
        printf("Success: sent %u byte(s) to [%s]:%u\n", payload_size, addr_str,
               port);
        puts("Success: sent!");//add by ljm
	struct timeval timestamp;
        gettimeofday(&timestamp, NULL);
        printf("current time:%ld\n", timestamp.tv_usec);
        xtimer_usleep(delay);
    }
}

static void start_server(char *port_str)
{
    uint16_t port;

    /* check if server is already running */
    if (server.target.pid != KERNEL_PID_UNDEF) {
        printf("Error: server already running on port %" PRIu32 "\n",
               server.demux_ctx);
        return;
    }
    /* parse port */
    port = atoi(port_str);
    if (port == 0) {
        puts("Error: invalid port specified");
        return;
    }
    /* start server (which means registering pktdump for the chosen port) */
    server.target.pid = gnrc_pktdump_pid;
    server.demux_ctx = (uint32_t)port;
    gnrc_netreg_register(GNRC_NETTYPE_UDP, &server);
    printf("Success: started UDP server on port %" PRIu16 "\n", port);
}

static void stop_server(void)
{
    /* check if server is running at all */
    if (server.target.pid == KERNEL_PID_UNDEF) {
        printf("Error: server was not running\n");
        return;
    }
    /* stop server */
    gnrc_netreg_unregister(GNRC_NETTYPE_UDP, &server);
    server.target.pid = KERNEL_PID_UNDEF;
    puts("Success: stopped UDP server");
}

//add by ljm
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       void get_random_str(char* random_str, int random_len)
{
    unsigned int i;
    for(i = 0; i < random_len; i++)
        random_str[i] = '1';
    random_str[i] = '\0';
}
static void dumb_delay(uint32_t delay)
{
    for (uint32_t i = 0; i < delay; i++) {
        __asm__("nop");
    }
}


void pid_cmd(int argc, char **argv)
{
    gnrc_netif_t *netif;
    for(int i=0;i<20;i++){
        if((netif==netif_get_by_id(i)))
    		printf("Pid %d, name is %s\n",i,thread_getname(i));
    }
}

int udp_cmd(int argc, char **argv)
{
//ljm  debug
    puts("udp.c:udp_cmd()\n");
    if (argc < 2) {
        printf("usage: %s [send|server]\n", argv[0]);
        return 1;
    }

    if (strcmp(argv[1], "send") == 0) {
        uint32_t num = 1;
        uint32_t delay = 1000000;
        if (argc < 5) {
            printf("usage: %s send <addr> <port> <data> [<num> [<delay in us>]]\n",
                   argv[0]);
            return 1;
        }
        if (argc > 5) {
            num = atoi(argv[5]);
        }
        if (argc > 6) {
            delay = atoi(argv[6]);
        }
        send(argv[2], argv[3], argv[4], num, delay);
    }
    //add by ljm
    else if (strcmp(argv[1], "test") == 0)
    {
	get_random_str(str,5);
        uint32_t num = 1;
        uint32_t delay = 1000000;
        if (argc > 4) {
            num = atoi(argv[5]);
        }
        if (argc > 5) {
            delay = atoi(argv[6]);
        }
        LED0_ON;
    	dumb_delay((500000UL));
    	LED0_OFF;
    	/*------------APP layer timestamp------------*/
    	struct timeval timestamp;
        gettimeofday(&timestamp, NULL);
        printf("APP layer start send: %ld\n", timestamp.tv_usec);

        send(argv[2], argv[3], str, num, delay);
        }
    else if (strcmp(argv[1], "server") == 0) {
        if (argc < 3) {
            printf("usage: %s server [start|stop]\n", argv[0]);
            return 1;
        }
        if (strcmp(argv[2], "start") == 0) {
            if (argc < 4) {
                printf("usage %s server start <port>\n", argv[0]);
                return 1;
            }
            start_server(argv[3]);
        }
        else if (strcmp(argv[2], "stop") == 0) {
            stop_server();
        }

    }
    else {
        puts("error: invalid command");
    }
    return 0;
}
