#ifndef SERVER_H
#define SERVER_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <string>
#include <functional>

#if defined (PICO)
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"
#endif
#include <fx/clock.h>

#define BUF_SIZE 2048
#define DEBUG_printf printf
#define UDP_PORT_REMOTE 4444
//#define USE_TCP

namespace tcp {

using TRecvCallback = std::function<void (uint8_t *, int)>;
TRecvCallback recvCallback = nullptr;

typedef struct TCP_SERVER_T_ {
    bool complete;
    char event[50];
    float value;
    int recv_len;
    int sent_len;
    struct tcp_pcb *client_pcb;
    struct tcp_pcb *server_pcb;
    uint8_t buffer_recv[BUF_SIZE];
    uint8_t buffer_sent[BUF_SIZE];
} TCP_SERVER_T;

struct server {

    ip_addr_t addr;
    std::string self_ip;
    struct udp_pcb *pcb;
    std::string wifi_ssid;
    TCP_SERVER_T *main_state;
    std::string wifi_password;
    uint32_t tcp_port = 4242;
    uint32_t udp_port = 4445;

    server() {}

    ~server() {
        #if defined (USE_TCP)
        tcp_server_close(main_state);
        #endif
    }

    void setRecvCallback(TRecvCallback cbk) {
        recvCallback = cbk;
    }

    static void udp_recv_callback(void *arg, udp_pcb *pcb, pbuf *p, const ip4_addr *addr, u16_t port) {
        if (p != NULL) {
            printf("[udp_recv_callback]\n");
            recvCallback((unsigned char *)p->payload, p->len);
            pbuf_free(p);
        }
    }

    void start(const std::string& ssid, const std::string& password) {
        wifi_ssid = ssid;
        wifi_password = password;
        if (cyw43_arch_init()){
            printf("failed to initialise\n");
            return;
        }
        cyw43_arch_enable_sta_mode();
        //cyw43_wifi_pm(&cyw43_state, CYW43_PERFORMANCE_PM);
        printf("connecting to wi-fi...\n");
        int failed = 1;
        while (failed) {
            failed = cyw43_arch_wifi_connect_timeout_ms(
                wifi_ssid.c_str(), wifi_password.c_str(), CYW43_AUTH_WPA2_AES_PSK, 60000);
            if (failed) {
                printf("failed to connect.\n");
                mcl::sleep_ms(500);
            }
        }
        self_ip = std::string(ip4addr_ntoa(netif_ip4_addr(netif_list)));
        printf("connected. self_ip %s\n", self_ip.c_str());

        #if defined (USE_TCP)
        printf("starting tcp server on port %u\n", udp_port);
        main_state = tcp_server_init();
        tcp_server_open(main_state, tcp_port);
        #else
        pcb = udp_new();
        err_t err = udp_bind(pcb, IP4_ADDR_ANY, udp_port);
        if (err != ERR_OK) {
            printf("error binding to port %d\n", udp_port);
            return;
        }
        udp_recv(pcb, udp_recv_callback, NULL);
        ipaddr_aton("255.255.255.255", &addr);
        printf("listenting for udp broadcast on %d", udp_port);
        #endif
    }

    void send_udp_data(const char *data, size_t len) {
        struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
        char *req = (char *) p->payload;
        memset(req, 0, len);
        memmove(req, data, len);
        err_t e = udp_sendto(pcb, p, &addr, UDP_PORT_REMOTE);
        pbuf_free(p);
        if (e != ERR_OK) {
            printf("failed to send udp packet. error: %d\n", e);
        } else {
            //printf("Sent packet\n");
        }
    }

    void send_data(const char *data, size_t len) {
        #if defined (USE_TCP)
        if (main_state->client_pcb) {
            if (tcp_write(main_state->client_pcb,
                    data, len, TCP_WRITE_FLAG_COPY) != ERR_OK) {
                DEBUG_printf("failed to send data\n");
            }
        }
        #else
        send_udp_data(data, len);
        #endif
    }

    #if defined (USE_TCP)
    static TCP_SERVER_T* tcp_server_init(void) {
        TCP_SERVER_T *state = (TCP_SERVER_T*)calloc(1, sizeof(TCP_SERVER_T));
        if (!state) {
            DEBUG_printf("failed to allocate state\n");
            return NULL;
        }
        return state;
    }

    static bool tcp_server_open(void *arg, uint32_t tcp_port){
        TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
        struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
        if (!pcb){
            DEBUG_printf("failed to create pcb\n");
            return false;
        }
        err_t err = tcp_bind(pcb, NULL, tcp_port);
        if (err){
                DEBUG_printf("failed to bind to port %u\n", tcp_port);
                return false;
        }
        state->server_pcb = tcp_listen_with_backlog(pcb, 1);
        if (!state->server_pcb){
            DEBUG_printf("failed to listen\n");
            if (pcb){
                tcp_close(pcb);
            }
            return false;
        }
        tcp_arg(state->server_pcb, state);
        tcp_accept(state->server_pcb, tcp_server_accept);
        return true;
    }

    static err_t tcp_server_close(void *arg) {
        TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
        err_t err = ERR_OK;
        if (state->client_pcb != NULL) {
            tcp_arg(state->client_pcb, NULL);
            tcp_poll(state->client_pcb, NULL, 0);
            tcp_sent(state->client_pcb, NULL);
            tcp_recv(state->client_pcb, NULL);
            tcp_err(state->client_pcb, NULL);
            err = tcp_close(state->client_pcb);
            if (err != ERR_OK) {
                DEBUG_printf("close failed %d, calling abort\n", err);
                tcp_abort(state->client_pcb);
                err = ERR_ABRT;
            }
            state->client_pcb = NULL;
        }
        if (state->server_pcb) {
            tcp_arg(state->server_pcb, NULL);
            tcp_close(state->server_pcb);
            state->server_pcb = NULL;
        }
        return err;
    }

    static err_t tcp_server_accept(void *arg, struct tcp_pcb *new_client_pcb, err_t err){
        TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
        if (err != ERR_OK || new_client_pcb == NULL){
            DEBUG_printf("Failure in accept\n");
            tcp_server_result(arg, err);
            return ERR_VAL;
        }
        DEBUG_printf("Client connected\n");
        state->client_pcb = new_client_pcb;
        tcp_arg(new_client_pcb, state);
        tcp_sent(new_client_pcb, tcp_server_sent);
        tcp_recv(new_client_pcb, tcp_server_recv);
        tcp_err(new_client_pcb, tcp_server_err);
        return ERR_OK;
    }

    static err_t tcp_server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len){
        TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
        //DEBUG_printf("tcp_server_sent %u bytes\n", len);
        state->sent_len += len;
        if (state->sent_len >= BUF_SIZE) {
            // We should get the data back from the client
            state->recv_len = 0;
            //DEBUG_printf("Waiting for buffer from client\n");
        }
        return ERR_OK;
    }

    static err_t tcp_server_result(void *arg, int status) {
        TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
        if (status == 0) {
            DEBUG_printf("test success\n");
        } else {
            DEBUG_printf("test failed %d\n", status);
        }
        state->complete = true;
        return 0;
    }

    static void tcp_server_err(void *arg, err_t err){
        if (err != ERR_ABRT){
            DEBUG_printf("tcp_client_err_fn %d\n", err);
            tcp_server_result(arg, err);
        }
    }

    static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err){
        TCP_SERVER_T *state = (TCP_SERVER_T*)arg;
        if (!p) {
            return tcp_server_result(arg, -1);
        }
        // this method is callback from lwIP, so cyw43_arch_lwip_begin is not required, however you
        // can use this method to cause an assertion in debug mode, if this method is called when
        // cyw43_arch_lwip_begin IS needed
        cyw43_arch_lwip_check();
        if (p->tot_len > 0) {
            //DEBUG_printf("tcp_server_recv %d/%d err %d\n", p->tot_len, state->recv_len, err);

            // Receive the buffer
            const uint16_t buffer_left = BUF_SIZE - state->recv_len;
            state->recv_len += pbuf_copy_partial(p, state->buffer_recv + state->recv_len,
                                             p->tot_len > buffer_left ? buffer_left : p->tot_len, 0);
            tcp_recved(tpcb, p->tot_len);
            recvCallback(state->buffer_recv, state->recv_len);
            state->recv_len =0;
        }
        pbuf_free(p);
        return ERR_OK;
    }
    #endif //USE_TCP
};
}

#endif
