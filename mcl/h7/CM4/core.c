#include <stddef.h>
#include <stdint.h>

#include <openamp.h>
#include <stm32h7xx_hal.h>

void MX_USART3_UART_Init(void);
void stlink_uart(const char *format, ...);

#define TEST_BUF_SIZE 128
char buf[TEST_BUF_SIZE];
#define RPMSG_CHAN_NAME "openamp_demo"

// cm4
volatile int message_received_cm4;
static struct rpmsg_endpoint rp_endpoint;

static int rpmsg_recv_callback(struct rpmsg_endpoint *ept, void *data,
        size_t len, uint32_t src, void *priv) {
    memset((void *)buf, 0, sizeof(char) * TEST_BUF_SIZE);
    strncpy((char *)buf, (char *)data, len);
    stlink_uart("CM4 rpmsg_recv_callback [%s]\n", buf);
    message_received_cm4 = 1;
    return 0;
}

int openamp_send_message(const char *data, int len) {
    return OPENAMP_send(&rp_endpoint, data, len);
}

void OpenAMP_init(void) {
    MAILBOX_Init();
    int status = MX_OPENAMP_Init(RPMSG_REMOTE, NULL);
    if (status != HAL_OK) {
        stlink_uart("MX_OPENAMP_Init failed cm4 %d\n", status); while(1);
        return;
    }
    status = OPENAMP_create_endpoint(&rp_endpoint, RPMSG_CHAN_NAME,
        RPMSG_ADDR_ANY, rpmsg_recv_callback, NULL);
    if (status < 0) {
        stlink_uart("OPENAMP_create_endpoint failed %d\n", status); while(1);
        return;
    }
}
