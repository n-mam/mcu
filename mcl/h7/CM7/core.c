#include <stddef.h>
#include <stdint.h>

#include <openamp.h>
#include <stm32h7xx_hal.h>

void MX_USART3_UART_Init(void);
void stlink_uart(const char *format, ...);

#define TEST_BUF_SIZE 128
char buf[TEST_BUF_SIZE];
#define RPMSG_CHAN_NAME "openamp_demo"

// cm7
volatile int message_received_cm7;
static volatile int service_created;
static struct rpmsg_endpoint rp_endpoint;

void service_destroy_cb(struct rpmsg_endpoint *ept) {
    service_created = 0;
}

static int rpmsg_recv_callback(struct rpmsg_endpoint *ept, void *data,
        size_t len, uint32_t src, void *priv) {
    memset((void *)buf, 0, sizeof(char) * TEST_BUF_SIZE);
    strncpy((char *)buf, (char *)data, len);
    stlink_uart("CM7 rpmsg_recv_callback [%s]\n", buf);
    message_received_cm7 = 1;
    return 0;
}

void new_service_cb(struct rpmsg_device *rdev, const char *name, uint32_t dest) {
    int status = OPENAMP_create_endpoint(&rp_endpoint, name,
        dest, rpmsg_recv_callback, service_destroy_cb);
    if (status < 0) {
        stlink_uart("OPENAMP_create_endpoint failed %d\n, status");
    }
    service_created = 1;
    stlink_uart("CM7 service created\n");
}

int openamp_send_message(const char *data, int len) {
    return OPENAMP_send(&rp_endpoint, data, len);
}

void OpenAMP_init(void) {
    MAILBOX_Init();
    int status = MX_OPENAMP_Init(RPMSG_MASTER, new_service_cb);
    if (status != HAL_OK) {
        stlink_uart("MX_OPENAMP_Init failed cm7 %d\n", status); while(1);
        return;
    }
    rpmsg_init_ept(&rp_endpoint, RPMSG_CHAN_NAME, RPMSG_ADDR_ANY,
        RPMSG_ADDR_ANY, NULL, NULL);
    OPENAMP_Wait_EndPointready(&rp_endpoint);
	// Send first dummy message to enable the channel
	int message = 0x00;
	OPENAMP_send(&rp_endpoint, &message, sizeof(message));
}
