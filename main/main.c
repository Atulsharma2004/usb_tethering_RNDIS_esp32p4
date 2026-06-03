#if 1
/*
 * SPDX-FileCopyrightText: 2025-2026
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * ESP32-P4 USB host: QCX216 (Qualcomm VID 0x05C6) over USB
 *   1) Read USB descriptors / strings
 *   2) Open CDC-ACM (AT) and send Qualcomm RNDIS setup ATs
 *   3) Start USB RNDIS host driver, wait for DHCP (NAT 192.168.10.x)
 *   4) ICMP ping connectivity check
 *
 * Requires esp-iot-solution: iot_usbh_cdc, iot_usbh_rndis, iot_eth, iot_eth_netif_glue
 * Set sdkconfig: CONFIG_USB_HOST_ENABLE_ENUM_FILTER_CALLBACK=y (optional)
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <inttypes.h>

#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "usb/usb_host.h"
#include "usbh_helper.h"
#include "iot_usbh_cdc.h"
#include "iot_usbh_rndis.h"
#include "iot_eth.h"
#include "iot_eth_netif_glue.h"

#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "ping/ping_sock.h"
#include "lwip/inet.h"
#include "lwip/ip_addr.h"
#include "lwip/ip6_addr.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

/* QCX216 default USB IDs (usbd_proc_cb_custom.c) */
#define QCX216_USB_VID                  (0x05C6)
#define QCX216_USB_PID                  (0x9330)

/* Composite layout (usb_device.c): 0=RNDIS, 1=AT, 2=LOG, 3=PPP.
 * ESP iot_usbh_cdc cannot parse if1 (no CDC notification EP); AT works on if2. */
#define QCX216_USB_ITF_RNDIS            (0)
#define QCX216_USB_ITF_AT               (1)
#define QCX216_USB_ITF_LOG              (2)
#define QCX216_USB_ITF_PPP              (3)

#define TAG                             "QCX216-USB"

#define BIT_DEV_CONNECTED               (BIT0)
#define BIT_AT_SCRIPT_DONE              (BIT1)
#define BIT_ETH_LINK_UP                 (BIT2)
#define BIT_ETH_GOT_IP                  (BIT3)
#define BIT_PING_OK                     (BIT4)
#define BIT_PING_DONE                   (BIT5)

#define USB_HOST_TASK_STACK             (4096)
#define USB_HOST_TASK_PRIO              (5)
#define AT_TASK_STACK                   (6144)
#define AT_TASK_PRIO                    (6)
#define NET_TASK_STACK                  (8192)
#define NET_TASK_PRIO                   (4)

#define AT_LINE_MAX                     (384)
#define AT_RESP_MAX                     (1024)
#define AT_CMD_TIMEOUT_MS               (30000)
#define AT_INTER_CMD_DELAY_MS           (800)
#define RNDIS_WAIT_AFTER_BIND_MS        (45000)
#define PING_TARGET_V6                  "2001:4860:4860::8888"
#define PING_COUNT                      (4)
#define PING_INTERVAL_MS                (1000)
#define PING_TIMEOUT_MS                 (3000)
#define DHCP_WAIT_MS                    (20000)
#define TEST_DNS_V6                     "2606:4700:4700::1111"
#define TEST_DNS_PORT                   (53)
#define HTTP_PROBE_URL                  "http://httpbin.org/get"
#define AT_ITF_UNKNOWN                  (0xFF)
#define IPV6_GLOBAL_WAIT_MS             (25000)
/* QCX216 NAT defaults (AT$QCNETCFG="nat",1,"192.168.10.2") */
#define MODEM_NAT_GW                    "192.168.10.2"
#define MODEM_HOST_IP                   "192.168.10.3"
#define MODEM_HOST_MASK                 "255.255.255.0"

#ifndef CONFIG_APP_QUIT_PIN
#define CONFIG_APP_QUIT_PIN             (0)
#endif

/* iot_usbh_cdc v2 port config (same as component test_apps) */
#define USBH_CDC_PORT_RINGBUF_CONFIG(_dev_addr, _itf_num) \
    { \
        .dev_addr = (_dev_addr), \
        .itf_num = (_itf_num), \
        .in_ringbuf_size = 2048, \
        .out_ringbuf_size = 2048, \
        .in_transfer_buffer_size = 512, \
        .out_transfer_buffer_size = 512, \
        .cbs = { .closed = NULL, .recv_data = NULL, .notif_cb = NULL, .user_data = NULL }, \
        .flags = USBH_CDC_FLAGS_DISABLE_NOTIFICATION, \
    }

static EventGroupHandle_t s_events;
static SemaphoreHandle_t s_at_port_mux;

static usbh_cdc_port_handle_t s_at_port;
static uint8_t s_dev_addr;
static uint8_t s_at_itf_num = AT_ITF_UNKNOWN;
static const usb_config_desc_t *s_active_cfg;
static iot_eth_driver_t *s_rndis_driver;
static iot_eth_handle_t s_eth;
static esp_netif_t *s_eth_netif;

static bool s_rndis_installed;
static bool s_at_script_started;
static bool s_net_checks_done;
static bool s_static_ip_applied;
static bool s_modem_ipv6_valid;
static esp_ip6_addr_t s_modem_ipv6;

/* Must be static: usbh_cdc_register_dev_event_cb() keeps this pointer forever. */
static const usb_device_match_id_t s_qcx_match[] = {
    {
        .match_flags = USB_DEVICE_ID_MATCH_VENDOR | USB_DEVICE_ID_MATCH_PRODUCT,
        .idVendor = QCX216_USB_VID,
        .idProduct = USB_DEVICE_PRODUCT_ANY,
    },
    {0},
};

/* -------------------------------------------------------------------------- */
/* USB descriptor logging                                                     */
/* -------------------------------------------------------------------------- */

static void log_device_identity(const usb_device_desc_t *desc,
                                const usb_config_desc_t *cfg)
{
    ESP_LOGI(TAG,
             "QCX216 attached: VID=0x%04" PRIX16 " PID=0x%04" PRIX16
             " class=0x%02x configs=%u",
             desc->idVendor,
             desc->idProduct,
             desc->bDeviceClass,
             desc->bNumConfigurations);
    ESP_LOGI(TAG,
             "String idx: mfg=%u product=%u serial=%u",
             desc->iManufacturer,
             desc->iProduct,
             desc->iSerialNumber);

    if (cfg != NULL) {
        ESP_LOGI(TAG,
                 "Active config: value=%u num_if=%u total_len=%u",
                 cfg->bConfigurationValue,
                 cfg->bNumInterfaces,
                 cfg->wTotalLength);
        for (int i = 0; i < cfg->bNumInterfaces; i++) {
            const usb_intf_desc_t *itf = usb_parse_interface_descriptor(cfg, i, 0, NULL);
            if (itf != NULL) {
                ESP_LOGI(TAG,
                         "  if[%d] num=%u class=0x%02x sub=0x%02x eps=%u",
                         i,
                         itf->bInterfaceNumber,
                         itf->bInterfaceClass,
                         itf->bInterfaceSubClass,
                         itf->bNumEndpoints);
            }
        }
    }
}

/* -------------------------------------------------------------------------- */
/* CDC-ACM AT port (interface 1 on QCX216 composite)                          */
/* -------------------------------------------------------------------------- */

/* First CDC-ACM control interface (class 0x02 / subclass 0x02) — matches itf2 on QCX216. */
static int find_qcx216_cdc_at_interface(const usb_config_desc_t *cfg, uint8_t *itf_num)
{
    if (cfg == NULL || itf_num == NULL) {
        return -1;
    }

    for (int i = 0; i < cfg->bNumInterfaces; i++) {
        const usb_intf_desc_t *itf = usb_parse_interface_descriptor(cfg, i, 0, NULL);
        if (itf == NULL) {
            continue;
        }
        if (itf->bInterfaceClass == 0xE0) {
            continue;
        }
        if (itf->bInterfaceClass == USB_CLASS_COMM && itf->bInterfaceSubClass == 0x02) {
            *itf_num = itf->bInterfaceNumber;
            return 0;
        }
    }
    return -1;
}

static esp_err_t open_at_cdc_port(void)
{
    if (s_at_port != NULL) {
        return ESP_OK;
    }
    if (s_dev_addr == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_at_itf_num == AT_ITF_UNKNOWN) {
        ESP_LOGE(TAG, "AT interface not selected — connect QCX216 first");
        return ESP_ERR_INVALID_STATE;
    }

    usbh_cdc_port_config_t port_cfg = USBH_CDC_PORT_RINGBUF_CONFIG(s_dev_addr, s_at_itf_num);
    ESP_LOGI(TAG, "Opening AT CDC: dev_addr=%u itf=%u (locked)", s_dev_addr, s_at_itf_num);
    esp_err_t err = usbh_cdc_port_open(&port_cfg, &s_at_port);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "AT open itf=%u failed: %s", s_at_itf_num, esp_err_to_name(err));
        s_at_port = NULL;
    }
    return err;
}

static esp_err_t at_port_write(const char *cmd)
{
    if (s_at_port == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    size_t len = strlen(cmd);
    ESP_LOGI(TAG, "AT TX: %s", cmd);
    return usbh_cdc_write_bytes(s_at_port, (const uint8_t *)cmd, len, pdMS_TO_TICKS(5000));
}

static bool resp_contains(const char *resp, const char *needle)
{
    return (resp != NULL) && (strstr(resp, needle) != NULL);
}

static esp_err_t at_port_read_response(char *buf, size_t buf_len, uint32_t timeout_ms)
{
    if (s_at_port == NULL || buf == NULL || buf_len < 2) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t total = 0;
    buf[0] = '\0';
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);

    while (xTaskGetTickCount() < deadline) {
        size_t rx_len = buf_len - total - 1;
        esp_err_t err = usbh_cdc_read_bytes(s_at_port,
                                            (uint8_t *)buf + total,
                                            &rx_len,
                                            pdMS_TO_TICKS(200));
        if (err == ESP_OK && rx_len > 0) {
            total += rx_len;
            buf[total] = '\0';
            if (resp_contains(buf, "OK") || resp_contains(buf, "ERROR") ||
                resp_contains(buf, "CME ERROR") || resp_contains(buf, "$QCNETDEVCTL:")) {
                return ESP_OK;
            }
        }
    }
    return (total > 0) ? ESP_OK : ESP_ERR_TIMEOUT;
}

static void at_drain_rx(void)
{
    /* Flush only — reading with 0 timeout logs "cdc read failed" when the ringbuf is empty. */
    if (s_at_port != NULL) {
        usbh_cdc_flush_rx_buffer(s_at_port);
    }
}

static esp_err_t at_send_expect_ok(const char *cmd, char *resp, size_t resp_len)
{
    char line[AT_LINE_MAX];
    snprintf(line, sizeof(line), "%s\r\n", cmd);

    if (xSemaphoreTake(s_at_port_mux, pdMS_TO_TICKS(5000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    at_drain_rx();
    ESP_LOGI(TAG, ">>> %s", cmd);

    esp_err_t err = at_port_write(line);
    if (err == ESP_OK) {
        err = at_port_read_response(resp, resp_len, AT_CMD_TIMEOUT_MS);
    }

    xSemaphoreGive(s_at_port_mux);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "<<< [%s] TIMEOUT (no response)", cmd);
        return err;
    }

    ESP_LOGI(TAG, "<<< [%s] resp:\n%s", cmd, resp);
    if (resp_contains(resp, "ERROR") || resp_contains(resp, "CME ERROR")) {
        ESP_LOGE(TAG, "<<< [%s] FAIL", cmd);
        return ESP_FAIL;
    }
    if (!resp_contains(resp, "OK")) {
        ESP_LOGW(TAG, "<<< [%s] no OK in response", cmd);
    } else {
        ESP_LOGI(TAG, "<<< [%s] OK", cmd);
    }
    return ESP_OK;
}

static bool parse_ipv6_quoted_field(const char *resp, esp_ip6_addr_t *out)
{
    const char *p = resp;

    if (resp == NULL || out == NULL) {
        return false;
    }

    while ((p = strchr(p, '"')) != NULL) {
        p++;
        const char *end = strchr(p, '"');
        if (end == NULL) {
            break;
        }
        if (memchr(p, ':', (size_t)(end - p)) != NULL) {
            char tmp[64];
            size_t len = (size_t)(end - p);
            if (len >= sizeof(tmp)) {
                p = end + 1;
                continue;
            }
            memcpy(tmp, p, len);
            tmp[len] = '\0';
            ip6_addr_t ip6 = {0};
            if (ip6addr_aton(tmp, &ip6)) {
                memcpy(out, &ip6, sizeof(esp_ip6_addr_t));
                return true;
            }
        }
        p = end + 1;
    }
    return false;
}

static esp_err_t at_query_modem_ipv6(char *resp, size_t resp_len)
{
    vTaskDelay(pdMS_TO_TICKS(1500));

    esp_err_t err = at_send_expect_ok("AT+CGPADDR=1", resp, resp_len);
    if (err != ESP_OK) {
        return err;
    }

    if (parse_ipv6_quoted_field(resp, &s_modem_ipv6)) {
        s_modem_ipv6_valid = true;
        ESP_LOGI(TAG, "Modem PDP IPv6 (AT+CGPADDR): " IPV6STR, IPV62STR(s_modem_ipv6));
    } else {
        ESP_LOGW(TAG, "No IPv6 in +CGPADDR response");
    }
    return ESP_OK;
}

static esp_err_t run_qcx216_at_script(void)
{
    char resp[AT_RESP_MAX];
    static const char *const k_cmds[] = {
        "AT",
        "AT+CGDCONT=1,\"IPV4V6\",\"airtelmeterv6\"",
        "AT$QCNETCFG=\"nat\",1,\"192.168.10.2\"",
        "AT$QCPCFG=\"usbCtrl\",0",
        "AT$QCPCFG=\"usbNet\",0",
        "AT+CGACT=1,1",
        "AT$QCNETDEVCTL=3,1,1",
    };

    ESP_LOGI(TAG, "Sending QCX216 USB/RNDIS AT script...");

    for (size_t i = 0; i < sizeof(k_cmds) / sizeof(k_cmds[0]); i++) {
        ESP_LOGI(TAG, "AT step %u/%u", (unsigned)(i + 1), (unsigned)(sizeof(k_cmds) / sizeof(k_cmds[0])));
        esp_err_t err = at_send_expect_ok(k_cmds[i], resp, sizeof(resp));
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "AT script STOP at step %u: %s", (unsigned)(i + 1), k_cmds[i]);
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(AT_INTER_CMD_DELAY_MS));
    }
    ESP_LOGI(TAG, "AT script: all commands got OK");

    if (resp_contains(resp, "$QCNETDEVCTL: 1") || resp_contains(resp, "$QCNETDEVCTL:1")) {
        ESP_LOGI(TAG, "Modem reported RNDIS/ETH bind OK");
    }

    at_query_modem_ipv6(resp, sizeof(resp));

    if (s_at_port != NULL) {
        usbh_cdc_port_close(s_at_port);
        s_at_port = NULL;
        ESP_LOGI(TAG, "AT port closed (RNDIS uses itf%u)", QCX216_USB_ITF_RNDIS);
    }
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/* USB RNDIS host driver                                                        */
/* -------------------------------------------------------------------------- */

static void install_usb_rndis_host(void)
{
    if (s_rndis_installed) {
        return;
    }

    usb_device_match_id_t *match = calloc(2, sizeof(usb_device_match_id_t));
    if (match == NULL) {
        ESP_LOGE(TAG, "Out of memory for RNDIS match list");
        return;
    }

    match[0].match_flags = USB_DEVICE_ID_MATCH_VENDOR | USB_DEVICE_ID_MATCH_PRODUCT;
    match[0].idVendor = QCX216_USB_VID;
    match[0].idProduct = USB_DEVICE_PRODUCT_ANY;

    iot_usbh_rndis_config_t rndis_cfg = {
        .match_id_list = match,
    };

    esp_err_t err = iot_eth_new_usb_rndis(&rndis_cfg, &s_rndis_driver);
    if (err != ESP_OK || s_rndis_driver == NULL) {
        ESP_LOGE(TAG, "iot_eth_new_usb_rndis failed: %s", esp_err_to_name(err));
        free(match);
        return;
    }

    iot_eth_config_t eth_cfg = {
        .driver = s_rndis_driver,
        .stack_input = NULL,
    };
    ESP_ERROR_CHECK(iot_eth_install(&eth_cfg, &s_eth));

    esp_netif_inherent_config_t netif_cfg = ESP_NETIF_INHERENT_DEFAULT_ETH();
    netif_cfg.if_key = "qcx216";
    netif_cfg.if_desc = "qcx216_rndis";
    netif_cfg.route_prio = 10;

    s_eth_netif = esp_netif_new(&(esp_netif_config_t){
        .base = &netif_cfg,
        .driver = NULL,
        .stack = ESP_NETIF_NETSTACK_DEFAULT_ETH,
    });
    assert(s_eth_netif);

    iot_eth_netif_glue_handle_t glue = iot_eth_new_netif_glue(s_eth);
    assert(glue);
    esp_netif_attach(s_eth_netif, glue);
    esp_netif_set_default_netif(s_eth_netif);
    ESP_ERROR_CHECK(iot_eth_start(s_eth));

    s_rndis_installed = true;
    ESP_LOGI(TAG, "USB RNDIS host driver started — waiting for link + DHCP");
}

/* -------------------------------------------------------------------------- */
/* USB host enum filter (config 1 for QCX216 composite)                         */
/* -------------------------------------------------------------------------- */

#if CONFIG_USB_HOST_ENABLE_ENUM_FILTER_CALLBACK

static bool usb_host_enum_filter_cb(const usb_device_desc_t *dev_desc,
                                    uint8_t *bConfigurationValue)
{
    *bConfigurationValue = 1;
    if (dev_desc->bNumConfigurations > 1) {
        *bConfigurationValue = 1;
    }
    ESP_LOGI(TAG,
             "Enumerate VID=0x%04" PRIX16 " PID=0x%04" PRIX16 " config=%u",
             dev_desc->idVendor,
             dev_desc->idProduct,
             *bConfigurationValue);
    return true;
}

static void usb_lib_task(void *arg)
{
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
        .enum_filter_cb = usb_host_enum_filter_cb,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));
    xTaskNotifyGive((TaskHandle_t)arg);

    bool has_clients = true;
    while (has_clients) {
        uint32_t event_flags;
        ESP_ERROR_CHECK(usb_host_lib_handle_events(portMAX_DELAY, &event_flags));
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            if (usb_host_device_free_all() == ESP_OK) {
                has_clients = false;
            }
        }
        if ((event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) && !has_clients) {
            break;
        }
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    usb_host_uninstall();
    vTaskDelete(NULL);
}

#endif /* CONFIG_USB_HOST_ENABLE_ENUM_FILTER_CALLBACK */

/* -------------------------------------------------------------------------- */
/* CDC device connect → AT script → RNDIS                                       */
/* -------------------------------------------------------------------------- */

static void kick_at_setup(uint8_t dev_addr);

static void at_setup_task(void *arg)
{
    (void)arg;

    for (;;) {
        xEventGroupWaitBits(s_events, BIT_DEV_CONNECTED, pdFALSE, pdTRUE, portMAX_DELAY);

        if (s_at_script_started) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        s_at_script_started = true;

        /* AT port is opened in the CDC connect callback before RNDIS runs. */
        vTaskDelay(pdMS_TO_TICKS(50));

        if (s_dev_addr == 0) {
            ESP_LOGE(TAG, "USB device address missing");
            s_at_script_started = false;
            continue;
        }

        if (open_at_cdc_port() != ESP_OK) {
            ESP_LOGE(TAG, "Cannot open AT port");
            s_at_script_started = false;
            continue;
        }

        if (run_qcx216_at_script() != ESP_OK) {
            ESP_LOGE(TAG, "AT script failed — fix APN/SIM and retry");
            s_at_script_started = false;
            continue;
        }

        xEventGroupSetBits(s_events, BIT_AT_SCRIPT_DONE);

        ESP_LOGI(TAG, "Waiting up to 30s for RNDIS link after AT bind...");
        EventBits_t link = xEventGroupWaitBits(s_events,
                                               BIT_ETH_LINK_UP,
                                               pdFALSE,
                                               pdTRUE,
                                               pdMS_TO_TICKS(30000));
        if (!(link & BIT_ETH_LINK_UP)) {
            ESP_LOGW(TAG, "No IOT_ETH link-up yet (modem may need more time)");
        }
    }
}

static void kick_at_setup(uint8_t dev_addr)
{
    if (dev_addr == 0) {
        return;
    }
    s_dev_addr = dev_addr;
    ESP_LOGI(TAG, "Kick AT setup for USB dev_addr=%u", s_dev_addr);
    xEventGroupSetBits(s_events, BIT_DEV_CONNECTED);
}

static void qcx216_dev_event_cb(usbh_cdc_device_event_t event,
                                usbh_cdc_device_event_data_t *event_data,
                                void *user_ctx)
{
    (void)user_ctx;

    if (event == CDC_HOST_DEVICE_EVENT_CONNECTED) {
        const usb_config_desc_t *cfg = event_data->new_dev.active_config_desc;
        const usb_device_desc_t *desc = event_data->new_dev.device_desc;

        if (desc == NULL || desc->idVendor != QCX216_USB_VID || desc->idProduct != QCX216_USB_PID) {
            ESP_LOGW(TAG, "Ignored USB device (not QCX216 0x%04X:0x%04X)",
                     desc ? desc->idVendor : 0,
                     desc ? desc->idProduct : 0);
            return;
        }

        s_dev_addr = event_data->new_dev.dev_addr;
        s_at_script_started = false;
        log_device_identity(desc, cfg);

        if (cfg != NULL && find_qcx216_cdc_at_interface(cfg, &s_at_itf_num) == 0) {
            ESP_LOGI(TAG, "QCX216 AT CDC interface detected: itf=%u", s_at_itf_num);
        } else {
            s_at_itf_num = QCX216_USB_ITF_LOG;
            ESP_LOGW(TAG, "CDC scan fallback: AT on itf=%u", s_at_itf_num);
        }

        /* Open AT before the RNDIS handler (register this callback after install_usb_rndis). */
        if (open_at_cdc_port() != ESP_OK) {
            ESP_LOGW(TAG, "Early AT open failed; at_setup will retry");
        }

        ESP_LOGI(TAG,
                 "CDC connect: dev_addr=%u AT=itf%u RNDIS=itf%u",
                 s_dev_addr,
                 s_at_itf_num,
                 QCX216_USB_ITF_RNDIS);
        kick_at_setup(s_dev_addr);
        return;
    }

    if (event == CDC_HOST_DEVICE_EVENT_DISCONNECTED) {
        ESP_LOGW(TAG, "QCX216 USB disconnected (addr=%u)",
                 event_data->dev_gone.dev_addr);
        if (s_at_port != NULL) {
            usbh_cdc_port_close(s_at_port);
            s_at_port = NULL;
        }
        s_dev_addr = 0;
        s_at_itf_num = AT_ITF_UNKNOWN;
        s_active_cfg = NULL;
        s_at_script_started = false;
        s_net_checks_done = false;
        s_static_ip_applied = false;
        s_modem_ipv6_valid = false;
        memset(&s_modem_ipv6, 0, sizeof(s_modem_ipv6));
        xEventGroupClearBits(s_events,
                            BIT_DEV_CONNECTED | BIT_AT_SCRIPT_DONE |
                            BIT_ETH_LINK_UP | BIT_ETH_GOT_IP | BIT_PING_OK | BIT_PING_DONE);
        if (s_eth) {
            iot_eth_stop(s_eth);
        }
    }
}

/* -------------------------------------------------------------------------- */
/* Ethernet / IP events + ping                                                  */
/* -------------------------------------------------------------------------- */

typedef struct {
    char target[48];
    uint32_t rtt_min_ms;
    uint32_t rtt_max_ms;
    uint64_t rtt_sum_ms;
} ping_stats_t;

static ping_stats_t s_ping_stats;

static void ping_stats_reset(const char *target)
{
    memset(&s_ping_stats, 0, sizeof(s_ping_stats));
    s_ping_stats.rtt_min_ms = UINT32_MAX;
    if (target != NULL) {
        snprintf(s_ping_stats.target, sizeof(s_ping_stats.target), "%s", target);
    }
}

static void ping_stats_update_rtt(uint32_t rtt_ms)
{
    if (rtt_ms < s_ping_stats.rtt_min_ms) {
        s_ping_stats.rtt_min_ms = rtt_ms;
    }
    if (rtt_ms > s_ping_stats.rtt_max_ms) {
        s_ping_stats.rtt_max_ms = rtt_ms;
    }
    s_ping_stats.rtt_sum_ms += rtt_ms;
}

static void ping_log_summary(esp_ping_handle_t hdl)
{
    uint32_t transmitted = 0;
    uint32_t received = 0;
    uint32_t duration_ms = 0;

    esp_ping_get_profile(hdl, ESP_PING_PROF_REQUEST, &transmitted, sizeof(transmitted));
    esp_ping_get_profile(hdl, ESP_PING_PROF_REPLY, &received, sizeof(received));
    esp_ping_get_profile(hdl, ESP_PING_PROF_DURATION, &duration_ms, sizeof(duration_ms));

    uint32_t lost = (transmitted > received) ? (transmitted - received) : 0;
    float loss_pct = (transmitted > 0) ? (100.0f * (float)lost / (float)transmitted) : 0.0f;

    ESP_LOGI(TAG, "========== PING %s ==========", s_ping_stats.target);
    ESP_LOGI(TAG,
             "  packets: transmitted=%" PRIu32 " received=%" PRIu32
             " lost=%" PRIu32 " (%.1f%% loss)",
             transmitted,
             received,
             lost,
             loss_pct);
    ESP_LOGI(TAG,
             "  result:  success=%" PRIu32 " fail=%" PRIu32,
             received,
             lost);

    if (received > 0 && s_ping_stats.rtt_min_ms != UINT32_MAX) {
        uint32_t avg_ms = (uint32_t)(s_ping_stats.rtt_sum_ms / received);
        ESP_LOGI(TAG,
                 "  rtt (ms): min=%" PRIu32 " avg=%" PRIu32 " max=%" PRIu32,
                 s_ping_stats.rtt_min_ms,
                 avg_ms,
                 s_ping_stats.rtt_max_ms);
    } else if (received == 0) {
        ESP_LOGW(TAG, "  rtt (ms): n/a (no replies)");
    }

    ESP_LOGI(TAG, "  session duration: %" PRIu32 " ms", duration_ms);
    ESP_LOGI(TAG, "================================");
}

static void on_ping_success(esp_ping_handle_t hdl, void *args)
{
    (void)args;

    uint16_t seqno = 0;
    uint32_t timegap_ms = 0;
    uint32_t recv_len = 0;
    uint8_t ttl = 0;

    esp_ping_get_profile(hdl, ESP_PING_PROF_SEQNO, &seqno, sizeof(seqno));
    esp_ping_get_profile(hdl, ESP_PING_PROF_TIMEGAP, &timegap_ms, sizeof(timegap_ms));
    esp_ping_get_profile(hdl, ESP_PING_PROF_SIZE, &recv_len, sizeof(recv_len));
    esp_ping_get_profile(hdl, ESP_PING_PROF_TTL, &ttl, sizeof(ttl));

    ping_stats_update_rtt(timegap_ms);

    ESP_LOGI(TAG,
             "  reply from %s: icmp_seq=%u bytes=%" PRIu32 " ttl=%u time=%" PRIu32 " ms",
             s_ping_stats.target,
             (unsigned)seqno,
             recv_len,
             (unsigned)ttl,
             timegap_ms);

    xEventGroupSetBits(s_events, BIT_PING_OK);
}

static void on_ping_timeout(esp_ping_handle_t hdl, void *args)
{
    (void)args;

    uint16_t seqno = 0;
    esp_ping_get_profile(hdl, ESP_PING_PROF_SEQNO, &seqno, sizeof(seqno));
    ESP_LOGW(TAG,
             "  request timeout for icmp_seq=%u (%s)",
             (unsigned)seqno,
             s_ping_stats.target);
}

static void on_ping_end(esp_ping_handle_t hdl, void *args)
{
    (void)args;
    ping_log_summary(hdl);
    xEventGroupSetBits(s_events, BIT_PING_DONE);
}

static void netif_log_ipv6_compare(void)
{
    esp_ip6_addr_t slaac = {0};

    if (s_eth_netif == NULL) {
        return;
    }
    if (esp_netif_get_ip6_global(s_eth_netif, &slaac) != ESP_OK) {
        ESP_LOGW(TAG, "No SLAAC global IPv6 on netif yet");
        return;
    }

    ESP_LOGI(TAG, "RNDIS SLAAC IPv6: " IPV6STR " (from host MAC EUI-64)", IPV62STR(slaac));
    if (s_modem_ipv6_valid) {
        ESP_LOGI(TAG, "Modem SIM IPv6:   " IPV6STR " (from AT+CGPADDR)", IPV62STR(s_modem_ipv6));
        if (memcmp(&slaac, &s_modem_ipv6, sizeof(slaac)) != 0) {
            ESP_LOGW(TAG,
                     "SLAAC host address differs from modem PDP — expected on RNDIS "
                     "(host uses USB MAC; modem WAN may use ::2)");
        }
    }
}

static void netif_apply_modem_ipv6_preferred(void)
{
    if (s_eth_netif == NULL || !s_modem_ipv6_valid) {
        return;
    }

    esp_err_t err = esp_netif_add_ip6_address(s_eth_netif, s_modem_ipv6, true);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Preferred IPv6 set to modem PDP: " IPV6STR, IPV62STR(s_modem_ipv6));
    } else {
        ESP_LOGW(TAG, "esp_netif_add_ip6_address failed: %s", esp_err_to_name(err));
    }
}

static bool wait_global_ipv6(uint32_t timeout_ms)
{
    if (s_eth_netif == NULL) {
        return false;
    }

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    while (xTaskGetTickCount() < deadline) {
        esp_ip6_addr_t addr = {0};
        if (esp_netif_get_ip6_global(s_eth_netif, &addr) == ESP_OK) {
            netif_log_ipv6_compare();
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    ESP_LOGW(TAG, "No global IPv6 after %lu ms", (unsigned long)timeout_ms);
    return false;
}

static esp_err_t ping_internet_check(void)
{
    esp_ping_config_t cfg = ESP_PING_DEFAULT_CONFIG();
    ip_addr_t target;
    if (!ipaddr_aton(PING_TARGET_V6, &target)) {
        return ESP_ERR_INVALID_ARG;
    }
    cfg.target_addr = target;
    cfg.count = PING_COUNT;
    cfg.interval_ms = PING_INTERVAL_MS;
    cfg.timeout_ms = PING_TIMEOUT_MS;
    cfg.task_stack_size = 4096;
    cfg.task_prio = 2;

    ping_stats_reset(PING_TARGET_V6);
    ESP_LOGI(TAG,
             "PING %s: %" PRIu32 " packets, %" PRIu32 " ms interval, %" PRIu32 " ms timeout",
             PING_TARGET_V6,
             cfg.count,
             cfg.interval_ms,
             cfg.timeout_ms);

    esp_ping_callbacks_t cbs = {
        .on_ping_success = on_ping_success,
        .on_ping_timeout = on_ping_timeout,
        .on_ping_end = on_ping_end,
    };

    esp_ping_handle_t ping;
    ESP_RETURN_ON_ERROR(esp_ping_new_session(&cfg, &cbs, &ping), TAG, "ping session");

    xEventGroupClearBits(s_events, BIT_PING_OK | BIT_PING_DONE);
    ESP_RETURN_ON_ERROR(esp_ping_start(ping), TAG, "ping start");

    uint32_t wait_ms = (PING_COUNT + 2) * (PING_INTERVAL_MS + PING_TIMEOUT_MS);
    EventBits_t bits = xEventGroupWaitBits(s_events,
                                           BIT_PING_DONE,
                                           pdTRUE,
                                           pdFALSE,
                                           pdMS_TO_TICKS(wait_ms));

    esp_ping_stop(ping);

    if (!(bits & BIT_PING_DONE)) {
        ESP_LOGW(TAG, "Ping session timed out before on_ping_end — printing partial stats");
        ping_log_summary(ping);
    }

    esp_ping_delete_session(ping);

    return (bits & BIT_PING_OK) ? ESP_OK : ESP_FAIL;
}

static void netif_log_ip(void)
{
    esp_netif_ip_info_t ip = {0};
    if (s_eth_netif != NULL && esp_netif_get_ip_info(s_eth_netif, &ip) == ESP_OK) {
        ESP_LOGI(TAG,
                 "Netif IP: " IPSTR " mask " IPSTR " gw " IPSTR,
                 IP2STR(&ip.ip),
                 IP2STR(&ip.netmask),
                 IP2STR(&ip.gw));
    }
}

static void netif_apply_modem_nat_static_ip(void)
{
    if (s_eth_netif == NULL || s_static_ip_applied) {
        return;
    }

    esp_netif_ip_info_t ip = {0};
    ip4addr_aton(MODEM_HOST_IP, (ip4_addr_t *)&ip.ip);
    ip4addr_aton(MODEM_NAT_GW, (ip4_addr_t *)&ip.gw);
    ip4addr_aton(MODEM_HOST_MASK, (ip4_addr_t *)&ip.netmask);

    esp_netif_dhcpc_stop(s_eth_netif);
    ESP_ERROR_CHECK(esp_netif_set_ip_info(s_eth_netif, &ip));
    s_static_ip_applied = true;
    xEventGroupSetBits(s_events, BIT_ETH_GOT_IP);
    ESP_LOGI(TAG,
             "Applied static NAT IP (no DHCP): host " MODEM_HOST_IP " gw " MODEM_NAT_GW);
    netif_log_ip();
}

static esp_err_t wait_for_ip_or_static(void)
{
    EventBits_t bits = xEventGroupWaitBits(s_events,
                                           BIT_ETH_GOT_IP,
                                           pdFALSE,
                                           pdTRUE,
                                           pdMS_TO_TICKS(DHCP_WAIT_MS));
    if (bits & BIT_ETH_GOT_IP) {
        netif_log_ip();
        return ESP_OK;
    }

    ESP_LOGW(TAG, "No DHCP in %d ms — applying static " MODEM_HOST_IP, DHCP_WAIT_MS);
    netif_apply_modem_nat_static_ip();
    return ESP_OK;
}

static esp_err_t tcp_connect_probe_v6(const char *host, uint16_t port)
{
    struct sockaddr_in6 sa = {0};
    sa.sin6_family = AF_INET6;
    sa.sin6_port = htons(port);

    if (inet_pton(AF_INET6, host, &sa.sin6_addr) != 1) {
        ESP_LOGE(TAG, "TCP invalid IPv6 address: %s", host);
        return ESP_ERR_INVALID_ARG;
    }

    int sock = socket(AF_INET6, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) {
        ESP_LOGE(TAG, "TCP socket() failed errno=%d", errno);
        return ESP_FAIL;
    }

    struct timeval tv = {.tv_sec = 15, .tv_usec = 0};
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    int err = connect(sock, (struct sockaddr *)&sa, sizeof(sa));
    if (err != 0) {
        ESP_LOGE(TAG, "TCP connect [%s]:%u failed errno=%d", host, port, errno);
        close(sock);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "TCP connect [%s]:%u OK", host, port);
    close(sock);
    return ESP_OK;
}

static esp_err_t http_probe_url(const char *url)
{
    esp_http_client_config_t cfg = {
        .url = url,
        .timeout_ms = 20000,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (client == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);

    ESP_LOGI(TAG, "HTTP GET %s -> %s status=%d", url, esp_err_to_name(err), status);
    return (err == ESP_OK && status >= 200 && status < 400) ? ESP_OK : ESP_FAIL;
}

static void iot_event_handle(void *arg,
                             esp_event_base_t event_base,
                             int32_t event_id,
                             void *event_data)
{
    (void)arg;

    if (event_base == IOT_ETH_EVENT) {
        switch (event_id) {
        case IOT_ETH_EVENT_START:
            ESP_LOGI(TAG, "RNDIS eth stack started");
            break;
        case IOT_ETH_EVENT_CONNECTED:
            ESP_LOGI(TAG, "RNDIS link up");
            xEventGroupSetBits(s_events, BIT_ETH_LINK_UP);
            break;
        case IOT_ETH_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "RNDIS link down");
            xEventGroupClearBits(s_events,
                                 BIT_ETH_LINK_UP | BIT_ETH_GOT_IP | BIT_PING_OK | BIT_PING_DONE);
            s_net_checks_done = false;
            break;
        default:
            break;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_ETH_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG,
                 "RNDIS DHCP: " IPSTR " mask " IPSTR " gw " IPSTR,
                 IP2STR(&ev->ip_info.ip),
                 IP2STR(&ev->ip_info.netmask),
                 IP2STR(&ev->ip_info.gw));

        /* Modem NAT hands out 192.168.10.2 as gateway; host must use .3 */
        ip4_addr_t modem_gw = {0};
        ip4addr_aton(MODEM_NAT_GW, &modem_gw);
        if (ev->ip_info.ip.addr == modem_gw.addr) {
            ESP_LOGW(TAG,
                     "DHCP gave modem GW " MODEM_NAT_GW " — switching host to " MODEM_HOST_IP);
            netif_apply_modem_nat_static_ip();
        } else {
            xEventGroupSetBits(s_events, BIT_ETH_GOT_IP);
        }
    }
}

static void network_task(void *arg)
{
    (void)arg;

    for (;;) {
        xEventGroupWaitBits(s_events, BIT_AT_SCRIPT_DONE, pdFALSE, pdTRUE, portMAX_DELAY);

        if (s_net_checks_done) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        ESP_LOGI(TAG, "AT script done — waiting for RNDIS link...");
        EventBits_t link = xEventGroupWaitBits(s_events,
                                               BIT_ETH_LINK_UP,
                                               pdFALSE,
                                               pdTRUE,
                                               pdMS_TO_TICKS(30000));
        if (!(link & BIT_ETH_LINK_UP)) {
            ESP_LOGW(TAG, "No RNDIS link — check USB / replug modem");
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        /* QCX216 NAT: host is always 192.168.10.3, modem gateway 192.168.10.2 */
        ESP_LOGI(TAG, "Applying static host IP " MODEM_HOST_IP " (gw " MODEM_NAT_GW ")");
        netif_apply_modem_nat_static_ip();
        if (!(xEventGroupGetBits(s_events) & BIT_ETH_GOT_IP)) {
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        wait_global_ipv6(IPV6_GLOBAL_WAIT_MS);
        netif_apply_modem_ipv6_preferred();

        if (ping_internet_check() == ESP_OK) {
            ESP_LOGI(TAG, "Ping IPv6: at least one reply received");
        } else {
            ESP_LOGW(TAG, "Ping IPv6: no replies (carrier may block ICMPv6)");
        }

        ESP_LOGI(TAG, "TCP test -> [%s]:%u", TEST_DNS_V6, TEST_DNS_PORT);
        if (tcp_connect_probe_v6(TEST_DNS_V6, TEST_DNS_PORT) == ESP_OK) {
            ESP_LOGI(TAG, "TCP IPv6 connect OK");
        } else {
            ESP_LOGW(TAG, "TCP IPv6 connect failed");
        }

        ESP_LOGI(TAG, "HTTPS GET -> %s", HTTP_PROBE_URL);
        if (http_probe_url(HTTP_PROBE_URL) == ESP_OK) {
            ESP_LOGI(TAG, "HTTPS probe OK — internet reachable");
        } else {
            ESP_LOGW(TAG, "HTTPS probe failed");
        }

        s_net_checks_done = true;
    }
}

/* -------------------------------------------------------------------------- */
/* app_main                                                                     */
/* -------------------------------------------------------------------------- */

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    s_events = xEventGroupCreate();
    s_at_port_mux = xSemaphoreCreateMutex();
    assert(s_events && s_at_port_mux);

    ESP_ERROR_CHECK(esp_event_handler_register(IOT_ETH_EVENT,
                                               ESP_EVENT_ANY_ID,
                                               iot_event_handle,
                                               NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,
                                               IP_EVENT_ETH_GOT_IP,
                                               iot_event_handle,
                                               NULL));

#if CONFIG_USB_HOST_ENABLE_ENUM_FILTER_CALLBACK
    BaseType_t ok = xTaskCreatePinnedToCore(usb_lib_task,
                                            "usb_lib",
                                            USB_HOST_TASK_STACK,
                                            xTaskGetCurrentTaskHandle(),
                                            USB_HOST_TASK_PRIO,
                                            NULL,
                                            0);
    assert(ok == pdPASS);
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000));
#endif

    usbh_cdc_driver_config_t cdc_cfg = {
        .task_stack_size = 4096,
        .task_priority = configMAX_PRIORITIES - 2,
        .task_coreid = 0,
#if CONFIG_USB_HOST_ENABLE_ENUM_FILTER_CALLBACK
        .skip_init_usb_host_driver = true,
#else
        .skip_init_usb_host_driver = false,
#endif
    };
    ESP_ERROR_CHECK(usbh_cdc_driver_install(&cdc_cfg));

    install_usb_rndis_host();

    /* Register after RNDIS so this callback runs first on NEW_DEV and opens AT
     * before the RNDIS handler claims interface 0. */
    ESP_ERROR_CHECK(usbh_cdc_register_dev_event_cb(s_qcx_match,
                                                   qcx216_dev_event_cb,
                                                   NULL));

    xTaskCreate(at_setup_task, "at_setup", AT_TASK_STACK, NULL, AT_TASK_PRIO, NULL);
    xTaskCreate(network_task, "net_task", NET_TASK_STACK, NULL, NET_TASK_PRIO, NULL);

#if CONFIG_APP_QUIT_PIN > 0
    const gpio_config_t quit_pin = {
        .pin_bit_mask = BIT64(CONFIG_APP_QUIT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&quit_pin));
#endif

    ESP_LOGI(TAG,
             "Ready. QCX216 0x%04X:0x%04X on ESP32 USB host. "
             "Tests: ping " PING_TARGET_V6 ", TCP [" TEST_DNS_V6 "]:%u, GET " HTTP_PROBE_URL,
             QCX216_USB_VID,
             QCX216_USB_PID,
             (unsigned)TEST_DNS_PORT);
}

#endif
