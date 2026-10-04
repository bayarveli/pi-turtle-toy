#include "ble_control.h"

#include <atomic>
#include <cctype>
#include <cstring>

#include "esp_log.h"
#include "nvs_flash.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

namespace {
constexpr char kTag[] = "ble";
constexpr char kDeviceName[] = "JoyBot";

std::atomic<bool> g_blinking{true};
uint8_t g_own_addr_type;

// Nordic UART Service (commonly selectable in BLE joystick apps).
const ble_uuid128_t kServiceUuid = BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                                                    0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E);
const ble_uuid128_t kRxUuid = BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                                               0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E);
const ble_uuid128_t kTxUuid = BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                                               0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E);

const ble_uuid16_t kHm10Service = BLE_UUID16_INIT(0xFFE0);
const ble_uuid16_t kHm10Char = BLE_UUID16_INIT(0xFFE1);

void start_advertising();

// Accepts text ("start"/"stop", "on"/"off", "1"/"0", "A"/"B") or raw bytes 0x01/0x00.
void handle_command(const uint8_t *data, uint16_t len)
{
    char text[32] = {};
    for (uint16_t i = 0; i < len && i < sizeof(text) - 1; ++i) {
        text[i] = static_cast<char>(std::tolower(data[i]));
    }
    ESP_LOGI(kTag, "RX %u bytes: \"%s\" first=0x%02x", len, text, len ? data[0] : 0);

    if (strstr(text, "stop") || strstr(text, "off") || strcmp(text, "0") == 0 || strcmp(text, "b") == 0 ||
        (len == 1 && data[0] == 0x00)) {
        g_blinking = false;
    } else if (strstr(text, "start") || strstr(text, "on") || strcmp(text, "1") == 0 || strcmp(text, "a") == 0 ||
               (len == 1 && data[0] == 0x01)) {
        g_blinking = true;
    }
}

int rx_access(uint16_t, uint16_t, ble_gatt_access_ctxt *ctxt, void *)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        return 0;
    }
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    uint8_t buf[32];
    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len > sizeof(buf)) {
        len = sizeof(buf);
    }
    if (ble_hs_mbuf_to_flat(ctxt->om, buf, len, nullptr) == 0) {
        handle_command(buf, len);
    }
    return 0;
}

int tx_access(uint16_t, uint16_t, ble_gatt_access_ctxt *, void *)
{
    return 0;
}

const ble_gatt_svc_def kServices[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &kServiceUuid.u,
        .characteristics = (ble_gatt_chr_def[]){
            {.uuid = &kRxUuid.u, .access_cb = rx_access,
             .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP},
            {.uuid = &kTxUuid.u, .access_cb = tx_access, .flags = BLE_GATT_CHR_F_NOTIFY},
            {0},
        },
    },
    {
        // HM-10 style serial service used by many BLE terminal/joystick apps.
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &kHm10Service.u,
        .characteristics = (ble_gatt_chr_def[]){
            {.uuid = &kHm10Char.u, .access_cb = rx_access,
             .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_NOTIFY |
                      BLE_GATT_CHR_F_READ},
            {0},
        },
    },
    {0},
};

int gap_event(ble_gap_event *event, void *)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(kTag, "connect status=%d", event->connect.status);
        if (event->connect.status != 0) {
            start_advertising();
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
    case BLE_GAP_EVENT_ADV_COMPLETE:
        start_advertising();
        break;
    default:
        break;
    }
    return 0;
}

void start_advertising()
{
    ble_hs_adv_fields fields{};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name = reinterpret_cast<const uint8_t *>(kDeviceName);
    fields.name_len = strlen(kDeviceName);
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    ble_hs_adv_fields rsp{};
    rsp.uuids128 = const_cast<ble_uuid128_t *>(&kServiceUuid);
    rsp.num_uuids128 = 1;
    rsp.uuids128_is_complete = 1;
    ble_gap_adv_rsp_set_fields(&rsp);

    ble_gap_adv_params params{};
    params.conn_mode = BLE_GAP_CONN_MODE_UND;
    params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    int rc = ble_gap_adv_start(g_own_addr_type, nullptr, BLE_HS_FOREVER, &params, gap_event, nullptr);
    if (rc != 0) {
        ESP_LOGE(kTag, "adv start failed: %d", rc);
    }
}

void on_sync()
{
    ble_hs_util_ensure_addr(0);
    ble_hs_id_infer_auto(0, &g_own_addr_type);
    start_advertising();
    ESP_LOGI(kTag, "Advertising as \"%s\"", kDeviceName);
}

void host_task(void *)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}
} // namespace

void ble_control_start()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(nimble_port_init());
    ble_hs_cfg.sync_cb = on_sync;
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ESP_ERROR_CHECK(ble_gatts_count_cfg(kServices));
    ESP_ERROR_CHECK(ble_gatts_add_svcs(kServices));
    ble_svc_gap_device_name_set(kDeviceName);
    nimble_port_freertos_init(host_task);
}

bool ble_control_blinking_enabled()
{
    return g_blinking;
}


