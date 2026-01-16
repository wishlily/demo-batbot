#include "ms200.h"

#include <math.h>
#include <string.h>
#include <assert.h>
#include <sys/param.h>

#include "esp_log.h"

#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "Current architecture is Big-Endian! Direct struct mapping for MS200 will fail."
#endif

#define SN_SIZE 16
#define VERSION_SIZE 32

static const char* TAG = "ms200";

// CRC8 data table, polynomial(0x4d):x^6 + x^3 + x^2 + 1
static const uint8_t CRC_TABLE[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
    0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
    0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
    0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
    0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
    0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
    0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
    0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
    0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
    0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
    0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8};

static char ms200_sn[SN_SIZE] = {0};
static char ms200_version[VERSION_SIZE] = {0};
static ms200_update_t ms200_update_fn = NULL;

// Returns the value of the calculated CRC8
static uint8_t ms200_calculate_crc8(uint8_t* protocol_buf, uint8_t crc_len)
{
    uint8_t crc = 0x00;
    for (int i = 0; i < crc_len; i++) {
        crc = CRC_TABLE[(crc ^ protocol_buf[i]) & 0xFF];
    }
    return crc;
}

typedef struct {
    uint8_t head1;
    uint8_t head2;
    uint8_t flag;
    uint8_t len;
    uint8_t payload[]; // C99 Flexible Array Member
} __attribute__((packed)) ms200_report_head_t;

typedef struct {
    uint8_t crc;
    uint8_t tail1;
    uint8_t tail2;
} __attribute__((packed)) ms200_report_tail_t;

static_assert(sizeof(ms200_report_head_t) == 4, "report size mismatch");
static_assert(offsetof(ms200_report_head_t, payload) == 4, "report payload offset incorrect");
static_assert(sizeof(ms200_report_tail_t) == 3, "report tail size mismatch");

static ms200_error_t ms200_parse_report(uint8_t* buf, uint16_t buf_len)
{
    if (buf_len < sizeof(ms200_report_head_t))
        return MS200_ERROR_LEN;

    const ms200_report_head_t* pkt = (const ms200_report_head_t*)buf;
    uint8_t data_len = pkt->len;

    uint16_t expected_packet_len = sizeof(ms200_report_head_t) + data_len + sizeof(ms200_report_tail_t);
    if (expected_packet_len > buf_len)
        return MS200_ERROR_LEN;

    const ms200_report_tail_t* tail = (const ms200_report_tail_t*)(pkt->payload + data_len);

    if (tail->tail1 != MS200_TAIL_1 || tail->tail2 != MS200_TAIL_2)
        return MS200_ERROR_TAIL;
    if (tail->crc != ms200_calculate_crc8(buf, data_len + sizeof(ms200_report_head_t)))
        return MS200_ERROR_CRC;

    // Determine the SN flag bit and save the SN information
    if (pkt->flag == MS200_FLAG_SN) {
        int len = MIN(data_len, SN_SIZE - 1);
        if (len > SN_SIZE - 1) {
            ESP_LOGE(TAG, "SN size overflow %d", data_len);
        }
        memcpy(ms200_sn, pkt->payload, len);
        ms200_sn[len] = '\0';
        ESP_LOGI(TAG, "SN: %s", ms200_sn);
    }
    // Determine the version flag bit and save the version information
    else if (pkt->flag == MS200_FLAG_VERSION) {
        int len = MIN(data_len, SN_SIZE - 1);
        if (len > VERSION_SIZE - 1) {
            ESP_LOGE(TAG, "Version size overflow %d", data_len);
        }
        memcpy(ms200_version, pkt->payload, len);
        ms200_version[len] = '\0';
        ESP_LOGI(TAG, "Version: %s", ms200_version);
    }
    return MS200_OK;
}

typedef struct {
    uint8_t header;
    uint8_t count;
    uint16_t speed;
    uint16_t start_angle;   // unit: 0.01 deg
    ms200_point_t points[]; // C99 Flexible Array Member
} __attribute__((packed)) ms200_pkg_head_t;

typedef struct {
    uint16_t end_angle; // unit: 0.01 deg
    uint16_t timestamp; // 0~29999ms first point timestamp
    uint8_t crc8;
} __attribute__((packed)) ms200_pkg_tail_t;

static_assert(sizeof(ms200_point_t) == 3, "point size mismatch");
static_assert(sizeof(ms200_pkg_head_t) == 6, "header size mismatch");
static_assert(sizeof(ms200_pkg_tail_t) == 5, "tail size mismatch");
static_assert(offsetof(ms200_pkg_head_t, points) == 6, "points offset mismatch");

static ms200_error_t ms200_parse_package(const uint8_t* buf, uint16_t buf_len, ms200_package_t* out)
{
    if (buf_len < sizeof(ms200_pkg_head_t))
        return MS200_ERROR_LEN;

    const ms200_pkg_head_t* hdr = (const ms200_pkg_head_t*)buf;
    uint8_t count = hdr->count & 0x1F;

    uint16_t points_len = count * sizeof(ms200_point_t);
    uint16_t expected_len = sizeof(ms200_pkg_head_t) + points_len + sizeof(ms200_pkg_tail_t);

    if (expected_len > buf_len)
        return MS200_ERROR_LEN;

    const ms200_pkg_tail_t* tail = (const ms200_pkg_tail_t*)((uint8_t*)hdr->points + points_len);
    if (tail->crc8 != ms200_calculate_crc8((uint8_t*)buf, expected_len - 1))
        return MS200_ERROR_CRC;

    out->speed = hdr->speed;
    out->start_angle = hdr->start_angle;
    out->end_angle = tail->end_angle;
    out->timestamp = tail->timestamp;

    if (count > MS200_POINT_PER_PACK) {
        ESP_LOGW(TAG, "Point count overflow %d", count);
        count = MS200_POINT_PER_PACK;
    }
    out->count = count;

    for (int i = 0; i < count; i++) {
        out->points[i].distance = hdr->points[i].distance;
        out->points[i].intensity = hdr->points[i].intensity;
    }
    return MS200_OK;
}

// Cache for receiving protocol data
static uint8_t rx_protocol_buf[MS200_BUF_MAX] = {0};
static ms200_package_t ms200_pkg = {0};

typedef enum {
    STATE_IDLE = 0,
    STATE_HEAD_2,      // Waiting for system packet second header 0xAA
    STATE_SYS_FLAG,    // System packet flag
    STATE_SYS_LEN,     // System packet length
    STATE_SYS_PAYLOAD, // System packet payload
    STATE_DATA_N,      // Point cloud data point count
    STATE_DATA_PAYLOAD // Point cloud data payload
} ms200_fsm_state_t;

// The lidar receives and processes data
void ms200_data_receive(uint8_t rx_data)
{
    static ms200_fsm_state_t state = STATE_IDLE;
    static uint16_t rx_idx = 0;
    static uint16_t expected_len = 0;

    if (rx_idx >= MS200_BUF_MAX) {
        ESP_LOGW(TAG, "Buffer overflow, resetting");
        state = STATE_IDLE;
        rx_idx = 0;
    }

    switch (state) {
    case STATE_IDLE:
        if (rx_data == MS200_HEAD_1) {
            state = STATE_HEAD_2;
            rx_protocol_buf[0] = rx_data;
            rx_idx = 1;
        } else if (rx_data == MS200_DATA_START) {
            state = STATE_DATA_N;
            rx_protocol_buf[0] = rx_data;
            rx_idx = 1;
        }
        break;
    // branch SN/Version
    case STATE_HEAD_2:
        if (rx_data == MS200_HEAD_2) {
            state = STATE_SYS_FLAG;
            rx_protocol_buf[rx_idx++] = rx_data;
        } else {
            state = STATE_IDLE;
            rx_idx = 0;
        }
        break;
    case STATE_SYS_FLAG:
        state = STATE_SYS_LEN;
        rx_protocol_buf[rx_idx++] = rx_data;
        break;
    case STATE_SYS_LEN:
        rx_protocol_buf[rx_idx++] = rx_data;
        expected_len = sizeof(ms200_report_head_t) + rx_data + sizeof(ms200_report_tail_t);
        if (expected_len > MS200_BUF_MAX) {
            state = STATE_IDLE;
            rx_idx = 0;
        } else {
            state = STATE_SYS_PAYLOAD;
        }
        break;
    case STATE_SYS_PAYLOAD:
        rx_protocol_buf[rx_idx++] = rx_data;
        if (rx_idx >= expected_len) {
            int rc = ms200_parse_report(rx_protocol_buf, rx_idx);
            if (rc != MS200_OK) {
                ESP_LOGE(TAG, "Parse system report error %d", rc);
            }
            state = STATE_IDLE;
            rx_idx = 0;
        }
        break;
    // branch Point Data
    case STATE_DATA_N:
        rx_protocol_buf[rx_idx++] = rx_data;
        expected_len = sizeof(ms200_pkg_head_t) + (rx_data & 0x1F) * 3 + sizeof(ms200_pkg_tail_t);
        if (expected_len > MS200_BUF_MAX) {
            state = STATE_IDLE;
            rx_idx = 0;
        } else {
            state = STATE_DATA_PAYLOAD;
        }
        break;
    case STATE_DATA_PAYLOAD:
        rx_protocol_buf[rx_idx++] = rx_data;
        if (rx_idx >= expected_len) {
            int rc = ms200_parse_package(rx_protocol_buf, rx_idx, &ms200_pkg);
            if (rc == MS200_OK && ms200_update_fn) {
                ms200_update_fn(&ms200_pkg);
            } else {
                ESP_LOGE(TAG, "Parse point cloud error %d", rc);
            }
            state = STATE_IDLE;
            rx_idx = 0;
        }
        break;
    default:
        state = STATE_IDLE;
        rx_idx = 0;
        break;
    }
}

void ms200_set_update_cb(ms200_update_t cb)
{
    ms200_update_fn = cb;
}
