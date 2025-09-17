#include "voice_proto.h"

static volatile voice_rx_state_t s_rx_state = RX_ST_WAIT_HEAD;
static volatile uint8_t  s_idx = 0;
static uint8_t  s_work[6];            // ISR 내부 임시 프레임 버퍼

static volatile bool     s_ready = false;
static volatile uint8_t  s_last[6];   // 메인 루프에서 읽을 최종 프레임

uint8_t Voice_CalcChecksum(uint8_t head, uint8_t cmd, uint8_t al, uint8_t ah)
{
    return (uint8_t)((head ^ cmd ^ al ^ ah) & 0xFF);
}

bool Voice_ValidateRaw6(const uint8_t f[6])
{
    if (f[0] != 0xAA) return false;
    if (f[5] != 0x55) return false;
    uint8_t chk = Voice_CalcChecksum(0xAA, f[1], f[2], f[3]);
    return (chk == f[4]);
}

void Voice_Init(void)
{
    s_rx_state = RX_ST_WAIT_HEAD;
    s_idx = 0;
    s_ready = false;
}

void Voice_RxByteFromIRQ(uint8_t b)
{
    switch (s_rx_state)
    {
        case RX_ST_WAIT_HEAD:
            if (b == 0xAA) {
                s_work[0] = 0xAA;
                s_idx = 1;
                s_rx_state = RX_ST_GET_CMD;
            }
            break;

        case RX_ST_GET_CMD:
            s_work[s_idx++] = b;
            s_rx_state = RX_ST_GET_AL;
            break;

        case RX_ST_GET_AL:
            s_work[s_idx++] = b;
            s_rx_state = RX_ST_GET_AH;
            break;

        case RX_ST_GET_AH:
            s_work[s_idx++] = b;
            s_rx_state = RX_ST_GET_CHK;
            break;

        case RX_ST_GET_CHK:
            s_work[s_idx++] = b;
            s_rx_state = RX_ST_GET_TAIL;
            break;

        case RX_ST_GET_TAIL:
            s_work[s_idx++] = b; // tail
            if (s_idx == 6) {
                for (int i = 0; i < 6; ++i) s_last[i] = s_work[i];
                s_ready = true;
            }
            s_rx_state = RX_ST_WAIT_HEAD;
            s_idx = 0;
            break;
    }
}

bool Voice_FrameAvailable(void)
{
    return s_ready;
}

bool Voice_TryPopFrame(voice_frame_t* out)
{
    if (!s_ready) return false;

    uint8_t f[6];
    for (int i = 0; i < 6; ++i) f[i] = s_last[i];
    s_ready = false;

    if (!Voice_ValidateRaw6(f)) return false;

    if (out) {
        for (int i = 0; i < 6; ++i) out->raw[i] = f[i];
        out->cmd = f[1];
        out->al  = f[2];
        out->ah  = f[3];
    }
    return true;
}
