#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// 6바이트 프레임: [0xAA, CMD, AL, AH, CHK, 0x55]
// CHK = (0xAA ^ CMD ^ AL ^ AH) & 0xFF

typedef enum {
    VOICE_CMD_FWD   = 0x01,
    VOICE_CMD_BACK  = 0x02,
    VOICE_CMD_STOP  = 0x03,
    VOICE_CMD_LEFT  = 0x04,
    VOICE_CMD_RIGHT = 0x05,
} voice_cmd_t;

typedef enum {
    RX_ST_WAIT_HEAD = 0,
    RX_ST_GET_CMD,
    RX_ST_GET_AL,
    RX_ST_GET_AH,
    RX_ST_GET_CHK,
    RX_ST_GET_TAIL
} voice_rx_state_t;

typedef struct {
    uint8_t raw[6]; // [AA CMD AL AH CHK 55]
    uint8_t cmd;
    uint8_t al;
    uint8_t ah;
} voice_frame_t;

// Lifecycle / FSM
void    Voice_Init(void);
void    Voice_RxByteFromIRQ(uint8_t b);     // IRQ 컨텍스트에서 바이트 투입
bool    Voice_FrameAvailable(void);         // 프레임 준비 여부
bool    Voice_TryPopFrame(voice_frame_t* out); // 준비된 프레임 꺼내기

// Helpers
uint8_t Voice_CalcChecksum(uint8_t head, uint8_t cmd, uint8_t al, uint8_t ah);
bool    Voice_ValidateRaw6(const uint8_t f[6]);

#ifdef __cplusplus
}
#endif
