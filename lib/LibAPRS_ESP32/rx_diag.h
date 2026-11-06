#ifndef RX_DIAG_H_
#define RX_DIAG_H_

// RX diagnostic counters. Updated by the RX path (AFSK_Poll -> modem -> Ax25BitParse),
// which runs in a single task, so plain increments are enough. Readers on other tasks
// may see a counter mid-update, which is fine for monitoring.

#include <stdint.h>
#include "modem.h"

struct Ax25RxDiag
{
	uint32_t flags;       // HDLC 0x7E flags seen
	uint32_t shortFrames; // frame closed with 1..16 bytes (noise, or a cut-off frame)
	uint32_t attempts;    // frame closed with >= 17 bytes, so a CRC check was run
	uint32_t crcOk;       // CRC passed
	uint32_t crcFail;     // CRC failed
	uint32_t nonAprs;     // CRC ok but rejected: control/PID is not APRS UI
	uint32_t dupes;       // CRC ok, but the other demodulator already stored it
	uint32_t stored;      // frame handed to the RX frame buffer
	uint32_t bufFull;     // frame dropped because the RX frame buffer was full
	uint32_t aborts;      // 7 ones in a row in the middle of a frame
	uint32_t tooLong;     // frame exceeded AX25_FRAME_MAX_SIZE
	uint32_t fx25Tags;    // FX.25 correlation tag matched
	uint32_t fx25FecOk;   // Reed-Solomon decode succeeded
	uint32_t fx25Ok;      // FX.25 frame passed CRC and was stored
	uint32_t fx25Fail;    // FX.25 frame failed CRC/parse
	uint32_t fx25Fixed;   // total bytes corrected by FEC
	uint16_t lastFailLen; // byte length (incl. CRC) of the last CRC failure
};

extern volatile struct Ax25RxDiag Ax25Diag[MODEM_MAX_DEMODULATOR_COUNT];

struct AfskRxDiag
{
	int16_t rawMin;     // lowest raw ADC sample since last reset (mV on ESP32, raw counts elsewhere)
	int16_t rawMax;     // highest raw ADC sample since last reset
	uint32_t blocks;    // audio blocks processed
	uint32_t gated;     // blocks skipped because audio level was below the DCD gate
	uint32_t overflows; // times the ADC FIFO held more than a full buffer of samples
};

extern volatile struct AfskRxDiag AfskDiag;

#endif /* RX_DIAG_H_ */
