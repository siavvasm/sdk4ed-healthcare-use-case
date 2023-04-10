/**************************************************************************//**
 * @file
 * @brief FIR filter and seizure detection related declarations
 * @version 0.0.1
 ******************************************************************************/

#ifndef _FIR_H_
#define _FIR_H_

#include <stdint.h>
#include <stdbool.h>

#define SEIZ_OFF 0
#define SEIZ_ON (!SEIZ_OFF)

/* Number of samples */
#define NR_SAMPLES 64

/* Threshold Defaults  */
#define THRES_U_DEFAULT 130000000
#define THRES_L_DEFAULT 10000000

extern uint32_t threshold_u;
extern uint32_t threshold_l;

extern int16_t coeffRE[NR_SAMPLES];
extern int16_t coeffIM[NR_SAMPLES];

typedef struct{
	int16_t ain[NR_SAMPLES];
	uint8_t seizure;
	uint32_t sum;
} seizure_detection;

void fir_simple_wav(uint32_t * const sum, const int16_t *Ain, const int16_t *COEFF_re,
		    const int16_t *COEFF_im, const int startIndex);

uint8_t detection(uint32_t sum, uint32_t High_thres, uint32_t Low_thres, uint8_t state);

#endif /* _FIR_H_ */
