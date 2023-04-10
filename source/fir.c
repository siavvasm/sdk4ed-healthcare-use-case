/**************************************************************************//**
 * @file
 * @brief FIR filter and seizure detection related definitions
 * @version 0.0.1
 ******************************************************************************/

#include "fir.h"

uint32_t threshold_u = THRES_U_DEFAULT;
uint32_t threshold_l = THRES_L_DEFAULT;

int16_t coeffRE[NR_SAMPLES]={
	13,   14,   11,    5,   -4,  -14,  -23,  -28,  -28,  -22,
	-10,    4,   19,   30,   36,   34,   26,   13,   -2,  -16,
	-25,  -28,  -26,  -19,  -10,    0,    8,   13,   14,   12,
	9,    4,    0,   -2,   -4,   -4,   -4,   -2,   -1,    0,
	0,    1,    1,    1,    0,    0,    0,    0,    0,    0,
	0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
	0,    0,    0,    0
};

int16_t coeffIM[NR_SAMPLES]={
	0,    6,   13,   19,   21,   19,   12,    2,  -11,  -23,
	-31,  -34,  -30,  -19,   -5,   11,   24,   32,   34,   28,
	18,    5,   -7,  -16,  -21,  -21,  -17,  -10,   -4,    2,
	6,    8,    8,    6,    4,    1,    0,   -2,   -2,   -2,
	-1,   -1,    0,    0,    0,    0,    0,    0,    0,    0,
	0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
	0,    0,    0,    0
};

/**************************************************************************//**
 * @brief Produce a sum from 64 sensor inputs and the two coefficient arrays
 *****************************************************************************/
void fir_simple_wav(uint32_t * const sum, const int16_t *Ain, const int16_t *COEFF_re,
		    const int16_t *COEFF_im, const int startIndex) {

  int8_t k;
  int32_t sum_partRE = 0;
  int32_t sum_partIM = 0;
  int32_t sq1 = 0;
  int32_t sq2 = 0;
  int8_t sample_idx=0;

  *sum = 0;
  sample_idx = startIndex;

  for (k = 0; k < NR_SAMPLES; k++ ) {
    if( sample_idx == -1){
      sample_idx = NR_SAMPLES-1;
    }

    // /** Imaginary Part*/
    sum_partIM += COEFF_im[k] * Ain[sample_idx];
    // /** Real Part*/
    sum_partRE += COEFF_re[k] * Ain[sample_idx];

    sample_idx--;
  }

  /** Absolute value of Complex*/
  sq1 = sum_partRE * sum_partRE;
  sq2 = sum_partIM * sum_partIM;

  *sum  = sq2 + sq1;
}

/**************************************************************************//**
 * @brief Return 1 if the seizure is detected
 *****************************************************************************/
uint8_t detection(uint32_t sum, uint32_t High_thres, uint32_t Low_thres, uint8_t state){
	
  if( sum > High_thres && state != SEIZ_ON ){
    return SEIZ_ON;
  }
  else if( sum < Low_thres && state == SEIZ_ON ){
    return SEIZ_OFF;
  }
  else {
    return state;
  }
}
