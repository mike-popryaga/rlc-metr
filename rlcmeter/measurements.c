/**
 * @defgroup: RLC Meter V6
 * @file:     measurments.c
 *
 * @brief:    Module of Pseudo-Class
 *            Hardware RLC meter for STM32F100
 *
 * @data:     February, 2013
 * @author:   Neekeetos
 *
 */


#include "measurements.h"
#include "arithmetic.h"
#include "main.h"
#include "math.h"
#include "eeprom.h"


static void runRound(void);
void TIM1_BRK_TIM15_IRQHandler(void);
void __attribute__((optimize("-O3"))) DMA1_Channel1_IRQHandler(void);
void __attribute__((optimize("-O3"))) DMA1_Channel4_IRQHandler(void);

///==============================================================================






extern volatile uint32_t vbat;
extern volatile uint32_t vcc;
extern volatile uint32_t vref;
extern volatile int32_t sign;
extern volatile int32_t gain;
extern volatile int32_t swap_dacs;
extern volatile int hundredMsTick;
extern int __attribute__ ((section (".noinit"))) findex, cstatus;
extern balance_data_t __attribute__ ((section (".noinit"))) corr;
extern cal_data_t __attribute__ ((section (".noinit"))) cal; // calibration constants
extern cplx __attribute__ ((section (".noinit"))) R;

extern  uint32_t dac_buf[DAC_N];
extern uint16_t __attribute__ ((section (".noinit"))) adc_dma[N];
extern const uint32_t cht[CH_NUM]; // + VBAT, TEMP, VREF
extern measure_t mdata;
extern cplx __attribute__ ((section (".noinit"))) mAcc[3];
extern volatile uint64_t sys_time_counter;
///==============================================================================

static volatile cplx mData[3];
static volatile uint32_t irq_request = 0;


static int16_t sine[N + N / 4];
static const uint32_t chn[CH_NUM] = { 1, 2, 7, 17, 8 }; // + VBAT, TEMP, VREF



static const uint32_t chk[CH_NUM] = {
                                      ADC_CHANGEOVR + N / 2,
                                      ADC_CHANGEOVR,
                                      ADC_CHANGEOVR,
                                      ADC_CHANGEOVR,
                                      ADC_CHANGEOVR }; // + VBAT, TEMP, VREF



/**
 * @brief:  runRound()
 *          This function calls __NOP() while irq_request and IRQ_ADC_SAMPLE == 1.
 *
 * @param:  void
 * @return: void
 */

static void runRound(void) // result in mData[3]
{
	irq_request |= IRQ_ADC_SAMPLE;
	while ((irq_request & IRQ_ADC_SAMPLE))
		__NOP();
}


//////////////////////////////////////////////////////
/**
 * @brief:  fillSine()
 *          This function fills sine massive
 *
 * @param:  freq
 * @return: void
 */

////////////////////////////////////////////////////////////
void fillSine(int freq) {
	int s, i;
	long long pp;
	for (i = 0i; i < N; i++) {
		pp = (((long long) freq * i << 32) / N);
		s = cordic((int) pp);

		if (s > 0)
			s = ((s >> 14) + 1) >> 1;
		else
			s = ((s >> 14) - 1) >> 1;

		sine[i] = s;
	}

	for (i = 0; i < N / 4; i++)
		sine[i + N] = sine[i];
	// load Corr[findex] from eeprom

	resetBalanceData();

	cstatus = 0;

	if (eepromLoadParam( EEP_BALANCE_BASE + findex, &corr) > 0)
		cstatus |= (1 << (findex + 8));
	if (eepromLoadParam( EEP_CAL_BASE + findex, &cal) > 0)
		cstatus |= (1 << (findex));

}



//----------------------------------------------------------------------------
// This function makes measurement
//----------------------------------------------------------------------------
void measure(cplx * Z, int rounds) {
	cplx I, ch[3];

	mAcc[0].Re = 0;
	mAcc[0].Im = 0;
	mAcc[1].Re = 0;
	mAcc[1].Im = 0;
	mAcc[2].Re = 0;
	mAcc[2].Im = 0;

	int r = rounds;

	while (r-- > 0) {
		runRound();
		mAcc[0].Re += mData[0].Re;
		mAcc[0].Im += mData[0].Im;
		mAcc[1].Re += mData[1].Re;
		mAcc[1].Im += mData[1].Im;
		mAcc[2].Re += mData[2].Re;
		mAcc[2].Im += mData[2].Im;

		runRound();
		mAcc[0].Re -= mData[0].Re;
		mAcc[0].Im -= mData[0].Im;
		mAcc[1].Re -= mData[1].Re;
		mAcc[1].Im -= mData[1].Im;
		mAcc[2].Re -= mData[2].Re;
		mAcc[2].Im -= mData[2].Im;

	}

	ch[0].Re = mAcc[0].Re;
	ch[0].Im = mAcc[0].Im;
	ch[1].Re = mAcc[1].Re;
	ch[1].Im = mAcc[1].Im;
	ch[2].Re = mAcc[2].Re;
	ch[2].Im = mAcc[2].Im;

	cplxMul(&ch[0], &corr.shift0);
	cplxMul(&ch[1], &corr.shift0);
	cplxMul(&ch[2], &corr.shift0);

	mAcc[0].Re -= ch[1].Re + ch[2].Re;
	mAcc[0].Im -= ch[1].Im + ch[2].Im;
	mAcc[1].Re -= ch[0].Re + ch[2].Re;
	mAcc[1].Im -= ch[0].Im + ch[2].Im;
	mAcc[2].Re -= ch[0].Re + ch[1].Re;
	mAcc[2].Im -= ch[0].Im + ch[1].Im;

	mdata.polar[0].Re = 0.5 * vcc * 1.4901161e-12
			* square(mAcc[0].Re * mAcc[0].Re + mAcc[0].Im * mAcc[0].Im); // 1/512/65536/20000
	mdata.polar[1].Re = 0.5 * vcc * 1.4901161e-12
			* square(mAcc[1].Re * mAcc[1].Re + mAcc[1].Im * mAcc[1].Im);
	mdata.polar[2].Re = 0.5 * vcc * 1.4901161e-12
			* square(mAcc[2].Re * mAcc[2].Re + mAcc[2].Im * mAcc[2].Im);

	mdata.polar[0].Im = 180.0 * atan2f(mAcc[0].Im, mAcc[0].Re) / M_PI;
	mdata.polar[1].Im = 180.0 * atan2f(mAcc[1].Im, mAcc[1].Re) / M_PI;
	mdata.polar[2].Im = 180.0 * atan2f(mAcc[2].Im, mAcc[2].Re) / M_PI;

	cplxMul(&mAcc[0], &(corr.Corr0));
	cplxMul(&mAcc[2], &(corr.Corr2));

	Z->Re = (mAcc[2].Re - mAcc[0].Re); // V
	Z->Im = (mAcc[2].Im - mAcc[0].Im);
	cplxMul(Z, &R);
	I.Re = (mAcc[0].Re - mAcc[1].Re); // I
	I.Im = (mAcc[0].Im - mAcc[1].Im);
	cplxDiv(Z, &I);
}

//----------------------------------------------------------------------------
float filter(int sidx, float new) {
	static float state[4];
	static int cnt[4];

	float t = (state[sidx] - new);
	state[sidx] = state[sidx] * (1 - ALPHA) + new * ALPHA;
	if (t < 0)
		cnt[sidx]++;
	else
		cnt[sidx]--;

	if ((cnt[sidx] > 2 * FLT_LEN) || (cnt[sidx] < -2 * FLT_LEN)) {
		state[sidx] = new;
		cnt[sidx] = 0;
	}

	return state[sidx];
}




/**
 * @brief:  resetBalanceData()
 *          This function resets corr variable
 *
 * @param:  void
 * @return: void
 */

void resetBalanceData(void) {

	corr.Corr0.Re = 1.0;
	corr.Corr0.Im = 0;
	corr.Corr2.Re = 1.0;
	corr.Corr2.Im = 0;

	corr.shift0.Re = 0;
	corr.shift0.Im = 0;
	corr.shift1.Re = 0;
	corr.shift1.Im = 0;
	corr.shift2.Re = 0;
	corr.shift2.Im = 0;
}

/////////////////////////////////////////////////////////////////////////////////
//                             IRQ/DMA HANDLERs
/////////////////////////////////////////////////////////////////////////////////

volatile int k = 0;
volatile long long mreal[CH_NUM];
volatile long long mimag[CH_NUM];

void TIM1_BRK_TIM15_IRQHandler(void) { // switch channels

	//if(zrap != 0) goto stop; // my test
	//zrap = 1;                 // my test

	static int round = 0;

	sys_time_counter++; //my

	//static char morg = 0;

	/*if (morg) {
		GPIOA->BSRR = GPIO_Pin_15;
		morg = 0;
	} else {
		GPIOA->BSRR = GPIO_Pin_15 << 16;
		morg = 1;
	};*/ // BACKLIGHT OFF

	TIM15->SR &= ~TIM_SR_CC1IF; // clear CC1IF flag

	GPIOB->BSRR = GPIO_Pin_14; // reset

	DMA1_Channel1->CCR &= ~DMA_CCR1_EN;
	DMA1->IFCR = DMA1_IT_GL1;

	DMA1_Channel1->CNDTR = N;
	DMA1_Channel1->CCR |= DMA_CCR1_EN;

	if (irq_request & IRQ_ADC_SAMPLE) {
		switch (k) {
		case VREF_CH:
			vref = (adc_dma[0] + adc_dma[1] + adc_dma[2] + adc_dma[3]
					+ adc_dma[4] + adc_dma[5] + adc_dma[6] + adc_dma[7]);
			vcc = 8 * 4096 * 1200 / vref;
			break;
		case BAT_CH:
			vbat = 2
					* (adc_dma[0] + adc_dma[1] + adc_dma[2] + adc_dma[3]
							+ adc_dma[4] + adc_dma[5] + adc_dma[6] + adc_dma[7])
					* 1200 / vref;
			break;
		}

		k++;
		if (((k >= VREF_CH) && (round > 0)) || (k >= CH_NUM)) {
			k = 0;
			round--;
			if (round < 0) {
				round = BAT_SKIP;
			}

			mData[0].Re = mreal[0];
			mData[0].Im = mimag[0];
			mData[1].Re = mreal[1];
			mData[1].Im = mimag[1];
			mData[2].Re = mreal[2];
			mData[2].Im = mimag[2];

			irq_request &= ~IRQ_ADC_SAMPLE;
		}
	}

	mreal[k] = 0;
	mimag[k] = 0;
	ADC1->SQR3 = chn[k];
	TIM15->ARR = cht[k];
	TIM15->CCR2 = chk[k];

	GPIOB->BRR = GPIO_Pin_14; // reset
	//zrap = 0;  // my test
//stop: exit(1); // my test
}

void __attribute__((optimize("-O3"))) DMA1_Channel1_IRQHandler(void) { // ADC DMA
	long long re, im;
	static int lc = 0;

	int i, j;

	GPIOB->BSRR = GPIO_Pin_13; // set

	lc++;
	if (lc > 200) {
		hundredMsTick++;
		lc = 0;
	}

	if ( DMA1->ISR & DMA_ISR_HTIF1) {
		j = 0;
	} else {
		j = N / 2;
	}

	DMA1->IFCR = DMA1_IT_GL1;

	i = 0;
	re = 0;
	im = 0;
	int16_t * cos = &sine[j + N / 4], *sin = &sine[j];
	uint16_t* buf = &adc_dma[j];

	while (i++ < N / 2) {
		int dat = ((*buf++) - SINE_OFFSET);

		re += ((int) (*cos++) * dat);
		im -= ((int) (*sin++) * dat);
	}

	GPIOB->BRR = GPIO_Pin_13; // reset
	__NOP();
	GPIOB->BSRR = GPIO_Pin_13; // set

	mreal[k] += re;
	mimag[k] += im;

	GPIOB->BRR = GPIO_Pin_13; // reset
}

void __attribute__((optimize("-O3"))) DMA1_Channel4_IRQHandler(void) { // DAC DMA

//	GPIOB->BSRR = GPIO_Pin_14; // reset

	int j;
	static uint32_t * dptr = dac_buf;
	static int sptr = 0;
	static int lsign = -1;
	static int lgain = 2048;
	const uint32_t k = (SINE_OFFSET | (SINE_OFFSET << 16));

	lsign = sign;
	lgain = gain;

	if (DMA1->ISR & DMA_ISR_HTIF4) {
		dptr = dac_buf;
	}

	DMA1->IFCR = DMA1_IT_GL4;

	if (sptr >= N) {
		sptr = 0;
	}
	j = DAC_N / 2;

	if (swap_dacs) {
		while (j > 0) {
			int32_t sig = (lgain * sine[sptr++]) >> 16; //11bit
			*dptr++ = k + (sig << 16) + lsign * sig;
			j--;
		}
	} else {
		while (j > 0) {
			int32_t sig = (lgain * sine[sptr++]) >> 16; //11bit
			*dptr++ = k + lsign * (sig << 16) + sig;
			j--;
		}
	}

//GPIOB->BRR = GPIO_Pin_14; // reset

}


