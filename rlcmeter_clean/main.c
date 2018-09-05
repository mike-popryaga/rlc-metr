/**
 * @defgroup: RLC Meter V6
 * @file:     main.c
 *
 * @brief:    Module of Pseudo-Class
 *            Hardware RLC meter for STM32F100
 *
 * @data:     February, 2013
 * @author:   Neekeetos
 *
 */

///=============================================================================
/// Includes
///=============================================================================
#include "string.h"
#include "math.h"
#include "misc.h"
#include "uart.h"

#include <malloc.h>
#include <stdio.h>
#include <stdlib.h>

#include <assert.h> // my test
#include "SPI_communication.h"
#include "stm32f10x_rcc.h"

///==============================================================================

/// Global
///==============================================================================

const char * sOnOff[] = { "OFF", "ON" };
const char * sLCMode[] = { "SER", "PAR" };
const char * sRLim[] = { "OFF", "1M", "10M" };
const char * sScrMode[] = { "1110", "1202ud", "1202", "1110ud" };
const char * sParamMode[] = { "OFF", "Z", "Q", "D", "VDD", "VBAT", "ANGL" };
const char * sFreqs[] = { " 1k", " 9k", "25k", "49k", "97k" };
const char * sLowBat[] = { "OFF", "3V0", "3V2", "3V5" };

const uint32_t iLowBat[] = { 0, 3000, 3200, 3500 };

#define FCNT 5
const int flist[FCNT] = { 1, 9, 25, 49, 97 };
const int fgain[FCNT] = { 1298, 1333, 1587, 2174, 3636 };
const float rlim[3] = { 1e12, 1e6, 10e6 };

volatile uint32_t vbat = 0;
volatile uint32_t vcc = 0;
volatile uint32_t vref = 0;

volatile int32_t sign = -1;
volatile int32_t gain = 2500;

volatile int32_t swap_dacs = 0;

volatile int hundredMsTick = 0;

volatile uint64_t sys_time_counter = 0;  //my
volatile uint64_t sys_time;  //my
uint64_t time_accumulator[100];  //my
volatile int zrap = 0;              // my test

const int cordic_ctab[] = { 0x20000000, 0x12E4051E, 0x09FB385B, 0x051111D4,
		0x028B0D43, 0x0145D7E1, 0x00A2F61E, 0x00517C55, 0x0028BE53, 0x00145F2F,
		0x000A2F98, 0x000517CC, 0x00028BE6, 0x000145F3, 0x0000A2FA, 0x0000517D,
		0x000028BE, 0x0000145F, 0x00000A30, 0x00000518, 0x0000028C, 0x00000146,
		0x000000A3, 0x00000051, 0x00000029, 0x00000014, 0x0000000A, 0x00000005,
		0x00000003, 0x00000001, 0x00000001, 0x00000000 };

const char dp[9] = { 'f', 'p', 'n', 'u', 'm', ' ', 'k', 'M', 'G' };

const int dig[] = { 1000000000, 100000000, 10000000, 1000000, 100000, 10000,
		1000, 100, 10, 1 };

///==============================================================================

cplx Z, base;
measure_t mdata;

cplx __attribute__ ((section (".noinit"))) mAcc[3];

balance_data_t __attribute__ ((section (".noinit"))) corr;
cal_data_t __attribute__ ((section (".noinit"))) cal; // calibration constants

int __attribute__ ((section (".noinit"))) apo;
cplx __attribute__ ((section (".noinit"))) R;

int freq;
int __attribute__ ((section (".noinit"))) findex, cstatus;
uint32_t __attribute__ ((section (".noinit"))) bitParams;

int __attribute__ ((section (".noinit"))) var1; //my test
int __attribute__ ((section (".noinit"))) var2; //my test
my_time time1;

uint32_t dac_buf[DAC_N];
uint16_t __attribute__ ((section (".noinit"))) adc_dma[N];

const uint32_t cht[CH_NUM] = {
ADC_N * OSR + ADC_CHANGEOVR - 1 + N / 2,
ADC_N * OSR + ADC_CHANGEOVR - 1,
ADC_N * OSR + ADC_CHANGEOVR - 1,
ADC_N + ADC_CHANGEOVR - 1,
ADC_N + ADC_CHANGEOVR - 1 }; // + VBAT, TEMP, VREF

//----------------------------------------------------------------------------
int menuCurrent = 0, btnState = 0, startTime = 0;

///==============================================================================

/// Function prototype
///==============================================================================
int menuProcess(int a);
int leafProcess(int a);
int leafInfo(int a);
int leafScr(int a);
int leafBalance(int a);
int leafCal(int a);
int leafCalReset(int a);
int leafBatLow(int a);
int diagScr(int btns);

int editScr(int sel, char* str, int btns);
int editContrast(int sel, char* str, int btns);
int editR(int sel, char* str, int btns);
int editBF(int sel, char* str, int btns);
int editBalance(int sel, char* str, int btns);
int editCal(int sel, char* str, int btns);
int editAverages(int sel, char* str, int btns);

void setup(void);

void erase(void);

/////
typedef struct {
	uint64_t sys_time;
	int set_sys_time;
	uint32_t push_time;
} timer_tt;

typedef struct {
	int flags;
	button_state_t current_state;
	button_state_t last_state;
	timer_tt * timer;
} button_tt;

state_toggle toggle = off;
button_position button;

timer_tt* new_timer(void);
int set_timer(timer_tt*, int);
int get_timer(timer_tt*);
int del_timer(timer_tt*);

///=============================================================================

/// Defines
///=============================================================================

#define   BIT_BL_MODE           0
#define   BIT_REL_MODE          1
#define   BIT_LC_MODE           2
#define   BIT_LIM_RANGE         3
#define   BIT_UART_MODE         4
#define   BIT_SCR_MODE          5
#define   BIT_SCR_CONTRAST      6
#define   BIT_AVERAGES          7
#define   BIT_1STPARAM          8
#define   BIT_2NDPARAM          9
#define   BIT_LOWBAT           10
#define   BIT_3RDPARAM         11
#define   CONFIG_BITS_COUNT    12
#define   MS                   16
#define   MAIN_PRESCALER_500K  64
#define   AMAUNT               20

const bitField configBits[] = { // val,        list,        startbit,  bitlen,  max
		{ &bitParams, sOnOff, 0, 1, 1 },  // BACKLIGHT
				{ &bitParams, sOnOff, 1, 1, 1 },  // REL MODE
				{ &bitParams, sLCMode, 2, 1, 1 },  // LC MODE
				{ &bitParams, sRLim, 3, 2, 2 },  // LIMIT RANGE
				{ &bitParams, sOnOff, 5, 1, 1 },  // UART MODE
				{ &bitParams, sScrMode, 6, 2, 3 },  // SCR MODE
				{ &bitParams, NULL, 8, 5, 31 },  // SCR CONTRAST
				{ &bitParams, NULL, 13, 5, 20 },  // AVERAGES
				{ &bitParams, sParamMode, 18, 3, 6 },  // 1st param
				{ &bitParams, sParamMode, 21, 3, 6 },  // 2nd param
				{ &bitParams, sLowBat, 24, 2, 3 },  // low bat
				{ &bitParams, sParamMode, 26, 3, 6 },  // 3rd param
		};

#define   MENU_ITEM_COUNT   21
#define   MENU_DISP          0
#define   MENU_MAIN          1
#define   MENU_CAL          14
#define   MENU_INFO         18
#define   MENU_SCR          19
#define   MENU_LOWBAT       20
#define   MENU_DIAG         21
// menuHandler   menuParamEdit
const menuItem menu[] = { // m             d              parent      first        last    name             timeout
				{ leafProcess, NULL, MENU_MAIN, 0, 0, "MEAS DISPLAY", -1 },  //0
				{ menuProcess, NULL, MENU_DISP, 2, 13, "MAIN MENU", 200 },   //1
				{ menuProcess, editContrast, MENU_MAIN, 0, 0, "CONTRAST", -1 }, //2
				{ menuProcess, editScr, MENU_MAIN, 0, 0, "DISP MODE", -1 },  //3
				{ menuProcess, editBF, MENU_MAIN, BIT_BL_MODE, 0, "BACKLIGHT",
						-1 },   //4
				{ menuProcess, editAverages, MENU_MAIN, 0, 0, "AVERAGES", -1 }, //5
				{ menuProcess, editR, MENU_MAIN, 0, 0, "SHUNT R", -1 },   //6
				{ menuProcess, editBF, MENU_MAIN, BIT_LC_MODE, 0, "LC EQUIV", -1 }, //7
				{ menuProcess, editBF, MENU_MAIN, BIT_LIM_RANGE, 0, "RANGE LIM",
						-1 },   //8
				{ menuProcess, editBF, MENU_MAIN, BIT_1STPARAM, 0, "PARAM 1", -1 }, //9
				{ menuProcess, editBF, MENU_MAIN, BIT_2NDPARAM, 0, "PARAM 2", -1 }, //10
				{ menuProcess, editBF, MENU_MAIN, BIT_3RDPARAM, 0, "PARAM 3", -1 }, //11
				{ menuProcess, editBF, MENU_MAIN, BIT_UART_MODE, 0, "UART OUT",
						-1 },   //12
				{ menuProcess, editBF, MENU_MAIN, BIT_LOWBAT, 0, "LOW BAT", -1 }, //13
				{ menuProcess, NULL, MENU_DISP, 15, 17, "CALIBRATION", 200 }, //14
				{ leafBalance, editBalance, MENU_DISP, 15, 0, "CH BALANCE", -1 }, //15
				{ leafCal, editCal, MENU_DISP, 0, 0, "OPEN-SHORT", -1 },   //16
				{ leafCalReset, NULL, MENU_CAL, 0, 0, "RESET", -1 },   //17
				{ leafInfo, NULL, MENU_DISP, 0, 0, "Info", 20 },   //18
				{ leafScr, NULL, MENU_DISP, 0, 0, "scrMode", -1 },   //19
				{ leafBatLow, NULL, MENU_DISP, 0, 0, "lowBat", 20 },   //20
				{ diagScr, NULL, MENU_DISP, 0, 0, "diagScr", -1 },   //21
		};

///==========================================================================================================================
///==========================================================================================================================
///==========================================================================================================================

#define DELAY_OVERHEAD (18 * (1000000000 / 24000000)) // Daumenpeilung

static unsigned delay_ns_per_tick = 1;

void delay_ns(unsigned ns) {
	unsigned t = DELAY_OVERHEAD;
	__asm volatile(".balign 8\n"
			"1: add %0, %1\n"
			"   cmp %0, %2\n"
			"   blo 1b" :: "r"(t),"r"(delay_ns_per_tick),"r"(ns));
}

/**
 * @brief:  main()
 *
 * @param:  void
 * @return: -
 */
timer_tt* new_timer(void) {
	return (timer_tt*) malloc(sizeof(timer_tt));
}

/**
 * @brief:  main()
 *
 * @param:  void
 * @return: -
 */
int set_timer(timer_tt* timer_pointer, int time_ms) {
	timer_pointer->sys_time = sys_time + time_ms;
	timer_pointer->set_sys_time = time_ms;
	return 0;
}
/**
 * @brief:  main()
 *
 * @param:  void
 * @return: -
 */
int get_timer(timer_tt* timer_pointer) {
	if (sys_time < timer_pointer->sys_time) {
		return 0;
	} else {
		return 1;
	}

}

/**
 * @brief:  main()
 *
 * @param:  void
 * @return: -
 */
int del_timer(timer_tt* timer_pointer) {
	free(timer_pointer);
	return 0;
}

button_tt* new_button_mikky(void) {

	button_tt* button = (button_tt*) malloc(sizeof(button_tt));
	button->current_state = 0;
	button->flags = 0;
	button->last_state = 0;
	button->timer = new_timer();

	return (button);
}

int state_button_mikky(button_position but_pos, button_tt* button) {
	timer_tt* my_timer = button->timer;
	register int flags_mask = 0;
	register int flags = button->flags;

	switch (button->current_state) {

	case depressed:

		if (but_pos == bottom_button || but_pos == central_button
				|| but_pos == top_button) {
			button->current_state = pressed;
		}
		button->last_state = depressed;
		flags = flags & 0b110;

		break;

	case pressed:
		if (button->last_state == depressed) {
			set_timer(my_timer, 10);
		}

		if (but_pos == bottom_button || but_pos == central_button
				|| but_pos == top_button) {
			if (get_timer(my_timer) == 1) {
				button->current_state = long_pressed;
				flags_mask = 1;
			}
		} else {
			button->current_state = depressed;
		}

		button->last_state = pressed;
		break;

	case long_pressed:
		if (button->last_state == pressed) {
			set_timer(my_timer, 10);
		}
		if (but_pos == bottom_button || but_pos == central_button
				|| but_pos == top_button) {
			if (get_timer(my_timer) == 1) {
				button->current_state = hold;
				flags_mask = 2;
			}
		} else {
			button->current_state = depressed;
		}

		button->last_state = long_pressed;
		break;

	case hold:
		if (button->last_state == long_pressed) {
			set_timer(my_timer, 1);
		}
		if (but_pos == bottom_button || but_pos == central_button
				|| but_pos == top_button) {
			button->current_state = hold;
		} else {
			button->current_state = depressed;
		}

		if (get_timer(my_timer) == 1) {
			flags_mask = 4;
			set_timer(my_timer, 5);
		}

		button->last_state = hold;
		break;
	}
	button->flags = flags | flags_mask;
	return 0;
}

#define DEBUG
#ifndef DEBUG
typedef struct {
	button_position button_pos;
	button_tt button;
	uint8_t time;
}t_debug;

t_debug debug[]= {
	{	top_button, {1, pressed, long_pressed, {20, 30, 40}}, 2},
	{	central_button, {1, long_pressed, depressed, {20, 30, 40}}, 5},
	{	bottom_button, {1, hold, pressed, {20, 30, 40}}, 10}
};

void my_unity_test(button_tt* but[]) {
	button_position button_pos;
	erase();
	uart_tx("\r\nBUTTOM            TESTING STATE         OUTPUT STATE        RESULT", 1);

	char position;
	for (position = 0; position < 3; position++) {

		if (!(*btn[position].port & btn[position].msk)) {
			state_button_mikky(position, but[position]);
			if(debug[position].button.current_state == pressed && but[position]->current_state == pressed)
			uart_tx("\r\ntop button        single press          press               positive", 1);
			else
			uart_tx("\r\ntop button        single press          not single press    negative", 1);
			if(debug[position].button.current_state == long_pressed && but[position]->current_state == long_pressed)
			uart_tx("\r\ncentral button    long press            long press          positive", 1);
			else
			uart_tx("\r\ncentral button    long press            not long press      negative", 1);
			if(debug[position].button.current_state == hold && but[position]->current_state == hold)
			uart_tx("\r\nbottom button     hold                  hold                positive", 1);
			else
			uart_tx("\r\nbottom button     hold                  not hold            negative", 1);
		}
		else {
			but[position]->current_state = depressed;
		}
	}
}
#endif DEBUG

/**
 * @brief:  main()
 *
 * @param:  void
 * @return: -
 */

int main() {

	//mask = 0;
	setup();
	initUART();
	setup_SPI();
	uart_tx("\r\nSECCESFULL!!!", 1);

	while (1) {

	}		// while 1*/
}

/////////////////////////////////////////////////////////////////////////////////
//                             SETUP
/////////////////////////////////////////////////////////////////////////////////

/**
 * @brief:  setup()
 *          This function sets up main settings
 *
 * @param:  void
 * @return: void
 */

void setup(void) {
	SCB->AIRCR = AIRCR_VECTKEY_MASK | NVIC_PriorityGroup_1;
	RCC->CFGR &= ~(RCC_CFGR_ADCPRE);

	RCC->APB1ENR = 0;
	RCC->APB2ENR = 0;

	RCC->AHBENR |= RCC_AHBENR_DMA1EN;

	RCC->APB1ENR = RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_DACEN
			| RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;
	RCC->APB2ENR = RCC_APB2ENR_ADC1EN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN
			| RCC_APB2ENR_IOPCEN | RCC_APB2ENR_USART1EN | RCC_APB2ENR_TIM15EN
			| RCC_APB2ENR_AFIOEN;

	AFIO->MAPR = AFIO_MAPR_TIM2_REMAP_FULLREMAP | AFIO_MAPR_SWJ_CFG_JTAGDISABLE
			| AFIO_MAPR_SPI1_REMAP //my XXX
//|AFIO_MAPR_USART1_REMAP // debug brd
	;
	AFIO->MAPR2 = AFIO_MAPR2_TIM15_REMAP;

	NVIC_SetPriority(DMA1_Channel1_IRQn,
			NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, 1));		//adc
	NVIC_SetPriority(DMA1_Channel4_IRQn,
			NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));		//dac
	NVIC_SetPriority(TIM1_BRK_TIM15_IRQn,
			NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));	//sequencer

	GPIOA->CRL = GPIO_CRL_MODE3_1;	// analog switch
	GPIOA->CRH =
	GPIO_CRH_CNF10_0 //usart rx
//|GPIO_CRH_CNF8_1|GPIO_CRH_MODE8_1 // MCO
	| GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9_1 // usart tx
			| GPIO_CRH_MODE15_1 // backlight
	;
//	|GPIO_CRH_CNF15_1|GPIO_CRH_MODE15_1 // TIM2 CH1
// MCO : |GPIO_CRH_MODE8_0|GPIO_CRH_CNF8_1	RCC->CFGR |= RCC_CFGR_MCO_SYSCLK;
//RCC->CFGR |= RCC_CFGR_MCO_HSE;
//RCC->CFGR |= RCC_CFGR_MCO_SYSCLK;

	uint32_t a_pll = (RCC->CFGR
			& (~(RCC_CFGR_SW | RCC_CFGR_SWS | RCC_CFGR_PPRE2)))
			| (uint32_t) RCC_CFGR_SW_PLL;
	uint32_t a_hse = (RCC->CFGR
			& (~(RCC_CFGR_SW | RCC_CFGR_SWS | RCC_CFGR_PPRE2)))
			| (uint32_t) RCC_CFGR_SW_HSE | RCC_CFGR_PPRE2_DIV16;

	RCC->APB2RSTR |= RCC_APB2RSTR_ADC1RST;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_ADC1RST;

	GPIOA->BRR = GPIO_Pin_0; // GUARD ON
	GPIOA->BRR = GPIO_Pin_3; // ANALOG ON

	GPIOB->CRL =
	GPIO_CRL_CNF7_1  // BUTTON 1 PU
	| GPIO_CRL_MODE3_1 //SCK
			| GPIO_CRL_MODE4_1 //MOSIF
			| GPIO_CRL_MODE5_1 // CS
			| GPIO_CRL_MODE6_1 //RES
			| GPIO_CRL_CNF3_1 | GPIO_CRL_CNF4_1 | GPIO_CRL_CNF5_1
//			|GPIO_CRL_CNF1_1|GPIO_CRL_MODE1 // tim3 ch4
//    		|GPIO_CRL_CNF3_1|GPIO_CRL_MODE3 // tim2 ch2
//			|GPIO_CRL_CNF6_1 | GPIO_CRL_MODE6_1 // usart tx
			;
	GPIOB->CRH =
	GPIO_CRH_CNF8_1 // BUTTON 2
	| GPIO_CRH_MODE13_1 // diag 1
			| GPIO_CRH_MODE14_1 // diag 2
	;

	GPIOB->BSRR = GPIO_Pin_8 | GPIO_Pin_7; // BUTTON 1,2 PU
	GPIOB->BRR = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_13
			| GPIO_Pin_14; // 0 diag 1,2 outputs

	GPIOC->CRL = 0;
	GPIOC->CRH = GPIO_CRH_CNF13_1; // BUTTON 3
	GPIOC->BSRR = GPIO_Pin_13; // BUTTON 3 PU

	TIM15->PSC = MAIN_PRESCALER_500K - 1; // prescale
	TIM15->ARR = cht[0];
	TIM15->CR1 = 0; //TIM_CR1_ARPE;
	TIM15->CR2 = TIM_CR2_MMS_2 | TIM_CR2_MMS_0; // TRGO trigger = oc2
	TIM15->CCR1 = ADC_CHANGEOVR / 4;
	TIM15->CCR2 = ADC_CHANGEOVR;
	TIM15->CCR3 = 0;
	TIM15->CCR4 = 0;
	TIM15->SMCR = 0;
	TIM15->CCMR1 = TIM_CCMR1_OC1M | TIM_CCMR1_OC1PE | TIM_CCMR1_OC2M; //| TIM_CCMR1_OC2PE ;
	TIM15->CCMR2 = 0;
	TIM15->CCER = TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC2E | TIM_CCER_CC2P;
	TIM15->DIER = TIM_DIER_CC1IE;
	TIM15->EGR = 0;
	TIM15->BDTR = 0;
//TIM_BDTR_AOE|TIM_BDTR_MOE;

	TIM2->PSC = 0;
	TIM2->ARR = MAIN_PRESCALER_500K - 1;
	TIM2->CR1 = TIM_CR1_ARPE;
	TIM2->CR2 = TIM_CR2_MMS_1; //TIM_CR2_MMS_2|TIM_CR2_MMS_1; update
	TIM2->CCR1 = 1;
	TIM2->CCR2 = 1;
	TIM2->CCR3 = 0;
	TIM2->CCR4 = 0;
	TIM2->SMCR = TIM_SMCR_SMS_2 | TIM_SMCR_SMS_0 | TIM_SMCR_TS_0;
	TIM2->CCMR1 = TIM_CCMR1_OC1M | TIM_CCMR1_OC1PE | TIM_CCMR1_OC2M
			| TIM_CCMR1_OC2PE;
	TIM2->CCMR2 = TIM_CCMR2_CC3S;
	TIM2->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3P;
	TIM2->DIER = 0;
	TIM2->EGR = 0;

	TIM3->PSC = 0;
	TIM3->ARR = MAIN_PRESCALER_500K - 1;
	TIM3->CR1 = TIM_CR1_ARPE;
	TIM3->CR2 = TIM_CR2_MMS_2; // TRGO trigger = oc1  = DAC TRIGGER
	TIM3->CCR1 = MAIN_PRESCALER_500K - 1;
	TIM3->CCR2 = 0;
	TIM3->CCR3 = 0;
	TIM3->CCR4 = 0; //32
	TIM3->SMCR = TIM_SMCR_SMS_2 | TIM_SMCR_TS_0; // reset on tim2 upd
	TIM3->CCMR1 = TIM_CCMR1_OC1M | TIM_CCMR1_OC1PE;
	TIM3->CCMR2 = TIM_CCMR2_OC4M | TIM_CCMR2_OC4PE;
	TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC4E;
	TIM3->DIER = 0;
	TIM3->EGR = 0;

	DAC->CR = DAC_CR_TEN1 | DAC_CR_TEN2 | DAC_CR_TSEL2_0
			| DAC_CR_TSEL1_0  //tim3_trgo
			| DAC_CR_BOFF1 | DAC_CR_BOFF2 | DAC_CR_WAVE1_1 | DAC_CR_MAMP1_1
			| DAC_CR_MAMP1_0 | DAC_CR_WAVE2_1 | DAC_CR_MAMP2_1 | DAC_CR_MAMP2_0
	;

	DMA1_Channel4->CCR = 0;
	DMA1_Channel4->CPAR = (uint32_t) &DAC->DHR12RD;
	DMA1_Channel4->CMAR = (uint32_t) dac_buf;  //sine;
	DMA1_Channel4->CNDTR = DAC_N;
	DMA1_Channel4->CCR = DMA_CCR1_MINC | DMA_CCR1_CIRC | DMA_CCR1_DIR
			| DMA_CCR1_PL_1 |
			DMA_MemoryDataSize_Word | DMA_PeripheralDataSize_Word
			| DMA_CCR1_HTIE | DMA_CCR1_TCIE;
	DMA1_Channel4->CCR |= DMA_CCR1_EN;

	DMA1_Channel1->CCR = 0;
	DMA1_Channel1->CPAR = (uint32_t) &ADC1->DR;
	DMA1_Channel1->CMAR = (uint32_t) adc_dma;
	DMA1_Channel1->CNDTR = N;
	DMA1_Channel1->CCR = DMA_CCR1_MINC | DMA_CCR1_PL_0 | DMA_CCR1_CIRC |
	DMA_MemoryDataSize_HalfWord | DMA_PeripheralDataSize_HalfWord
			| DMA_CCR1_TCIE | DMA_CCR1_HTIE
	;
	DMA1_Channel1->CCR |= DMA_CCR1_EN;

	ADC1->CR1 = ADC_CR1_DISCEN;
	ADC1->CR2 = ADC_CR2_EXTSEL_0 | ADC_CR2_EXTSEL_1 | ADC_CR2_EXTTRIG
			| ADC_CR2_DMA | ADC_CR2_TSVREFE; // tim2 - cc2

	ADC1->SMPR1 = ADC_SMPR1_SMP16 | ADC_SMPR1_SMP17;
	ADC1->SMPR2 = ADC_SMPR2_SMP8 | ADC_SMPR2_SMP7_1 | ADC_SMPR2_SMP2_1
			| ADC_SMPR2_SMP1_1;
	ADC1->SQR1 = 0;
	ADC1->SQR3 = 1;

	ADC1->CR2 |= ADC_CR2_ADON;
	ADC1->CR2 |= ADC_CR2_RSTCAL;
	while ((ADC1->CR2 & ADC_CR2_RSTCAL) == ADC_CR2_RSTCAL) {
		;
	}
	ADC1->CR2 |= ADC_CR2_CAL;
	while ((ADC1->CR2 & ADC_CR2_CAL) == ADC_CR2_CAL) {
		;
	}

	DAC->CR |= DAC_CR_EN1 | DAC_CR_EN2;
	DAC->CR |= DAC_CR_DMAEN2;

	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);

	TIM2->CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
	TIM3->CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
// sync to hse
	RCC->CFGR = a_hse;
	RCC->CFGR = a_pll;
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	RCC->CFGR = a_hse;
	RCC->CFGR = a_pll;
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	RCC->CFGR = a_hse;
	RCC->CFGR = a_pll;
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
// sync to hse
	TIM15->CR1 = TIM_CR1_CEN;

	GPIOA->BSRR = GPIO_Pin_15; // BACKLIGHT ON
}

void erase(void) {
	uart_tx("\f", 1);
	//for(int item = 0; item < 20; item++) uart_tx("\r\b", 1);
}

void print_to_consol_cplx_x(cplx* ptr) {
	int var_Re = ptr->Re;
	int var_Im = ptr->Im;
	char str1[10];
	char str2[10];
	itoa(var_Re, str1, 10);
	itoa(var_Im, str2, 10);
	uart_tx("\r\n", 1);
	uart_tx(str1, 1);
	uart_tx("\r\n", 1);
	uart_tx(str2, 1);
}

/**
 * @brief:  print_to_consol_int()
 *          This function I have written for myself.
 *          It prints to console variable which has type int.
 *
 * @param:  * ptr
 * @return: void
 */

void print_to_consol_int(int* ptr) {
	char str[10];
	itoa(*ptr, str, 10);
	//uart_tx("\r\n", 1);
	uart_tx(str, 1);
	//uart_tx("\r\n", 1);
}

/////////////////////////////////////////////////////////////////////////////////
//                             IRQ/DMA HANDLERs
/////////////////////////////////////////////////////////////////////////////////

volatile int k = 0;
volatile long long mreal[CH_NUM];
volatile long long mimag[CH_NUM];

void TIM1_BRK_TIM15_IRQHandler(void) {
}

void __attribute__((optimize("-O3"))) DMA1_Channel1_IRQHandler(void) {
}

void __attribute__((optimize("-O3"))) DMA1_Channel4_IRQHandler(void) {
}
