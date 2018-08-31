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
#include "n1110.h"
#include "main.h"
#include "eeprom.h"

#include "measurements.h"
#include "arithmetic.h"

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

const uint32_t cht[CH_NUM] = {
ADC_N * OSR + ADC_CHANGEOVR - 1 + N / 2,
ADC_N * OSR + ADC_CHANGEOVR - 1,
ADC_N * OSR + ADC_CHANGEOVR - 1,
ADC_N + ADC_CHANGEOVR - 1,
ADC_N + ADC_CHANGEOVR - 1 }; // + VBAT, TEMP, VREF

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
uint64_t  time_accumulator[100];  //my
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

const buttons_t btn[BTN_CNT] = { { &GPIOC->IDR, GPIO_Pin_13 }, { &GPIOB->IDR,
GPIO_Pin_8 }, { &GPIOB->IDR, GPIO_Pin_7 } };

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

uint32_t dac_buf[DAC_N];

uint16_t __attribute__ ((section (".noinit"))) adc_dma[N];
char __attribute__ ((section (".noinit"))) tmpStr[20]; // temp string

int __attribute__ ((section (".noinit"))) var1; //my test
int __attribute__ ((section (".noinit"))) var2; //my test
my_time time1;

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

void printBat(int x, int y, int percent);
uint32_t setBf(int idx, uint32_t val);
uint32_t getBf(int idx);


void resetBalanceData(void);

void puthex(int inp, short size);
void sputdec(char * buf, int inp);
void printFloat(int x, int y, float num, char * suffix);
void printSmallFloat(float num, char * suffix);

void powerOff(void);
void waitz(float lim);
void waitOpen(void);
void waitClose(void);
void print_openshort(int t);
void calibrate(void);
void balance(void);
unsigned int processButton(unsigned int state);
int buttons(void);
void printSecondary(int x, int y, int idx);

void setBacklight(int mode);
void setup(void); // my test
void table(void);
void erase(void);


/////
typedef struct {
	uint64_t sys_time;
	int set_sys_time;
	uint32_t push_time; // my test
} timer_tt;

typedef struct {
	int flags;
	button_state_t current_state;
	button_state_t last_state;
	timer_tt * timer;
} button_tt;

state_toggle toggle = off;
button_position button;

//void my_unity_test(button_position, button_tt* );
void my_unity_test(button_tt**);

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
				       { leafProcess,     NULL,          MENU_MAIN,     0,            0,   "MEAS DISPLAY",      -1 },  //0
				       { menuProcess,     NULL,          MENU_DISP,     2,           13,   "MAIN MENU",        200 },   //1
				       { menuProcess,     editContrast,  MENU_MAIN,     0,            0,   "CONTRAST",          -1 }, //2
				       { menuProcess,     editScr,       MENU_MAIN,     0,            0,   "DISP MODE", -1 },  //3
				       { menuProcess,     editBF,        MENU_MAIN,    BIT_BL_MODE,   0, "BACKLIGHT",-1 },   //4
				       { menuProcess,     editAverages,  MENU_MAIN,     0,            0, "AVERAGES", -1 }, //5
				       { menuProcess,     editR,         MENU_MAIN,     0,            0, "SHUNT R", -1 },   //6
				       { menuProcess,     editBF,        MENU_MAIN,    BIT_LC_MODE,   0, "LC EQUIV", -1 }, //7
				       { menuProcess,     editBF,        MENU_MAIN, BIT_LIM_RANGE,    0, "RANGE LIM",	-1 },   //8
				       { menuProcess,     editBF,        MENU_MAIN, BIT_1STPARAM,     0, "PARAM 1", -1 }, //9
				       { menuProcess,     editBF,        MENU_MAIN, BIT_2NDPARAM,     0, "PARAM 2", -1 }, //10
				       { menuProcess,     editBF,        MENU_MAIN, BIT_3RDPARAM,     0, "PARAM 3", -1 }, //11
				       { menuProcess,     editBF,        MENU_MAIN, BIT_UART_MODE,    0, "UART OUT", -1 },   //12
				       { menuProcess,     editBF,        MENU_MAIN, BIT_LOWBAT,       0, "LOW BAT", -1 }, //13
				       { menuProcess,     NULL,          MENU_DISP,      15,         17, "CALIBRATION", 200 }, //14
				       { leafBalance,     editBalance,   MENU_DISP,      15,          0, "CH BALANCE", -1 }, //15
				       { leafCal,         editCal,       MENU_DISP,       0,          0, "OPEN-SHORT", -1 },   //16
				       { leafCalReset,    NULL,          MENU_CAL,        0,          0, "RESET", -1 },   //17
				       { leafInfo,        NULL,          MENU_DISP,       0,          0, "Info", 20 },   //18
				       { leafScr,         NULL,          MENU_DISP,       0,          0, "scrMode", -1 },   //19
				       { leafBatLow,      NULL,          MENU_DISP,       0,          0, "lowBat", 20 },   //20
				       { diagScr,         NULL,          MENU_DISP,       0,          0, "diagScr", -1 },   //21
		};

///==========================================================================================================================
///==========================================================================================================================
///==========================================================================================================================

#define DELAY_OVERHEAD (18 * (1000000000 / 24000000)) // Daumenpeilung

static unsigned delay_ns_per_tick = 1;

void
delay_ns(unsigned ns)
{
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

		if (but_pos == bottom_button || but_pos == central_button || but_pos == top_button) {
			button->current_state = pressed;
		}
		button->last_state = depressed;
		flags = flags & 0b110;

		break;

	case pressed:
		if (button->last_state == depressed) {
			set_timer(my_timer, 10);
		}

		if (but_pos == bottom_button || but_pos == central_button || but_pos == top_button) {
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
		if (but_pos == bottom_button || but_pos == central_button || but_pos == top_button) {
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
		if (but_pos == bottom_button || but_pos == central_button || but_pos == top_button) {
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
                 } t_debug;

t_debug debug[]=  {
                 	{top_button,         {1,  pressed,         long_pressed,    {20, 30, 40}},    2},
                 	{central_button,     {1,  long_pressed,    depressed,       {20, 30, 40}},    5},
                 	{bottom_button,      {1,  hold,            pressed,         {20, 30, 40}},    10}
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

	mask = 0;
	setup();
	initUART();

	// my SPI test

	setup_SPI();
	uart_tx("\r\nSECCESFULL!!!", 1);
	//SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);

	/*uart_tx("\r\n", 1);

	uart_tx("Hello, ARM11\n", 1);

	uart_tx("\r\n", 1);
	uart_tx(VER, 1);
	uart_tx("\r\n", 1);*/
	//table();
	button_tt* bottom_but =  new_button_mikky();
	button_tt* central_but = new_button_mikky();
	button_tt* top_but = new_button_mikky();

	button_tt* but[3] = {bottom_but, central_but, top_but};

	base.Re = 0.0;
	base.Im = 0.0;

	mask = 0x00;

// This function sets up EEPROM
	eepromInit();

	R.Re = SHUNT;
	R.Im = 0.00;

	bitParams = 0;

	setBf(BIT_SCR_CONTRAST, DEF_CONTRAST);
	setBf(BIT_AVERAGES, DEF_AVERAGES);
	setBf(BIT_BL_MODE, 1);	//BL on

	eepromLoadParam(EEP_PARAMS1, &bitParams);

	R.Re = SHUNT;
	R.Im = 0.00;

	eepromLoadParam(EEP_SHUNT, &R);

	if ((apo > APO_MAX) || (apo <= 0))
		apo = APO_4MIN;

	if (findex < 0 || findex >= FCNT)
		findex = 0;
	freq = flist[findex];
	gain = fgain[findex];

	fillSine(freq);

	lcd_init(getBf(BIT_SCR_MODE));
	lcd_setcontrast(getBf(BIT_SCR_CONTRAST));
	setBacklight(getBf(BIT_BL_MODE));
	lcd_clear();

	measure(&Z, 1);

	sign = -1;
	swap_dacs = 0;

	menuCurrent = MENU_INFO;
	startTime = hundredMsTick;

	if (vbat < iLowBat[getBf(BIT_LOWBAT)])
		menuCurrent = MENU_LOWBAT;

//	int xstep=0,zidx = 0;

	while (1) {

		btnState = buttons();

		//asm ( "mov %1, %0" : "=r" (sys_time) : "r" (sys_time_counter));
		__disable_irq();
		sys_time = sys_time_counter;
		__enable_irq();

		//my_unity_test(but);

		//last_state = state_button(button_1, last_state);

		//GPIOA->BSRR = GPIO_Pin_15;
		//table();
		//GPIOA->BSRR = GPIO_Pin_15 << 16;

		if (btnState)
			startTime = hundredMsTick;

		if ((startTime + apo) < hundredMsTick)
			powerOff();

		int menuNew = menu[menuCurrent].m(btnState);

		if (menuNew != menuCurrent) {
			startTime = hundredMsTick;
			menuCurrent = menuNew;
			if ((menuCurrent < 0) || (menuCurrent > MENU_ITEM_COUNT))
				menuCurrent = 0;
			lcd_clear();
		}
		setBacklight(getBf(BIT_BL_MODE));

		measure(&Z, 1);

		if ((cstatus & (1 << findex))) {
			cplx tmp1, tmp2;
			tmp1.Re = cal.Zs.Re - Z.Re;
			tmp1.Im = cal.Zs.Im - Z.Im;

			tmp2.Re = cal.Zo.Re;
			tmp2.Im = cal.Zo.Im;

			cplxMul(&tmp2, &tmp1);

			tmp1.Re = Z.Re - cal.Zo.Re;
			tmp1.Im = Z.Im - cal.Zo.Im;

			cplxDiv(&tmp2, &tmp1);

			Z.Re = tmp2.Re;
			Z.Im = tmp2.Im;
		}

		Z.Re = filter(0, Z.Re);
		Z.Im = filter(1, Z.Im);

	}		// while 1*/
}


/**
 * @brief:  table()
 *          This function draws menu table during IRQ.
 *
 * @param:  void
 * @return: void
 */

void table(void) {
	static const char menu_item[][AMAUNT] = { "CONTRAST",   "DISP MODE",   "BACKLIGHT",   "AVERAGES",   "SHUNT R",
			                           "LC EQUIV",   "RANGE LIM",   "PARAM 1",     "PARAM 2",    "PARAM 3",
					                   "UART OUT",   "LOW BAT",     "CALIBRATION", "CH BALANCE", "OPEN-SHORT",
					                   "RESET",      "Info",        "scrMode",     "lowBat",     "diagScr"
	                                 };
	char count = 0;
    uart_tx("\r\t\tMAIN MENU\n", 1);
    for (int item = 0; item < AMAUNT; item++) {
    	uart_tx("\r\t", 1);
    	uart_tx(menu_item[item], 1);
    	uart_tx("\n", 1);
    }
    while(count < AMAUNT+5) {
    	uart_tx("\r\b", 1);
    	++count;
    }
}


/////////////////////////////////////////////////////////////////////////////////
//                             MENU HANDLER
/////////////////////////////////////////////////////////////////////////////////

/**
 * @brief:  menuProcess()
 *          This function defines menu process
 *
 * @param:  void
 * @return: menuCurrent
 */

int menuProcess(int btns) {

	static int selected = 0;
	static int editable = 0;

	if ((selected < menu[menuCurrent].first)
			|| (selected > menu[menuCurrent].last)) {
		selected = menu[menuCurrent].first;
		editable = 0;
	}

	if (editable == 0) {
		if ((btns) & ((BTN_PUSH | BTN_LPUSH | BTN_RPT) << SP_BTN)) {
			selected--;
		} // down
		if ((btns) & ((BTN_PUSH | BTN_LPUSH | BTN_RPT) << REL_BTN)) {
			selected++;
		} // up
	} else {
		menu[selected].d(selected, tmpStr, btns);
	}

	if (selected > menu[menuCurrent].last)
		selected = menu[menuCurrent].last;

	int min = selected - 1;
	int max = menu[menuCurrent].last;

	if ((min + MAX_MENU_ITEMS - 1) > max)
		min = max - MAX_MENU_ITEMS + 1;
	if (min < menu[menuCurrent].first)
		min = menu[menuCurrent].first;

	if (max > min + MAX_MENU_ITEMS - 1)
		max = min + MAX_MENU_ITEMS - 1;
	if (max > menu[menuCurrent].last)
		max = menu[menuCurrent].last;

	if (menu[menuCurrent].parent >= 0) {
		lcd_gotoxy(0, 0);
		lcd_putstr(menu[menuCurrent].name, 1);
	}

	lcd_gotoxy(6 * 14, 0);
	if (min != menu[menuCurrent].first)
		lcd_putstr("<", 0);
	else
		lcd_putstr(" ", 0);
	if (max != menu[menuCurrent].last)
		lcd_putstr(">", 0);
	lcd_putstr("", 1);

	for (int i = 0; i <= (max - min); i++) // display items
			{
		mask = 0x00;
		//if( (min  + i + 1) == selected) mask = 0x80;

		lcd_gotoxy(0, i + 2);
		if (selected == (min + i))
			mask = 0xFF;

		lcd_putstr(menu[min + i].name, 1);

		if (menu[min + i].d != NULL) {
			menu[min + i].d(min + i, tmpStr, 0);
			if ((selected == (min + i)) && editable)
				mask = 0x00;
			lcd_gotoxy(6 * (16 - strlen(tmpStr)), i + 2);
			lcd_putstr(tmpStr, 1);
		}
	}

	mask = 0x00;

	if ((btns) & BTN_PUSH) // menu select
	{

		if (editable == 0) {
			if (menu[selected].d != NULL) {
				if (menu[selected].d(selected, tmpStr, 0)) {
					return selected;
				} else {
					editable = 1;
				}
			} else
				return selected;
		} else
			editable = 0;
	}

	if ((editable == 0)
			&& (((btns) & BTN_LPUSH)
					| (menu[menuCurrent].timeout < (hundredMsTick - startTime)))) {
		return menu[menuCurrent].parent;
	} // menu select
	return menuCurrent;

}

//----------------------------------------------------------------------------
int leafProcess(int btns) {
	static cplx sum;
	static int round = 0;
	int averages = getBf(BIT_AVERAGES);
	float ls, cs, d, d2, q, zlimit;

	sum.Re += Z.Re;
	sum.Im += Z.Im;
	round++;

	if (round >= averages) {

		Z.Re = sum.Re / averages;
		Z.Im = sum.Im / averages;

		if (getBf(BIT_REL_MODE)) {
			Z.Re -= base.Re;
			Z.Im -= base.Im;
		} else {
			base.Re = Z.Re;
			base.Im = Z.Im;
		}

		ls = Z.Im / 6.283185307 / (freq * 1000.0);
		cs = -1 / 6.283185307 / (freq * 1000.0) / Z.Im;

		d = Z.Re / Z.Im;
		if (d < 0)
			d = -d;
		d2 = d * d;
		q = 1 / d;

		lcd_gotoxy(13 * 6, 2); //13*6
		lcd_putstr(sLCMode[getBf(BIT_LC_MODE)], 0);
		if (getBf(BIT_LC_MODE) == 1) // par
				{
			ls = ls * (1 + d2);
			cs = cs / (1 + d2);
			Z.Re = Z.Re * (1.0 + d2) / d2;
		}

		zlimit = rlim[getBf(BIT_LIM_RANGE)];

		mdata.Zpolar.Re = square(Z.Re * Z.Re + Z.Im * Z.Im);
		mdata.Zpolar.Im = 180.0 * atan2f(Z.Im, Z.Re) / M_PI;

		mdata.Q = q;
		mdata.D = d;
//	mod = square(Z.Re*Z.Re + Z.Im*Z.Im);

		printSecondary(0, 0, getBf(BIT_1STPARAM));
		printSecondary(0, 1, getBf(BIT_2NDPARAM));
		printSecondary(0, 2, getBf(BIT_3RDPARAM));

		if (mdata.Zpolar.Re < zlimit) {
			printFloat(0, 3, Z.Re, "\\");

			if (Z.Im > 0) {
				printFloat(0, 6, ls, "H");
			} else {
				printFloat(0, 6, cs, "F");
			}
		} else {
			for (int i = 3; i < 8; i++) {
				lcd_gotoxy(0, i);
				lcd_putstr("      \\", 1);
			}
			lcd_gotoxy(0, 8);
			lcd_putstr(" ", 1);
			lcd_gotoxy(0, 4);
			lcd_putstr("OUT OF RANGE", 1);
		}

		if (getBf(BIT_UART_MODE)) {
			uart_tx("\r\n", 1);
		}

		sum.Re = 0;
		sum.Im = 0;
		round = 0;

	}

	printBat(6 * 13 - 2, 0,
			(((int) vbat - BAT_MIN) * 100) / (BAT_MAX - BAT_MIN));

	lcd_gotoxy(13 * 6, 3);
	lcd_putstr(sFreqs[findex], 0);

	lcd_gotoxy(13 * 6, 4);
	if ((cstatus & (1 << (findex + 8))))
		lcd_putstr("BAL", 0);
	else
		lcd_putstr("---", 0);

	lcd_gotoxy(13 * 6, 5);
	if ((cstatus & (1 << findex)))
		lcd_putstr("CAL", 0);
	else
		lcd_putstr("---", 0);

	lcd_gotoxy(13 * 6, 6);
	if ((apo - hundredMsTick + startTime) < APO_4MIN)
		lcd_putstr("APO", 0);
	else
		lcd_putstr("   ", 0);

	lcd_gotoxy(13 * 6, 7);
	if (getBf(BIT_REL_MODE))
		lcd_putstr(">-<", 0);
	else
		lcd_putstr("   ", 0);

	if (vbat < iLowBat[getBf(BIT_LOWBAT)])
		return MENU_LOWBAT;

	if (btns & (BTN_PUSH << SP_BTN)) {
		findex++;
		if (findex >= FCNT)
			findex = 0;
		freq = flist[findex];
		gain = fgain[findex];
		fillSine(freq);
		setBf(BIT_REL_MODE, 0); //reset rel mode on freq switch
	}
	if (btns & (BTN_LPUSH << SP_BTN)) {
		return MENU_CAL;
	}

	if (btns & (BTN_PUSH << REL_BTN)) {
		setBf(BIT_REL_MODE, getBf(BIT_REL_MODE) - 1);
	}
	if (btns & (BTN_LPUSH << REL_BTN)) {
		setBacklight((getBf(BIT_BL_MODE) + 1) & 1);
	}

	if (btns & (BTN_PUSH << PWR_BTN)) {
		return menu[menuCurrent].parent;
	} // menu select
	if (btns & (BTN_LPUSH << PWR_BTN))
		powerOff();

	return menuCurrent;
}

/**
 * @brief:  leafBatLow()
 *          This function turns off RLC device if power charge is low
 *
 * @param:  btns
 * @return: menuCurrent or MENU_DISP
 */
int leafBatLow(int btns) {
	lcd_gotoxy(2 * 6, 4);
	lcd_putstr("Battery low!", 0);

	//print_to_consol_int(&btns); // my test

	if ((menu[menuCurrent].timeout < (hundredMsTick - startTime))) {
		powerOff();
	} // menu select

	if (btns & (BTN_PUSH << REL_BTN)) {
		setBf(BIT_LOWBAT, 0);
		return (MENU_DISP);
	}

	return menuCurrent;
}

/**
 * @brief:  leafInfo()
 *          This function shows greeting information on LCD.
 *          Menu current state will be show on LCD
 *
 * @param:  btns
 * @return:
 */

int leafInfo(int btns) {

	/*lcd_gotoxy(0,0);lcd_putstr( VER ,1);
	 lcd_gotoxy(0,2);lcd_putstr("   Neekeetos    ",0);
	 lcd_gotoxy(0,3);lcd_putstr("   @yahoo.com   ",0);
	 lcd_gotoxy(0,4);lcd_putstr(" big thanks to  ",0);
	 lcd_gotoxy(0,5);lcd_putstr(" TESLight, Link ",0);
	 lcd_gotoxy(0,6);lcd_putstr("Ozzy & radiocats",0);
	 lcd_gotoxy(0,7);lcd_putstr(" @ radiokot.ru  ",0);*/
	lcd_gotoxy(7, 7);
	lcd_putstr(" Hello, ARM11!  ", 0); // waitz(30);// my test

	if ((menu[menuCurrent].timeout < (hundredMsTick - startTime))) {
		return menu[menuCurrent].parent;
	} // menu select

	if (btns & (BTN_PUSH << PWR_BTN)) {
		return menu[menuCurrent].parent;
	}
	if (btns & (BTN_PUSH << REL_BTN)) {
		return (MENU_SCR);
	}
	if (btns & (BTN_PUSH << SP_BTN)) {
		return (MENU_DIAG);
	}

	return menuCurrent;
}

/**
 * @brief:  leafScr()
 *          This function doesn't call in the program
 *
 * @param:  btns
 * @return: menuCurrent
 */

int leafScr(int btns) {

	lcd_gotoxy(0, 0);
	lcd_putstr("TOP LEFT", 1);
	lcd_gotoxy(0, 4);
	lcd_putstr("screen mode set ", 0);
	lcd_gotoxy(0, 7);
	lcd_putstr("    BOTTOM RIGHT", 0);

	editScr(0, tmpStr, btns);

	if (btns & (BTN_PUSH << PWR_BTN)) {
		eepromSaveParam( EEP_PARAMS1, &bitParams, sizeof(bitParams));
		return menu[menuCurrent].parent;
	} // menu select

	return menuCurrent;
}

//----------------------------------------------------------------------------

int diagScr(int btns) {

	static int round = 0;
	int averages = getBf(BIT_AVERAGES);
	static int ptr = 0;
	static int sigma = 0, rel = 1;
	static float V1[MS], V2[MS], V7[MS], A1[MS], A2[MS], A7[MS];
	float angle1, angle2, angle7;

	lcd_gotoxy(0, 0);
	lcd_putstr("DIAG ", 0);
	lcd_putstr(sFreqs[findex], 0);
	if (sigma)
		lcd_putstr("   SIGMA", 1);
	else
		lcd_putstr("    MEAN", 1);
	lcd_gotoxy(6 * 13, 1);
	if (rel)
		lcd_putstr("REL", 1);
	else
		lcd_putstr("    ", 1);

	angle1 = mdata.polar[0].Im;
	angle2 = mdata.polar[1].Im;
	angle7 = mdata.polar[2].Im - 180;

	if (angle1 < 0) {
		angle1 += 180;
		angle2 += 180;
		angle7 += 180;
	}
	if (angle7 < 0) {
		angle7 += 360;
	}

	if (rel) {
		angle2 = angle2 - angle1;
		angle7 = angle7 - angle1;

		V1[ptr] = 1e-3 * mdata.polar[0].Re;
		V2[ptr] = 1e-3 * mdata.polar[1].Re - V1[ptr];
		V7[ptr] = 1e-3 * mdata.polar[2].Re - V1[ptr];
	} else {
		V1[ptr] = 1e-3 * mdata.polar[0].Re;
		V2[ptr] = 1e-3 * mdata.polar[1].Re;
		V7[ptr] = 1e-3 * mdata.polar[2].Re;
	}

	A1[ptr] = angle1;
	A2[ptr] = angle2;
	A7[ptr] = angle7;

	ptr++;
	if (ptr >= MS)
		ptr = 0;

	round++;
	if (round >= averages) {
		if (sigma) {
			lcd_gotoxy(0, 2);
			lcd_putstr("Ch1(PA1)", 0);
			printSmallFloat(getSigma(V1, MS), "V");
			lcd_gotoxy(0, 3);
			lcd_putstr("        ", 0);
			printSmallFloat(getSigma(A1, MS), "`");
			lcd_gotoxy(0, 4);
			lcd_putstr("Ch2(PA2)", 0);
			printSmallFloat(getSigma(V2, MS), "V");
			lcd_gotoxy(0, 5);
			lcd_putstr("        ", 0);
			printSmallFloat(getSigma(A2, MS), "`");
			lcd_gotoxy(0, 6);
			lcd_putstr("Ch7(PA7)", 0);
			printSmallFloat(getSigma(V7, MS), "V");
			lcd_gotoxy(0, 7);
			lcd_putstr("        ", 0);
			printSmallFloat(getSigma(A7, MS), "`");

		} else {
			lcd_gotoxy(0, 2);
			lcd_putstr("Ch1(PA1)", 0);
			printSmallFloat(getMean(V1, MS), "V");
			lcd_gotoxy(0, 3);
			lcd_putstr("        ", 0);
			printSmallFloat(getMean(A1, MS), "`");
			lcd_gotoxy(0, 4);
			lcd_putstr("Ch2(PA2)", 0);
			printSmallFloat(getMean(V2, MS), "V");
			lcd_gotoxy(0, 5);
			lcd_putstr("        ", 0);
			printSmallFloat(getMean(A2, MS), "`");
			lcd_gotoxy(0, 6);
			lcd_putstr("Ch7(PA7)", 0);
			printSmallFloat(getMean(V7, MS), "V");
			lcd_gotoxy(0, 7);
			lcd_putstr("        ", 0);
			printSmallFloat(getMean(A7, MS), "`");
		}

		round = 0;
	}

	if (btns & (BTN_PUSH << REL_BTN)) {
		if (sigma)
			sigma = 0;
		else
			sigma = 1;
	}
	if (btns & (BTN_LPUSH << REL_BTN)) {
		if (rel)
			rel = 0;
		else
			rel = 1;
	}
	if (btns & (BTN_PUSH << SP_BTN)) {
		findex++;
		if (findex >= FCNT)
			findex = 0;
		freq = flist[findex];
		gain = fgain[findex];
		fillSine(freq);
	}

	if (btns & (BTN_PUSH << PWR_BTN)) {
		gain = fgain[findex];
		return menu[menuCurrent].parent;
	} // menu select

	return menuCurrent;
}

/**
 * @brief:  leafBalance()
 *          This function returns menu state.
 *          It writes "Balance" to LCD and calls balance().
 *
 * @param:  btns
 * @return: menu[menuCurrent].parent
 */

int leafBalance(int btns) {

	lcd_gotoxy(0, 0);
	lcd_putstr("Balance", 1);

	balance();

	return menu[menuCurrent].parent;
}

/**
 * @brief:  leafCal()
 *          This function returns menu state.
 *          It writes "Calibration" to LCD and calls calibrate().
 *
 * @param:  btns
 * @return: menu[menuCurrent].parent
 */

int leafCal(int btns) {

	lcd_gotoxy(0, 0);
	lcd_putstr("Calibration", 1);

	calibrate();

	return menu[menuCurrent].parent;
}

/**
 * @brief:  leafCalReset()
 *          This function calculates global variable cstatus.
 *          eepromVoidParam function is called twice.
 *          corr variable is reseted.
 *          In the end this function returns menu state.
 *
 * @param:  btns
 * @return: menu[menuCurrent].parent
 */

int leafCalReset(int btns) {

	cstatus &= ~((1 << (findex)) | (1 << (findex + 8)));

	eepromVoidParam(EEP_BALANCE_BASE + findex);
	eepromVoidParam(EEP_CAL_BASE + findex);

	resetBalanceData();

	return menu[menuCurrent].parent;
}

/**
 * @brief:  editScr()
 *          This function
 *
 * @param:  sel, * str, btns
 * @return: 0, * str
 */
int editScr(int sel, char* str, int btns) {

	uint32_t m = getBf(BIT_SCR_MODE);

	strcpy(str, configBits[BIT_SCR_MODE].list[m]);

	if (btns & (BTN_PUSH << SP_BTN)) {
		m += 1;
		if (m > configBits[BIT_SCR_MODE].max)
			m = 0;
		lcd_init(m);
	}
	if (btns & (BTN_PUSH << REL_BTN)) {
		m -= 1;
		lcd_init(m);
	}
	setBf(BIT_SCR_MODE, m);

	return 0;
}

/**
 * @brief:  editBalance()
 *          This function copies string "OK" or "--" to str pointer
 *
 * @param:  sel, * str, btns
 * @return: -1, * str
 */

int editBalance(int sel, char* str, int btns) {

	if (cstatus & (1 << (findex + 8)))
		strcpy(str, "OK");
	else
		strcpy(str, "--");

	return (-1); // jump to child
}

/**
 * @brief:  editCal()
 *          This function copies string "OK" or "--" to str pointer
 *
 * @param:  sel, * str, btns
 * @return: -1, * str
 */

int editCal(int sel, char* str, int btns) {

	if (cstatus & (1 << (findex)))
		strcpy(str, "OK");
	else
		strcpy(str, "--");

	return (-1); // jump to child
}

/**
 * @brief:  editContrast()
 *          This function edits LCD contrast
 *
 * @param:  sel, * str, btns
 * @return: 0, * str
 */

int editContrast(int sel, char* str, int btns) {
	uint32_t bf = getBf(BIT_SCR_CONTRAST);

	sputdec(str, bf);

	if (btns & (BTN_LPUSH << PWR_BTN))
		bf = DEF_CONTRAST;

	if (btns & (BTN_PUSH << SP_BTN)) {
		bf += 1;
	}
	if (btns & (BTN_PUSH << REL_BTN)) {
		bf -= 1;
	}

	lcd_setcontrast(setBf(BIT_SCR_CONTRAST, bf));

	return 0;
}

/**
 * @brief:  editR()
 *          This function modifies (subtracts and adds 0.1 or 1) variable R
 *
 * @param:  sel, * str, btns
 * @return: 0, * str
 */

int editR(int sel, char* str, int btns) {

	sputdec(str, R.Re * 10);
	int k = strlen(str);
	str[k] = str[k - 1];
	str[k - 1] = '.';
	str[k + 1] = '\\'; // ohm
	str[k + 2] = 0;

	if (btns & (BTN_LPUSH << PWR_BTN))
		R.Re = SHUNT;

	if (btns & (BTN_PUSH << SP_BTN)) {
		R.Re += 0.1;
	}
	if (btns & (BTN_PUSH << REL_BTN)) {
		R.Re -= 0.1;
	}
	if (btns & ((BTN_LPUSH | BTN_RPT) << SP_BTN)) {
		R.Re += 1;
	}
	if (btns & ((BTN_LPUSH | BTN_RPT) << REL_BTN)) {
		R.Re -= 1;
	}

	if (R.Re > 2000 || R.Re < 10)
		R.Re = SHUNT;

	return 0;
}

/**
 * @brief:  editAverages()
 *          This function modifies (subtracts and adds 1) variable bf
 *
 * @param:  sel, * str, btns
 * @return: 0, * str
 */

int editAverages(int sel, char* str, int btns) {
	uint32_t bf = getBf(BIT_AVERAGES);

	sputdec(str, bf);

	if (btns & (BTN_LPUSH << PWR_BTN))
		bf = DEF_AVERAGES;

	if (btns & (BTN_PUSH << SP_BTN)) {
		bf += 1;
	}
	if (btns & (BTN_PUSH << REL_BTN)) {
		bf -= 1;
	}

	if (bf < 1)
		bf = 1;
	if (bf > 20)
		bf = 20;

	setBf(BIT_AVERAGES, bf);

	return 0;
}

/**
 * @brief:  editBF()
 *          This function modifies (subtracts and adds 1) variable bf
 *
 * @param:  sel, * str, btns
 * @return: 0, * str
 */
int editBF(int sel, char* str, int btns) {

	int bfi = menu[sel].first;

	str[0] = 0;

	if (bfi >= CONFIG_BITS_COUNT)
		return 0;

	uint32_t bf = getBf(bfi);

	strcpy(str, configBits[bfi].list[bf]);

	if (btns & (BTN_PUSH << SP_BTN)) {
		bf += 1;
		if (bf > configBits[bfi].max)
			bf = 0;
	}
	if (btns & (BTN_PUSH << REL_BTN)) {
		bf -= 1;
	}

	setBf(bfi, bf);

	return 0;
}

/////////////////////////////////////////////////////////////////////////////////
//                              METROLOGY
/////////////////////////////////////////////////////////////////////////////////


/**
 * @brief:  calibrate()
 *          This function does calibration.
 *          During this process cal variable is initializing
 *          cal variable will be save to eeprom in the function end
 *
 * @param:  void
 * @return: void
 */

void calibrate(void) {
	waitOpen();

	print_openshort(0);

	measure(&(cal.Zo), CAL_ROUNDS);

	waitClose();

	print_openshort(1);

	measure(&(cal.Zs), CAL_ROUNDS);

	// save to eeprom
	eepromSaveParam( EEP_CAL_BASE + findex, &cal, sizeof(cal));

	cstatus |= (1 << (findex));
}



/////////////////////////////////////////////////////////////////////////////////
//              PUT TO STRING DIFERENT TYPE ARRAY AND TRANSMIT TO UART
/////////////////////////////////////////////////////////////////////////////////

/**
 * @brief:  puthex()
 *          This function fills buf array of hex variables and transmits to UART buf array.
 *          The function isn't being called during the program performance.
 *
 * @param:  inp, size
 * @return: void
 */

void puthex(int inp, short size) {
	char buf[9];
	int j;

	for (j = size - 1; j >= 0; j--) {
		char dig = (inp & 0xf);
		if (dig < 10) {
			buf[j] = 0x30 + dig;
		} else {
			buf[j] = 0x41 + dig - 10;
		}
		inp >>= 4;
	}

	buf[size] = 0;
	uart_tx(buf, 1);
}

/**
 * @brief:  sputdec()
 *          This function fills buf array of integer type
 *          if startflag variable == 0
 *
 * @param:  * buf, inp
 * @return: * buf
 */

void sputdec(char * buf, int inp) {
	int ptr = 0;
	int s, startflag = 1;
	int digit;

	for (ptr = 0; ptr < 12; ptr++)
		buf[ptr] = 0;
	ptr = 0;

	if (inp < 0) {
		inp = -inp;
		buf[ptr++] = '-';
	}

	for (s = 0; s < 10; s++) {
		digit = 0;
		while (inp >= dig[s]) {
			inp -= dig[s];
			digit++;
		}

		if (digit != 0)
			startflag = 0;
		if (s == 9)
			startflag = 0;
		if (startflag == 0)
			buf[ptr++] = 0x30 + digit;
	}
	return;
}

/**
 * @brief:  putdec()
 *          This function transmits to UART buf massive
 *
 * @param:  inp
 * @return: void
 */

void putdec(int inp) {
	char buf[12];
	sputdec(buf, inp);
	uart_tx(buf, 1);
}

/**
 * @brief:  sputFloat()
 *          This function fills buf array of float type
 *          The function defines magnitude of measurements
 *          for instance, p, n, u, k, M, G, etc.
 *
 * @param:  num, *out, *suffix
 * @return: *out, *suffix
 */

void sputFloat(float num, char * out, char * suffix) {
	char nnn[20];
	int i, dot;

	out[0] = ' ';
	if (num < 0) {
		num = -num;
		out[0] = '-';
	}
	if (num > 1e10)
		num = 1e10;
	if (num < 1e-15)
		num = 1e-15;

	int exp = 19;

	if (num < 10000.0) {
		while (num < 10000.0) {
			num *= 10.0;
			exp--;
		}
	} else {
		while (num > 99999.4) {
			num *= 0.1;
			exp++;
		}
	}

	dot = (exp + 30) % 3;
	exp = (exp) / 3;

	if (exp < 0 || exp > 8) {
		for (i = 0; i < 6; i++) {
			out[i + 1] = '-';
		}
	} else {

		sputdec(nnn, num + 0.5);

		char * optr = &out[1];
		for (i = 0; i < 6; i++) {
			*optr++ = nnn[i];
			if (i == dot) {
				*optr++ = '.';
			}
		}
	}

	out[6] = 0;
	suffix[0] = dp[exp];
}

/**
 * @brief:  printFloat()
 *          This function transmits to UART "out" and "sfx" arrays
 *          for LCD measurement watching
 *
 * @param:  x, y, num, *suffix
 * @return: *suffix
 */

void printFloat(int x, int y, float num, char * suffix) {
	char out[20];
	char sfx[20];

	sfx[0] = ' ';
	strcpy(sfx + 1, suffix);
	sputFloat(num, out, sfx);

	lcd_gotoxy(x, y + 1);
	if (num >= 0)
		lcd_putstr(" ", 0);
	else
		lcd_putstr("-", 0);

	lcd_putnum(x + 6, y, out);
	lcd_gotoxy(15 * 4 + x, y + 1);
	lcd_putstr(sfx, 0);

	if (getBf(BIT_UART_MODE)) {
		if (sfx[1] == '\\')
			strcpy(&sfx[1], "Ohm");
		uart_tx(out, 1);
		uart_tx(sfx, 1);
		uart_tx(" ", 1);
	}
}

/**
 * @brief:  printSmallFloat()
 *          This function prints to LCD.
 *
 * @param:  num, *suffix
 * @return: void
 */

void printSmallFloat(float num, char * suffix) {
	char out[20];
	char sfx[20];

	sfx[0] = ' ';
	strcpy(sfx + 1, suffix);
	sputFloat(num, out, sfx);

	lcd_putstr(out, 0);
	lcd_putstr(sfx, 0);
}

/////////////////////////////////////////////////////////////////////////////////
//              MANAGMENT BY PERIPHERAL (LCD, BUTTONS, BATTARY ETC.)
/////////////////////////////////////////////////////////////////////////////////

/**
 * @brief:  powerOff()
 *          This function turns off the RLC device.
 *          Same parameters will be save to EEPROM.
 *          MCU reset request will be call in the end of function.
 *
 * @param:  void
 * @return: void
 */

void powerOff(void) {
	int cnt = 0;

	eepromSaveParam( EEP_SHUNT, &R, sizeof(R));
	eepromSaveParam( EEP_PARAMS1, &bitParams, sizeof(bitParams));

	AFIO->EXTICR[3] = AFIO_EXTICR4_EXTI13_PC;

	EXTI->FTSR = EXTI_FTSR_TR13;
	EXTI->IMR = 0;
	EXTI->EMR = EXTI_EMR_MR13;
	EXTI->PR = 0x0003FFFF;

	DAC->CR = 0;
	ADC1->CR2 = 0;
	GPIOA->CRH &= ~(
	GPIO_CRH_CNF9 | GPIO_CRH_MODE9 // usart tx
			| GPIO_CRH_CNF10 | GPIO_CRH_MODE10 // usart rx
	);

	lcd_write(COMMAND, 0xA5); // DAL
	lcd_write(COMMAND, 0xAE); // turn off display

	GPIOA->BSRR = GPIO_Pin_3; // ANALOG OFF
	GPIOA->BRR = GPIO_Pin_15; // BACKLIGHT OFF

	SCB->SCR |= SCB_SCR_SLEEPDEEP;
	PWR->CR &= ~(PWR_CR_PDDS | PWR_CR_LPDS);
	PWR->CR |= PWR_CR_LPDS;

	while (!(GPIOC->IDR & GPIO_Pin_13))
		;

	apo = APO_MAX;
	while (cnt < 1000000) {
		if (cnt > 400000)
			GPIOA->BSRR = GPIO_Pin_15; // BACKLIGHT ON
		if (!(GPIOC->IDR & GPIO_Pin_13))
			cnt++;
		else {
			if (cnt > 400000) {
				apo = APO_4MIN;
				break;
			} //512 = 4min
			GPIOA->BRR = GPIO_Pin_15;
			if (cnt > 0)
				cnt = 0;
			else
				cnt--;
			if (cnt < -100) {
				EXTI->PR = 0x0003FFFF;
				__NOP();
				__WFE();
			}
		}
	}
	NVIC_SystemReset();

	//GPIOA->BSRR = GPIO_Pin_15; // BACKLIGHT ON
}

/**
 * @brief:  setBacklight()
 *          This function initializes GPIOA->BSRR structure
 *          and calls setBf()
 *
 * @param:  mode
 * @return: void
 */

void setBacklight(int mode) {
	if (mode) {
		GPIOA->BSRR = GPIO_Pin_15; // BACKLIGHT ON
		setBf(BIT_BL_MODE, 1);
	} else {
		GPIOA->BSRR = GPIO_Pin_15 << 16; // BACKLIGHT OFF
		setBf(BIT_BL_MODE, 0);
	}
}

//----------------------------------------------------------------------------
void printSecondary(int x, int y, int idx) {
	lcd_gotoxy(x, y);

	switch (idx) //{"OFF","Z","Q","D","VDD","VBAT"};
	{
	case 1:
		lcd_putstr("Z  ", 0);
		printSmallFloat(mdata.Zpolar.Re, "\\");
		break;
	case 2:
		lcd_putstr("Q  ", 0);
		printSmallFloat(mdata.Q, " ");
		break;
	case 3:
		lcd_putstr("D  ", 0);
		printSmallFloat(mdata.D, " ");
		break;
	case 4:
		lcd_putstr("VD ", 0);
		printSmallFloat(vcc * 0.001, "V");
		break;
	case 5:
		lcd_putstr("VB ", 0);
		printSmallFloat(vbat * 0.001, "V");
		break;
	case 6:
		lcd_putstr("AN ", 0);
		printSmallFloat(mdata.Zpolar.Im, "`");
		break;
	default:
		lcd_putstr("          ", 0);
	}
	return;
}

/**
 * @brief:  waitz()
 *          This function calls measure function.
 *          Device will be turn off if user pushes power button.
 *
 * @param:  lim
 * @return: void
 */

void waitz(float lim) {
	float min = -lim, max = lim;
	float z;
	cplx Z;

	sign = -1;

	if (lim < 0) // minimum
			{
		z = min * 2;
		while (z > min) {
			measure(&Z, 1);
			z = absolute(Z.Re);
			if ((buttons() >> PWR_BTN) & BTN_PUSH)
				powerOff();
		}

	} else {
		z = 0;
		while (z < max * max) {
			measure(&Z, 1);
			z = (Z.Re * Z.Re + Z.Im * Z.Im);
			if ((buttons() >> PWR_BTN) & BTN_PUSH)
				powerOff();
		}
	}

	measure(&Z, CAL_SETTLE);
}

/**
 * @brief:  waitOpen()
 *          This function writes string "Open leads!!" to LCD (LCD coordinate: x = 0, y = 2).
 *          The waitz function calling is doing in the end
 *
 * @param:  void
 * @return: void
 */

void waitOpen(void) {
	lcd_gotoxy(0, 2);
	lcd_putstr("Open leads!!", 1);
	waitz(1000);
}

/**
 * @brief:  waitClose()
 *          This function writes string "Close leads!!" to LCD (LCD coordinate: x = 0, y = 2).
 *          The waitz function calling is doing in the end
 *
 * @param:  void
 * @return: void
 */

void waitClose(void) {
	lcd_gotoxy(0, 2);
	lcd_putstr("Close leads!!", 1);
	waitz(-10);
}

/**
 * @brief:  print_openshort()
 *          This function implements case loop
 *          t parameter defines that will be write to LCD
 *
 * @param:  t
 * @return: void
 */

void print_openshort(int t) // open - 0
{
	lcd_gotoxy(0, 2);

	switch (t) {
	case 0:
		lcd_putstr("Open cal    ", 1);
		break;
	case 1:
		lcd_putstr("Short cal   ", 1);
		break;
	case 2:
		lcd_putstr("Balance 1   ", 1);
		break;
	case 3:
		lcd_putstr("Balance 2   ", 1);
		break;
	default:
		lcd_putstr("Error!!!!!  ", 1);
		break;
	}
}

//----------------------------------------------------------------------------

void balance(void) {
	cplx ch0, ch2;
	uint32_t ogain = gain;

	resetBalanceData();

	waitOpen();

	print_openshort(2); // ------------------

	swap_dacs = 0;
	sign = -1;

	measure(&Z, 1);
	measure(&Z, CAL_ROUNDS);

	ch0.Re = mAcc[1].Re;
	ch0.Im = mAcc[1].Im;

	ch2.Re = mAcc[2].Re;
	ch2.Im = mAcc[2].Im;

	print_openshort(3); // ------------------

	sign = 1;

	measure(&Z, 1);
	measure(&Z, CAL_ROUNDS);

	if (ch0.Re * mAcc[1].Re > 0) {
		ch0.Re -= mAcc[1].Re;
		ch0.Im -= mAcc[1].Im;
		ch2.Re -= mAcc[2].Re;
		ch2.Im -= mAcc[2].Im;

	} else {

		ch0.Re += mAcc[1].Re;
		ch0.Im += mAcc[1].Im;
		ch2.Re += mAcc[2].Re;
		ch2.Im += mAcc[2].Im;
	}

	cplxDiv(&ch0, &ch2);

	corr.shift0.Re = ch0.Re;
	corr.shift0.Im = ch0.Im;

	//
	sign = 1;
	gain = ogain;
	measure(&Z, 8);

	print_openshort(0);

	measure(&Z, 1);
	measure(&Z, CAL_ROUNDS);

	ch0.Re = mAcc[1].Re;
	ch0.Im = mAcc[1].Im;

	cplxDiv(&ch0, &mAcc[0]); // corr ch0 to match ch1

	waitClose();

	sign = 1;
	measure(&Z, 8);
	print_openshort(1);

	measure(&Z, 2);
	measure(&Z, CAL_ROUNDS);

	ch2.Re = mAcc[0].Re;
	ch2.Im = mAcc[0].Im;

	cplxDiv(&ch2, &mAcc[2]); // corr ch2 to match ch0
	cplxMul(&ch2, &ch0);    // full corr ch2

	corr.Corr0.Re = ch0.Re;
	corr.Corr0.Im = ch0.Im;
	corr.Corr2.Re = ch2.Re;
	corr.Corr2.Im = ch2.Im;

	// save Corr to eeprom

	eepromSaveParam( EEP_BALANCE_BASE + findex, &corr, sizeof(corr));

	cstatus |= (1 << (findex + 8));
	sign = -1;
	swap_dacs = 0;
}

/**
 * @brief:  processButton()
 *          This function indicates push process.
 *          The function will be return 1 if user pushes button.
 *
 * @param:  state
 * @return: res
 */

unsigned int processButton(unsigned int state) {
	int res = 0;
	if (((state & PUSH_MSK) == PUSH_MAP)
			&& ((state & LPUSH_MSK) != (LPUSH_MSK - 1))) {
		res |= BTN_PUSH;
		//print_to_consol_int(&res);
		//asm ( "mov %1, %0" : "=r" (sys_time) : "r" (sys_time_counter)); // sys_time = sys_time_counter;
		//sys_time = sys_time_counter - sys_time;
		//sys_time_counter++; //my
	    //my_timer1();
	} // push debounce
	if ((state & LPUSH_MSK) == LPUSH_MAP) {
		res |= BTN_LPUSH;
		//print_to_consol_int(&res);
		//asm ( "mov %1, %0" : "=r" (sys_time) : "r" (sys_time_counter)); // sys_time = sys_time_counter;
		//sys_time = sys_time_counter - sys_time;
		//sys_time_counter++; //my
		//my_timer1();
	}
	if ((state & RPT_MSK) == RPT_MAP) {
		res |= BTN_RPT;
		//print_to_consol_int(&res);
		//asm ( "mov %1, %0" : "=r" (sys_time) : "r" (sys_time_counter)); // sys_time = sys_time_counter;
		//sys_time = sys_time_counter - sys_time;
		//sys_time_counter++; //my
		//my_timer1();
	}
	//my_button(res); // my test
	return res;
}

/**
 * @brief:  buttons()
 *          This function defines which button user has pushed.
 *
 * @param:  void
 * @return: out
 */

int buttons(void) {
	int out = 0;
	static unsigned int state[BTN_CNT];

	for (int i = 0; i < BTN_CNT; i++) {
		state[i] <<= 1;
		if (!(*btn[i].port & btn[i].msk)) {
			state[i] |= 1;
		}
		out |= processButton(state[i]) << i;
		//my_timer2(out); // my test
	}
	return (out);
}

/**
 * @brief:  printBat()
 *          This function writes to LCD battery charge
 *
 * @param:  x, y, percent
 * @return: void
 */

void printBat(int x, int y, int percent) {

	lcd_gotoxy(x, y);

	if (percent < 0)
		percent = 0;
	int active = 1 + (BAT_N_SEG * percent) / 100;

	for (int i = 0; i < BAT_N_SEG; i++) {
		if (i < active) {
			lcd_write(DATA, 0x7F);
			lcd_write(DATA, 0x7F);
			lcd_write(DATA, 0x00);
		} else {
			lcd_write(DATA, 0x41);
			lcd_write(DATA, 0x41);
			lcd_write(DATA, 0x41);
		}
	}
	lcd_write(DATA, 0x7F); // tail
	lcd_write(DATA, 0x1C);

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

	GPIOA->CRL = GPIO_CRL_MODE3_1;// analog switch
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
            | GPIO_CRL_CNF3_1
			| GPIO_CRL_CNF4_1
			| GPIO_CRL_CNF5_1
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

/**
 * @brief:  getBf()
 *          This function sets up configuration mode
 *
 * @param:  idx
 * @return: val if making is successful, 0 otherwise
 */

uint32_t getBf(int idx) {
	if (idx >= CONFIG_BITS_COUNT)
		return 0;

	uint32_t mask = (0xFFFFFFFF >> (32 - configBits[idx].lenBits));
	uint32_t val = mask & ((*configBits[idx].val) >> configBits[idx].startBit);
	if (val > configBits[idx].max)
		val = configBits[idx].max;

	return (val);
}

/**
 * @brief:  setBf()
 *          This function sets up configuration mode
 *
 * @param:  idx, val
 * @return: val if making is successful, 0 otherwise
 */

uint32_t setBf(int idx, uint32_t val) {
	if (idx >= CONFIG_BITS_COUNT)
		return (0);
	uint32_t mask = (0xFFFFFFFF >> (32 - configBits[idx].lenBits))
			<< configBits[idx].startBit;
	if (val > configBits[idx].max)
		val = configBits[idx].max;
	*configBits[idx].val = ((*configBits[idx].val) & (~mask))
			| (val << configBits[idx].startBit);  // reset field

	return (val);
}

///==============================================================================
///                                   E O F
///=============================================================================

/*
	t_debug debug[]=       {{bottom_button,      {1, depressed,    depressed,       {20, 30, 40}},    1},
	                        {bottom_button,      {1, pressed,      depressed,       {20, 30, 40}},    4},
	                        {bottom_button,      {1, long_pressed, pressed,         {20, 30, 40}},    10},
	                        {bottom_button,      {1, hold,         long_pressed,    {20, 30, 40}},    1},

	                        {central_button,     {1, depressed,    depressed,       {20, 30, 40}},    4},
	                       	{central_button,     {1, pressed,      depressed,       {20, 30, 40}},    10},
	                        {central_button,     {1, long_pressed, pressed,         {20, 30, 40}},    1},
	                        {central_button,     {1, hold,         long_pressed,    {20, 30, 40}},    4},

						    {top_button,         {1, depressed,    depressed,       {20, 30, 40}},    4},
							{top_button,         {1, pressed,      depressed,       {20, 30, 40}},    10},
							{top_button,         {1, long_pressed, pressed,         {20, 30, 40}},    1},
							{top_button,         {1, hold,         long_pressed,    {20, 30, 40}},    4}};

int state_button_ivan(button_position but_pos, button_tt* button) {
	timer_tt* my_timer = button->timer;
	register int flags_mask = 0;
	register int flags = button->flags;

	switch (button->current_state) {

	case depressed:
		button->timer->push_time = 0;
		switch (but_pos) {
		case bottom_button:
			button->current_state = pressed;
			break;
		case central_button:
			button->current_state = pressed;
			break;
		case top_button:
			button->current_state = pressed;
			break;
		}
		button->last_state = depressed;
		flags = flags & 0b110;

		break;

	case pressed:
		button->timer->push_time++;
		if (button->last_state == depressed) {
			set_timer(my_timer, 10);
		}
        switch(but_pos)  {
		case bottom_button:
			if (get_timer(my_timer) == 1) {
				button->current_state = long_pressed;
				flags_mask = 1;
			}
			break;
		case central_button:
			if (get_timer(my_timer) == 1) {
				button->current_state = long_pressed;
				flags_mask = 1;
				//button->timer->push_time++;
			}
			break;
		case top_button:
			if (get_timer(my_timer) == 1) {
				button->current_state = long_pressed;
				flags_mask = 1;
				//button->timer->push_time++;
			}
			break;
		default: button->current_state = depressed;
        }
		button->last_state = pressed;
		break;

	case long_pressed:
		button->timer->push_time++;
		if (button->last_state == pressed) {
			set_timer(my_timer, 10);
		}
		switch (but_pos) {
		case bottom_button:
			if (get_timer(my_timer) == 1) {
				button->current_state = hold;
				flags_mask = 2;
			}
			break;
		case central_button:
			if (get_timer(my_timer) == 1) {
				button->current_state = hold;
				flags_mask = 2;
				//button->timer->push_time++;
			}
			break;
		case top_button:
			if (get_timer(my_timer) == 1) {
				button->current_state = hold;
				flags_mask = 2;
				//button->timer->push_time++;
			}
			break;
		default:
			button->current_state = depressed;
		}

		button->last_state = long_pressed;
		break;

	case hold:
		button->timer->push_time++;
		if (button->last_state == long_pressed) {
			set_timer(my_timer, 1);
		}
		switch (but_pos) {
		case bottom_button:
			button->current_state = hold;
			break;
		case central_button:
			button->current_state = hold;
			//button->timer->push_time++;
			break;
		case top_button:
			button->current_state = hold;
			//button->timer->push_time++;
			break;
		default:
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
}*/

/*void my_unity_test(button_position button_pos, button_tt* button) {
	static uint8_t count = 0;
	uint64_t time, push_time;
	timer_tt* my_timer;

	if (count == 0) {
		assert(state_button_ivan(button_pos, button) == 0);
		uart_tx("\r\nSUCCESSFUL", 1);
		count = 1;
	}

	if (!(*btn[button_pos].port & btn[button_pos].msk))
		state_button_ivan(button_pos, button);
	else
		button->current_state = depressed;

    switch(button->current_state) {
    case depressed:
		switch(button_pos) {

		case bottom_button:
			push_time = 0;
			time = sys_time;
		    uart_tx("\n\rBottom button     is depressed     ", 1);
		    print_to_consol_int(&push_time); uart_tx("                      ", 1);
		    print_to_consol_int(&time);
		    break;
		case central_button:
			push_time = 0;
			time = sys_time;
    		uart_tx("\n\rCentral button    is depressed     ", 1);
    		print_to_consol_int(&push_time); uart_tx("                      ", 1);
    		print_to_consol_int(&time);
    		break;
		case top_button:
			push_time = 0;
			time = sys_time;
		    uart_tx("\n\rTop button        is depressed     ", 1);
		    print_to_consol_int(&push_time); uart_tx("                      ", 1);
		    print_to_consol_int(&time);
		    break;
    	}
		break;

    case pressed :
    	push_time =  button->timer->push_time;
		switch(button_pos) {
		case bottom_button:
			time = sys_time;
		    uart_tx("\n\rBottom button     is pressed       ", 1);
		    print_to_consol_int(&push_time); uart_tx("                      ", 1);
		    print_to_consol_int(&time);
		    break;
		case central_button:
			time = sys_time;
    		uart_tx("\n\rCentral button    is pressed       ", 1);
    		print_to_consol_int(&push_time); uart_tx("                      ", 1);
    		print_to_consol_int(&time);
    		break;
		case top_button:
			time = sys_time;
		    uart_tx("\n\rTop button        is pressed       ", 1);
		    print_to_consol_int(&push_time); uart_tx("                      ", 1);
		    print_to_consol_int(&time);
		    break;
    	}
		break;

    case long_pressed :
    	push_time =  button->timer->push_time;
		switch(button_pos) {
		case bottom_button:
			time = sys_time;
		    uart_tx("\n\rBottom button     is long pressed  ", 1);
		    print_to_consol_int(&push_time); uart_tx("                      ", 1);
		    print_to_consol_int(&time);
		    break;
		case central_button:
			time = sys_time;
    		uart_tx("\n\rCentral button    is long pressed  ", 1);
    		print_to_consol_int(&push_time); uart_tx("                      ", 1);
    		print_to_consol_int(&time);
    		break;
		case top_button:
			time = sys_time;
		    uart_tx("\n\rTop button        is long pressed  ", 1);
		    print_to_consol_int(&push_time); uart_tx("                      ", 1);
		    print_to_consol_int(&time);
		    break;
    	}
		break;

	case hold:
		push_time = button->timer->push_time;
		switch (button_pos) {
		case bottom_button:
			time = sys_time;
			uart_tx("\n\rBottom button     is held          ", 1);
			print_to_consol_int(&push_time); uart_tx("                      ", 1);
			print_to_consol_int(&time);
			break;
		case central_button:
			time = sys_time;
			uart_tx("\n\rCentral button    is held          ", 1);
			print_to_consol_int(&push_time); uart_tx("                      ", 1);
			print_to_consol_int(&time);
			break;
		case top_button:
			time = sys_time;
			uart_tx("\n\rTop button        is held          ", 1);
			print_to_consol_int(&push_time); uart_tx("                      ", 1);
			print_to_consol_int(&time);
			break;
	    	}
			break;
	    }
    }*/

/*

#define TEST
void test(void);
void test(void) {
#ifdef TEST
	uart_tx("\r\nHello, World", 1);
#else
	uart_tx("\r\nHello, ARM", 1);
#endif
}

char state_button(char button_1, char last_state) {
	 // definition of button push
	if (!(*btn[button_1].port & btn[button_1].msk))
		toggle = on;

	else
		toggle = off;


	// definition of current event (press, hold etc.)
	if (toggle == on && last_state < 4)
		current_event = press;
	else if (toggle == on && (last_state >= 4 && last_state <= 7))
		current_event = long_press;
	else if (toggle == on && last_state > 7)
		current_event = hold;
	else
		current_event = depress;

	erase(); // clean console

	// definition of current button state (pressed, long pressed etc.)
	switch (current_event) {

	case depress:
		current_button_state = depressed;
		last_state = 0;

		uart_tx("\n\rButton is depressed", 1);
		break;

	case press:
		current_button_state = pressed;
		last_state++;

		uart_tx("\n\rButton is pressed", 1);
		break;

	case long_press:
		current_button_state = long_pressed;
		last_state++;

		uart_tx("\n\rButton is long pressed", 1);
		break;

	case hold:
		current_button_state = held;
		last_state++;

		uart_tx("\n\rButton is held", 1);
		break;
	}

	return last_state;
}

void state_mashina(button_number button, state_toggle toggle, char* last_state) {

	// definition of current event (press, hold etc.)
	if (toggle == on && (*last_state) < 4)
		current_event = press;
	else if (toggle == on && ((*last_state) >= 4 && (*last_state) <= 7))
		current_event = long_press;
	else if (toggle == on && (*last_state) > 7)
		current_event = hold;
	else
		current_event = depress;


	// definition of current button state (pressed, long pressed etc.)
	switch (current_event) {

	case depress:
		current_button_state = depressed;
		(*last_state) = 0;

		uart_tx("\n\rButton is depressed", 1);
		break;

	case press:
		current_button_state = pressed;
		(*last_state)++;

		uart_tx("\n\rButton is pressed", 1);
		break;

	case long_press:
		current_button_state = long_pressed;
		(*last_state)++;

		uart_tx("\n\rButton is long pressed", 1);
		break;

	case hold:
		current_button_state = held;
		(*last_state)++;

		uart_tx("\n\rButton is held", 1);
		break;
	}
	erase(); // clean console
}
/**
 * @brief:  my_button()
 *          This function defines state of button.
 *
 * @param:  void
 * @return: void
 */
/*
void my_button(uint64_t time_accumulator[], int index, uint64_t* flag1) {
	uint64_t flag2 = 0;
	//print_to_consol_int(flag1);
	//print_to_consol_int(&flag2);
	*flag1 = time_accumulator[index];
	flag2 = time_accumulator[--index];
	*flag1 -= flag2;
	//uart_tx("\r\n---", 1);
	//print_to_consol_int(flag1);
	//print_to_consol_int(&flag2);
	//uart_tx("\r\n===", 1);
	if((*flag1) > 20 ) {
		current_event = press;
	}
	else if ((*flag1) >= 2 && (*flag1) <= 10) {
		current_event = hold;
	}
	else {
		current_event = depress;
	}
    *flag1 = 0;
	erase(); // Form feed/clear screen

	switch(current_event) {
	    // Button is depressed
	    case depress:
			current_button_state = depressed;

			uart_tx("\r\nButton is depressed", 1);
			break;
		// Button is pressed
		case press:
			current_button_state = pressed;

			uart_tx("\r\nButton is pressed", 1);
			break;
		// Button is long pressed
		case long_press:
			current_button_state = long_pressed;

			uart_tx("\r\nButton is long pressed", 1);
			break;
        // Button is held
		case hold:
			current_button_state = held;

			uart_tx("\r\nButton is held", 1);
		    break;
	}
}


/**
 * @brief:  my_timer()
 *          This function outputs current system time.
 *
 * @param:  void
 * @return: void
 */
/*
void my_timer1(int* number, uint64_t* flag1) {
	//sys_time = sys_time_counter - sys_time;
	uint64_t *depr = 0;
	for (int item = 0; item < 3; item++) {
		if (!(*btn[item].port & btn[item].msk)) {   // indication of button push
			int index = ++(*number);
			//asm ( "mov %1, %0" : "=r" (sys_time) : "r" (sys_time_counter));
			time_accumulator[index] = sys_time;
			my_button(time_accumulator, index, &flag1);
			}
		else {
			//asm ( "mov %1, %0" : "=r" (sys_time) : "r" (sys_time_counter));
			time_accumulator[98] = sys_time;
			time_accumulator[99] = sys_time;
			my_button(time_accumulator, 99, &depr); // my test
		}
	}
}

void my_timer2(int out) {

/*
	erase();

	if(out == 0 ) current_event = depress;
	else if (out == 1 || out == 2) current_event = press;
	else if (out == 256 || out == 512 || out == 1024) current_event = long_press;
	else current_event = hold;

	switch(current_event) {
		    // Button is depressed
		    case depress:
				current_button_state = depressed;

				uart_tx("\rButton is depressed", 1);
				break;
			// Button is pressed
			case press:
				current_button_state = pressed;

				uart_tx("\rButton is pressed", 1);
				break;
			// Button is long pressed
			case long_press:
				current_button_state = long_pressed;

				uart_tx("\rButton is long pressed", 1);
				break;
	        // Button is held
			case hold:
				current_button_state = held;

				uart_tx("\rButton is held", 1);
			    break;
		}*/
//}


void erase(void) {
	 uart_tx("\f", 1);
	 //for(int item = 0; item < 20; item++) uart_tx("\r\b", 1);
}

/*int main() {
 setup();
 initUART();
 uart_tx("\r\n", 1);
 void* xgoto[] = { &&a1, &&a2, &&a3 };
 int ip = 0;
 void* label;
 AGAIN:
 //for(;;) {
 uart_tx("ip = %d\n", ip);
 goto *xgoto[ip];
 // };
 a1:
 ip = 1;
 uart_tx("\ra1\n", 1);
 goto AGAIN;
 a2:
 ip = 2;
 uart_tx("\ra2\n", 1);
 goto AGAIN;
 a3:
 ip = 3;
 uart_tx("\ra3\n", 1);

 goto AGAIN;
 }*/
/**
 * @brief:  print_to_consol_cplx_x()
 *          This function I have written for myself.
 *          It prints to console variable which has type cplx.
 *
 * @param:  * ptr
 * @return: void
 */

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

/**
 * @brief:  my_test()
 *          This function I have written for myself.
 *          It prints to console variable GPIO_Pin_13.
 *
 * @param:  void
 * @return: void
 */

void my_test() {
	var1 = GPIO_Pin_13;
	print_to_consol_int(&var1);
	var2 = GPIO_Pin_13;
	uart_tx("\r\n", 1);
	uart_tx("var2 = ", 1);
	print_to_consol_int(&var2);
	uart_tx("\r\n", 1);
}
int add(int var1, int var2) {
	return var1 + var2;
}
//*****my test*****
//*****************

//*****my test*****
//*****************
//my_test(); // my test
//typedef int (*ptr) (int, int); // my test
//ptr var3 = add(1, 2); // my test
//print_to_consol_int(&var3); // my test

//int (*functionPtr)(int, int); // my test
//functionPtr = &add; // my test
//int sum = (*functionPtr)(2, 3); // my test
//print_to_consol_int(&sum); // my test
//print_to_consol_int(functionPtr); // my test
//print_to_consol_int(&add); // my test
//*****************
//*****my test*****

//*****my test*****
//char array[30];
//float pii = 3.123;
//int ret = sprintf(array, "Value of pi = %f \0", pii);
//uart_tx(array,1);
//print_to_consol_int(&ret);
//*****my test*****
