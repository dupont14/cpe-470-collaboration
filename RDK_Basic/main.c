
/************************************************************************/
/*																		*/
/*	main.c	--	Main program module for project							*/
/*																		*/
/************************************************************************/
/*	Author: 	Dion Moses												*/
/*	Copyright 2009, Digilent Inc.										*/
/************************************************************************/
/*  Module Description: 												*/
/*																		*/
/*	This program is a reference design for the Digilent	Basic			*/
/*	Robotic Development Kit (RDK-Basic) with the Cerebot 32MX4 			*/
/*	Microcontroller board.  It uses two timers to drive two motors 		*/
/*	with output compare modules.										*/
/*																		*/
/*																		*/
/************************************************************************/
/*  Revision History:													*/
/*																		*/
/*	 12/09/09(DionM): created											*/
/*   12/29/09(LeviB): altered to add movement functions and PmodBtn and */
/*					  PmodSwt functionality								*/
/*	 12/08/10(AaronO): renamed to RDK_Basic								*/	
/************************************************************************/

/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */

#include <plib.h>
#include <stdio.h>
#include <math.h>
#include "stdtypes.h"
#include "config.h"
#include "MtrCtrl.h"
#include "spi.h"
#include "util.h"
#include "sys/attribs.h"

/* ------------------------------------------------------------ */
/*				Local Type Definitions							*/
/* ------------------------------------------------------------ */

#define		TCKPS22 			6
#define 	TCKPS21				5
#define 	TCKPS20				4

#define		TCKPS32 			6
#define 	TCKPS31				5
#define 	TCKPS30				4
/* ------------------------------------------------------------ */
/*				Global Variables								*/
/* ------------------------------------------------------------ */
#ifndef OVERRIDE_CONFIG_BITS

#pragma config ICESEL   = ICS_PGx2		// ICE/ICD Comm Channel Select
#pragma config BWP      = OFF			// Boot Flash Write Protect
#pragma config CP       = OFF			// Code Protect
#pragma config FNOSC    = PRIPLL		// Oscillator Selection
#pragma config FSOSCEN  = OFF			// Secondary Oscillator Enable
#pragma config IESO     = OFF			// Internal/External Switch-over
#pragma config POSCMOD  = HS			// Primary Oscillator
#pragma config OSCIOFNC = OFF			// CLKO Enable
#pragma config FPBDIV   = DIV_8			// Peripheral Clock divisor
#pragma config FCKSM    = CSDCMD		// Clock Switching & Fail Safe Clock Monitor
#pragma config WDTPS    = PS1			// Watchdog Timer Postscale
#pragma config FWDTEN   = OFF			// Watchdog Timer 
#pragma config FPLLIDIV = DIV_2			// PLL Input Divider
#pragma config FPLLMUL  = MUL_16		// PLL Multiplier
#pragma config UPLLIDIV = DIV_2			// USB PLL Input Divider
#pragma config UPLLEN   = OFF			// USB PLL Enabled
#pragma config FPLLODIV = DIV_1			// PLL Output Divider
#pragma config PWP      = OFF			// Program Flash Write Protect
#pragma config DEBUG    = OFF			// Debugger Enable/Disable
    
#endif

/* ------------------------------------------------------------ */
/*				Local Variables									*/
/* ------------------------------------------------------------ */

#define	stPressed	1
#define	stReleased	0

#define	cstMaxCnt	10 // number of consecutive reads required for
					   // the state of a button to be updated

#define WAVE_WAITING_FAR       0
#define WAVE_WAITING_CLOSE     1
#define WAVE_WAITING_FAR_AGAIN 2

#define WAVE_CLOSE_DISTANCE    12.0
#define WAVE_FAR_DISTANCE      18.0

#define DOUBLE_WAVE_TIME       1000
#define MIN_WAVE_TIME          80
#define SPIN_TIME_360          1600
#define SPIN_COOLDOWN_TIME     1500

#define LCD_UPDATE_TIME        250

struct btn {
	BYTE	stBtn;	// status of the button (pressed or released)
	BYTE	stCur;  // current read state of the button
	BYTE	stPrev; // previous read state of the button
	BYTE	cst;	// number of consecutive reads of the same button 
					// state
};

//PmodCLS instructions
static	char szClearScreen[] = { 0x1B, '[', 'j', 0};

static	char szCursorOff[] = { 0x1B, '[', '0', 'c', 0 };
static	char szBacklightOn[]     = { 0x1B, '[', '3', 'e', 0 };

static	char szScrollLeft[] = {0x1B, '[', '1', '@', 0}; 
static	char szScrollRight[] = {0x1B, '[', '1', 'A', 0}; 
static	char szWrapMode[] = {0x1B, '[', '0', 'h', 0}; 

static	char szCursorPos[] = {0x1B, '[', '1', ';', '0', 'H', 0}; 
char strBuffer[16] = {"0000111100001111"};
/* ------------------------------------------------------------ */
/*				Global Variables				                */
/* ------------------------------------------------------------ */

volatile	struct btn	btnBtn1;
volatile	struct btn	btnBtn2;

volatile	struct btn	PmodBtn1;
volatile	struct btn	PmodBtn2;
volatile	struct btn	PmodBtn3;
volatile	struct btn	PmodBtn4;

volatile	struct btn	PmodSwt1;
volatile	struct btn	PmodSwt2;
volatile	struct btn	PmodSwt3;
volatile	struct btn	PmodSwt4;


// cpe 470 while loop declaration for Input capture buffe


int ic2_count = 0;
int ic3_count = 0;


int ic2_time = 0;
int ic2_previous_time = 0;
int ic2_delta_time = 0;


int ic3_time = 0;
int ic3_previous_time = 0;
int ic3_delta_time = 0;

int data_array[200];

float ic2_speed=0;
float ic3_speed=0;


//pid controller declerations
int t5_count = 0;

float kp = 8.75;
float ki = 2.18;
float kd = .4375;
float error_ic2 = 0;
float prev_error_ic2 = 0;
float ei_ic2 = 0;


float error_ic3 = 0;
float prev_error_ic3 = 0;
float ei_ic3 = 0;
float value;
int val_arr[10];
//int speed_arr[4000];
int motorspeed = 0;
int avg_count = 0;
int speed_arr_count = 0;
int secondary_count = 0;




// ADC conversion decleration

float ADCValue0 = 0;	// Reading AN0(zero), pin 1 of connector JJ -- servo sensor (center)
float ADCValue1 = 0;
float ADCValue2 = 0;	

float Distance0=0;
float Distance1=0;
float Distance2=0;

float dspeed0=0;
float dspeed1=0;

volatile DWORD ms_count = 0;

int wave_state = WAVE_WAITING_FAR;
int wave_count = 0;

DWORD first_wave_time = 0;
DWORD last_wave_time = 0;
DWORD wave_cooldown_time = 0;

BOOL spin_running = fFalse;
DWORD spin_stop_time = 0;


/* ------------------------------------------------------------ */
/*				Forward Declarations							*/
/* ------------------------------------------------------------ */

void	DeviceInit(void);
void	AppInit(void);
void	Wait_ms(WORD ms);
void	Pmod8LDSet( BYTE stLeds );

/* ------------------------------------------------------------ */
/*				Interrupt Service Routines						*/
/* ------------------------------------------------------------ */
/***	Timer5Handler
**
**	Parameters:
**		none
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Interrupt service routine for Timer 5 interrupt. Timer 5
**		is used to perform software debouncing of the on-board
**		buttons. It is also used as a time base for updating
**		the on-board LEDs and the Pmod8LD LEDs at a regular interval.
*/


void __ISR(_TIMER_5_VECTOR, ipl7AUTO) Timer5Handler(void)
{
	static	WORD tusLeds = 0;
    static int timer5_ms_counter = 0;
	
	mT5ClearIntFlag();
    
    timer5_ms_counter++;
    if(timer5_ms_counter >= 10)
    {
        timer5_ms_counter = 0;
        ms_count++;
    }
    
    prtLed4Set = (1 << bnLed4 );
	
	// Read the raw state of the button pins.
	btnBtn1.stCur = ( prtBtn1 & ( 1 << bnBtn1 ) ) ? stPressed : stReleased;
	btnBtn2.stCur = ( prtBtn2 & ( 1 << bnBtn2 ) ) ? stPressed : stReleased;
	
	//Read the raw state of the PmodBTN pins
	PmodBtn1.stCur = ( prtJE1 & ( 1 << bnJE1 ) ) ? stPressed : stReleased;
	PmodBtn2.stCur = ( prtJE2 & ( 1 << bnJE2 ) ) ? stPressed : stReleased;
	PmodBtn3.stCur = ( prtJE3 & ( 1 << bnJE3 ) ) ? stPressed : stReleased;
	PmodBtn4.stCur = ( prtJE4 & ( 1 << bnJE4 ) ) ? stPressed : stReleased;

	//Read the raw state of the PmodSWT pins
	PmodSwt1.stCur = ( prtJA1 & ( 1 << swtJA1 ) ) ? stPressed : stReleased;
	PmodSwt2.stCur = ( prtJA2 & ( 1 << swtJA2 ) ) ? stPressed : stReleased;
	PmodSwt3.stCur = ( prtJA3 & ( 1 << swtJA3 ) ) ? stPressed : stReleased;
	PmodSwt4.stCur = ( prtJA4 & ( 1 << swtJA4 ) ) ? stPressed : stReleased;

	// Update state counts.
	btnBtn1.cst = ( btnBtn1.stCur == btnBtn1.stPrev ) ? btnBtn1.cst + 1 : 0;
	btnBtn2.cst = ( btnBtn2.stCur == btnBtn2.stPrev ) ? btnBtn2.cst + 1 : 0;

	//Update state counts for PmodBTN
	PmodBtn1.cst = (PmodBtn1.stCur == PmodBtn1.stPrev) ? PmodBtn1.cst +1 : 0;
	PmodBtn2.cst = (PmodBtn2.stCur == PmodBtn2.stPrev) ? PmodBtn2.cst +1 : 0;
	PmodBtn3.cst = (PmodBtn3.stCur == PmodBtn3.stPrev) ? PmodBtn3.cst +1 : 0;
	PmodBtn4.cst = (PmodBtn4.stCur == PmodBtn4.stPrev) ? PmodBtn4.cst +1 : 0;

	//Update state counts for PmodSWT
	PmodSwt1.cst = (PmodSwt1.stCur == PmodSwt1.stPrev) ? PmodSwt1.cst +1 : 0;
	PmodSwt2.cst = (PmodSwt2.stCur == PmodSwt2.stPrev) ? PmodSwt2.cst +1 : 0;
	PmodSwt3.cst = (PmodSwt3.stCur == PmodSwt3.stPrev) ? PmodSwt3.cst +1 : 0;
	PmodSwt4.cst = (PmodSwt4.stCur == PmodSwt4.stPrev) ? PmodSwt4.cst +1 : 0;
	
	// Save the current state.
	btnBtn1.stPrev = btnBtn1.stCur;
	btnBtn2.stPrev = btnBtn2.stCur;

	// Save the current state for PmodBTN
	PmodBtn1.stPrev = PmodBtn1.stCur;
	PmodBtn2.stPrev = PmodBtn2.stCur;
	PmodBtn3.stPrev = PmodBtn3.stCur;
	PmodBtn4.stPrev = PmodBtn4.stCur;

	// Save the current state for PmodSWT
	PmodSwt1.stPrev = PmodSwt1.stCur;
	PmodSwt2.stPrev = PmodSwt2.stCur;
	PmodSwt3.stPrev = PmodSwt3.stCur;
	PmodSwt4.stPrev = PmodSwt4.stCur;
	
	// Update the state of button 1 if necessary.
	if ( cstMaxCnt == btnBtn1.cst ) {
		btnBtn1.stBtn = btnBtn1.stCur;
		btnBtn1.cst = 0;
	}
	
	// Update the state of button 2 if necessary.
	if ( cstMaxCnt == btnBtn2.cst ) {
		btnBtn2.stBtn = btnBtn2.stCur;
		btnBtn2.cst = 0;
	}

	//if statements for buttons

	// Update the state of PmodBTN1 if necessary.
	if ( cstMaxCnt == PmodBtn1.cst ) {
		PmodBtn1.stBtn = PmodBtn1.stCur;
		PmodBtn1.cst = 0;
	}
	
	// Update the state of PmodBTN2 if necessary.
	if ( cstMaxCnt == PmodBtn2.cst ) {
		PmodBtn2.stBtn = PmodBtn2.stCur;
		PmodBtn2.cst = 0;
	}

	// Update the state of PmodBTN3 if necessary.
	if ( cstMaxCnt == PmodBtn3.cst ) {
		PmodBtn3.stBtn = PmodBtn3.stCur;
		PmodBtn3.cst = 0;
	}

	// Update the state of PmodBTN4 if necessary.
	if ( cstMaxCnt == PmodBtn4.cst ) {
		PmodBtn4.stBtn = PmodBtn4.stCur;
		PmodBtn4.cst = 0;
	}

	//if statements for switches

	// Update the state of PmodSWT1 if necessary.
	if ( cstMaxCnt == PmodSwt1.cst ) {
		PmodSwt1.stBtn = PmodSwt1.stCur;
		PmodSwt1.cst = 0;
	}
	
	// Update the state of PmodSWT2 if necessary.
	if ( cstMaxCnt == PmodSwt2.cst ) {
		PmodSwt2.stBtn = PmodSwt2.stCur;
		PmodSwt2.cst = 0;
	}

	// Update the state of PmodSWT3 if necessary.
	if ( cstMaxCnt == PmodSwt3.cst ) {
		PmodSwt3.stBtn = PmodSwt3.stCur;
		PmodSwt3.cst = 0;
	}

	// Update the state of PmodSWT4 if necessary.
	if ( cstMaxCnt == PmodSwt4.cst ) {
		PmodSwt4.stBtn = PmodSwt4.stCur;
		PmodSwt4.cst = 0;
	}
    
    
    prtLed4Clr = (1 << bnLed4);
    
    t5_count ++;
    
    if(t5_count > 10)
    {
        error_ic2 =  dspeed0 - ic2_speed;
        ei_ic2 = ei_ic2 + error_ic2;
        	ADCValue1 = (float)ADC1BUF1*3.3/1023.0;
            value = ((kp*error_ic3)+(ki*ei_ic3)+kd*(error_ic3-prev_error_ic3));
        prev_error_ic2 = error_ic2;      
        if(value > 9999)
        {
            value = 9999;
        }
        else if(value < 0)
        {
            value = 0;
        }
        else
        {
            value = value;
        }
        // OC2R = value;
        // OC2RS = value;
        
        
        error_ic3 =  dspeed1 - ic3_speed;
        ei_ic3 = ei_ic3 + error_ic3;
        value = ((kp*error_ic3)+(ki*ei_ic3)+kd*(error_ic3-prev_error_ic3));
        prev_error_ic3 = error_ic3;      
        if(value > 9999)
        {
            value = 9999;
        }
        else if(value < 0)
        {
            value = 0;
        }
        else
        {
            value = value;
        }
        /*
        val_arr[avg_count] = value;
        
        motorspeed = Average(val_arr, 10);
        
        avg_count ++;
        
        if(avg_count > 9)
        {
            avg_count = 0;
        }
        */
      
        


        // OC3R = motorspeed;
        // OC3RS = motorspeed;
        
        

        /*secondary_count++;
        
        if(secondary_count > 5000)
        {
            speed_arr[speed_arr_count] = motorspeed;
            speed_arr_count++;
                
            if (speed_arr_count > 3999 )
            {
                speed_arr_count=0;
            }
            
        }*/
        
            

        t5_count = 0;
    }
    
    prtLed4Clr = (1 << bnLed4 );
    
}

// SET DUTY CYCLES TO ZERO FIRST --> MOTOR CONNECTED 
// DETERMINE NUMBER OF PULSES PER WHEEL ROTATION
void __ISR(_INPUT_CAPTURE_2_VECTOR, ipl4AUTO) _IC2_IntHandler(void)
{
    // clear interrupt flag for Input Capture 2
    IFS0CLR = ( 1 << 9);
    ic2_count ++;
    // buffer not empty is bit 3 of IC2CON.
    
    while(IC2CON & (1 << 3))
    {
    ic2_time = IC2BUF & 0x0000FFFF;
    }
    ic2_delta_time = ic2_time - ic2_previous_time;
    ic2_previous_time = ic2_time;
    
    if(ic2_delta_time < 0)
    {
        ic2_delta_time = ic2_delta_time + 65000;
    }
ic2_speed =53750/(float)ic2_delta_time;// this should be the speed formula but im not sure 
    
// increment counter
}
// This commented block outlined in class:
/* // this interrupt handler does not clear the flag for you after vectoring isr
void __ISR(_INPUT_CAPTURE_3_VECTOR, ipl4) _IC3_IntHandler(void)
{
// clear interrupt flag for Input Capture 3
// increment counter
    
} */
void __ISR(_INPUT_CAPTURE_3_VECTOR, ipl4) _IC3_IntHandler(void)
{
    IFS0CLR = (1 << 13);
    
    while(IC3CON & (1 << 3))
    {
        ic3_time = IC3BUF & 0x0000FFFF;
    }
    ic3_delta_time = ic3_time - ic3_previous_time;
    ic3_previous_time = ic3_time;
    
    
    if(ic3_delta_time < 0)
    {
        ic3_delta_time = ic3_delta_time + 65535;
    }
    
    data_array[ic3_count] = ic3_delta_time;
    
    ic3_count++;
    
  
    if (ic3_count > 200)
    {
        ic3_count = 0;
    }
ic3_speed =53750/((float)ic3_delta_time);
}


/* ------------------------------------------------------------ */
/*				Procedure Definitions							*/
/* ------------------------------------------------------------ */
/***	main
**
**	Synopsis:
**		st = main()
**
**	Parameters:
**		none
**
**	Return Values:
**		does not return
**
**	Errors:
**		none
**
**	Description:
**		Main program module. Performs basic board initialization
**		and then enters the main program loop.
*/

void __ISR(_ADC_VECTOR, ipl3AUTO) _ADC_IntHandler(void) 
{

//   ADC CONVERSION
//   ISR is written assuming that ADC channels are scanned and the interrupt is thrown after all the conversions are complete
//

	prtLed3Set = (1 << bnLed3);   		// turn LED3 on in the beginning of interrupt
	IFS1CLR = ( 1 << 1 );  			// clear interrupt flag for ADC1 Convert Done

//  Read the a/d buffers and convert to voltages
	ADCValue0 = (float)ADC1BUF0*3.3/1023.0;	// Reading AN0(zero), pin 1 of connector JJ -- servo sensor (center)
	ADCValue1 = (float)ADC1BUF1*3.3/1023.0;
    ADCValue2 = (float)ADC1BUF2*3.3/1023.0;		
	Distance0=10.379*pow(ADCValue0,-1.202);
	Distance1=10.379*pow(ADCValue1,-1.202);
    Distance2=10.379*pow(ADCValue2,-1.202);
    
	prtLed3Clr = (1 << bnLed3);   // turn LED3 off at the end of interrupt
}


int main(void) {

	BYTE	stBtn1;
	BYTE	stBtn2;

	BYTE	stPmodBtn1;
	BYTE	stPmodBtn2;
	BYTE	stPmodBtn3;
	BYTE	stPmodBtn4;

	BYTE	stPmodSwt1;
	BYTE	stPmodSwt2;
	BYTE	stPmodSwt3;
	BYTE	stPmodSwt4;
    
    float left_distance = 0;
    DWORD now_time = 0;
    DWORD next_lcd_time = 0;
    
    
    
	DeviceInit();
	AppInit();
	
	

	
    

    OC2R = 0;
    OC2RS = 0;

    
    OC3R = 0;
    OC3RS = 0;
    
	INTDisableInterrupts();
	DelayMs(500);

	   int n1,n2;
    char buffer1 [50],buffer2 [50];
	
/*
	//write to PmodCLS
	SpiEnable();
	SpiPutBuff(szClearScreen, 3);
	DelayMs(4);
	SpiPutBuff(szBacklightOn, 4);
	DelayMs(4);
	SpiPutBuff(szCursorOff, 4);
	DelayMs(4);
	SpiPutBuff("Hello from", 10);
	DelayMs(4);
	SpiPutBuff(szCursorPos, 6);
	DelayMs(4);
	SpiPutBuff("Digilent!", 9);
	DelayMs(2000);
	SpiDisable();
*/

	prtLed1Set	= ( 1 << bnLed1 );
	INTEnableInterrupts();
	while (fTrue)
	{		
		INTDisableInterrupts();
	
		//get data here
		stBtn1 = btnBtn1.stBtn;
		stBtn2 = btnBtn2.stBtn;

		stPmodBtn1 = PmodBtn1.stBtn;
		stPmodBtn2 = PmodBtn2.stBtn;
		stPmodBtn3 = PmodBtn3.stBtn;
		stPmodBtn4 = PmodBtn4.stBtn;

		stPmodSwt1 = PmodSwt1.stBtn;
		stPmodSwt2 = PmodSwt2.stBtn;
		stPmodSwt3 = PmodSwt3.stBtn;
		stPmodSwt4 = PmodSwt4.stBtn;
        
        left_distance = Distance1;
        now_time = ms_count;
	        
	        

		INTEnableInterrupts();
		//configure OCR to go forward

		/*  COMMENTED OUT FOR HW2 WATCH/SFR EXERCISE (lines 354 through 557 in the handout)
		if(stPressed == stPmodBtn1){
			//start motor if button 2 pressed

			MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
			UpdateMotors(); 
			Wait_ms(0x0A00);
			MtrCtrlFwd();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			
		}else if(stPressed == stPmodBtn2){
			//start left turn

			MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
			UpdateMotors(); 
			Wait_ms(0x0A00);
			MtrCtrlBwd();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			
		}else if(stPressed == stPmodBtn3){
			//start right turn

			MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
			UpdateMotors(); 
			Wait_ms(0x0A00);
			MtrCtrlRight();
			UpdateMotors();
			Wait_ms(0x0200);
			MtrCtrlStop();
			UpdateMotors();

		} else if(stPressed == stPmodBtn4){
			//start move backward

			MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
			UpdateMotors(); 
			Wait_ms(0x0A00);
			MtrCtrlLeft();
			UpdateMotors();
			Wait_ms(0x0200);
			MtrCtrlStop();
			UpdateMotors();

		} else if(stPressed == stPmodSwt1){
			//make square to right

			MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
			UpdateMotors(); 
			Wait_ms(0x0A00);  //gives delay to toggle switch
			MtrCtrlFwd();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlRight();
			UpdateMotors();		// first turn
			Wait_ms(0x0180);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlFwd();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlRight();     // second turn
			UpdateMotors();
			Wait_ms(0x0180);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlFwd();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlRight();		// third turn
			UpdateMotors();
			Wait_ms(0x0180);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlFwd();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlRight();
			UpdateMotors();
			Wait_ms(0x0180);
			MtrCtrlStop();
			UpdateMotors();

		} else if(stPressed == stPmodSwt2){
			//make triangle to left

			MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
			UpdateMotors(); 
			Wait_ms(0x0A00); //gives delay to toggle switch
			MtrCtrlFwd();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlLeft();  	//first turn
			UpdateMotors();
			Wait_ms(0x0280);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlFwd();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlLeft();		//second turn
			UpdateMotors();
			Wait_ms(0x0280);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlFwd();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlLeft();		//third turn
			UpdateMotors();
			Wait_ms(0x0280);
			MtrCtrlStop();
			UpdateMotors();
		
		}else if(stPressed == stPmodSwt3){
			// Three point turn around

			MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
			UpdateMotors(); 
			Wait_ms(0x0A00);  //gives delay to toggle switch
			MtrCtrlFwdRight();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlBwdLeft();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0500);
			MtrCtrlFwd();
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();

		}else if(stPressed == stPmodSwt4){
			// dance
			
			MtrCtrlStop();  //stop before sending new data to avoid possible short circuit
			UpdateMotors(); 
			Wait_ms(0x0A00);  //gives delay to toggle switch
			MtrCtrlFwdLeft(); // step left
			UpdateMotors();
			Wait_ms(0x0300);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0200);
			MtrCtrlFwdRight(); // step right
			UpdateMotors();		
			Wait_ms(0x0300);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0200);
			MtrCtrlFwdLeft();  // step left
			UpdateMotors();
			Wait_ms(0x0300);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0200);
			MtrCtrlFwdRight(); // step right
			UpdateMotors();
			Wait_ms(0x0200);
			MtrCtrlStop();
			UpdateMotors();
			Wait_ms(0x0200);
			MtrCtrlLeft();     // spin
			UpdateMotors();
			Wait_ms(0x0800);
			MtrCtrlStop();
			UpdateMotors();
		}  //end if
		*/  // end commented block for HW2
        
        // stop the spin if the time is done
        if(spin_running == fTrue)
        {
            if(now_time >= spin_stop_time)
            {
                MtrCtrlStop();
                UpdateMotors();
                spin_running = fFalse;
            }
        }
        
        // check for two fast hand waves on the left sensor
        if(spin_running == fFalse)
        {
            if(now_time > wave_cooldown_time)
            {
                if(wave_state == WAVE_WAITING_FAR)
                {
                    // wait until the hand is not close
                    if(left_distance > WAVE_FAR_DISTANCE)
                    {
                        wave_state = WAVE_WAITING_CLOSE;
                    }
                }
                else if(wave_state == WAVE_WAITING_CLOSE)
                {
                    // check if hand is close
                    if(left_distance < WAVE_CLOSE_DISTANCE)
                    {
                        wave_state = WAVE_WAITING_FAR_AGAIN;
                    }
                }
                else if(wave_state == WAVE_WAITING_FAR_AGAIN)
                {
                    // check if hand moved away again
                    if(left_distance > WAVE_FAR_DISTANCE)
                    {
                        if((now_time - last_wave_time) > MIN_WAVE_TIME)
                        {
                            last_wave_time = now_time;
                            
                            if(wave_count == 0)
                            {
                                // this means one wave happened
                                wave_count = 1;
                                first_wave_time = now_time;
                            }
                            else
                            {
                                if((now_time - first_wave_time) < DOUBLE_WAVE_TIME)
                                {
                                    // start the spin
                                    MtrCtrlStop();
                                    UpdateMotors();
                                    
                                    MtrCtrlLeft();
                                    UpdateMotors();
                                    
                                    spin_running = fTrue;
                                    spin_stop_time = now_time + SPIN_TIME_360;
                                    
                                    wave_count = 0;
                                    wave_state = WAVE_WAITING_FAR;
                                    wave_cooldown_time = now_time + SPIN_TIME_360 + SPIN_COOLDOWN_TIME;
                                }
                                else
                                {
                                    wave_count = 1;
                                    first_wave_time = now_time;
                                }
                            }
                        }
                        
                        if(spin_running == fFalse)
                        {
                            wave_state = WAVE_WAITING_CLOSE;
                        }
                    }
                }
            }
        }

		// HW2: add code to set SFR values + breakpoint on the next line
		int nice_place_for_a_breakpoint;
		nice_place_for_a_breakpoint = 100;


		nice_place_for_a_breakpoint = 200;
	//LCD update
    if(now_time >= next_lcd_time)
    {
    next_lcd_time = now_time + LCD_UPDATE_TIME;
	SpiEnable();
	SpiPutBuff(szClearScreen, 3);
	DelayMs(4);
	SpiPutBuff(szBacklightOn, 4);
	DelayMs(4);
	SpiPutBuff(szCursorOff, 4);
	DelayMs(4);
    n2=sprintf(buffer2,"Left=%.2f",left_distance);
	SpiPutBuff(buffer2, n2);
	DelayMs(4);
	SpiPutBuff(szCursorPos, 6);
	DelayMs(4);
    if(spin_running == fTrue)
    {
    n1=sprintf(buffer1,"SPIN");
    }
    else
    {
    n1=sprintf(buffer1,"IC2=%.2f",ic2_speed);
    }
	SpiPutBuff(buffer1, n1);
	SpiDisable();
    }
	       
if(spin_running == fFalse)
{
if(Distance0>20){
trisMtrLeftDirClr=(1<<bnMtrLeftDir);
prtMtrLeftDirSet=(1<<bnMtrLeftDir);

trisMtrRightDirClr=(1<<bnMtrRightDir);
prtMtrRightDirClr=(1<<bnMtrRightDir);
dspeed0 = 7 + 0.6 * (Distance0 - 20);
dspeed1 = 7 + 0.6 * (Distance0 - 20);
}
else if(Distance0<7){
trisMtrLeftDirClr=(1<<bnMtrLeftDir);
prtMtrLeftDirClr=(1<<bnMtrLeftDir);

trisMtrRightDirClr=(1<<bnMtrRightDir);
prtMtrRightDirSet=(1<<bnMtrRightDir);
dspeed0=7 + 0.6 * (7 - Distance0);
dspeed1=7 + 0.6 * (7 - Distance0);
	SpiDisable();
       
//if(Distance0>20){
//trisMtrLeftDirClr=(1<<bnMtrLeftDir);
//prtMtrLeftDirSet=(1<<bnMtrLeftDir);
//
//trisMtrRightDirClr=(1<<bnMtrRightDir);
//prtMtrRightDirClr=(1<<bnMtrRightDir);
//dspeed0=7+.6*(Distance0-20);
//dspeed1=7+.6*(Distance0-20);
//}
//else if(Distance0<7){
//trisMtrLeftDirClr=(1<<bnMtrLeftDir);
//prtMtrLeftDirClr=(1<<bnMtrLeftDir);
//
//trisMtrRightDirClr=(1<<bnMtrRightDir);
//prtMtrRightDirSet=(1<<bnMtrRightDir);
//dspeed0 = 7 + 0.6 * (7 - Distance0);
//dspeed1 = 7 + 0.6 * (7 - Distance0);


}
else{
dspeed0=0;
dspeed1=0;
}
}

//if(Distance1<5){
//dspeed0=8;
//DelayMs(700);
//dspeed0=0;
//}


	}  //end while
}  //end main

/* ------------------------------------------------------------ */
/***	DeviceInit
**
**	Synopsis:
**		DeviceInit()
**
**	Parameters:
**		none
**
**	Return Values:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Initializes on chip peripheral devices to the default
**		state.
*/

void DeviceInit() {

	// Configure left motor direction pin and set default direction.
	trisMtrLeftDirClr	= ( 1 << bnMtrLeftDir );
	prtMtrLeftDirClr	= ( 1 << bnMtrLeftDir );	// forward
	
	// Configure right motor direction pin and set default direction.
	trisMtrRightDirClr	= ( 1 << bnMtrRightDir );	
	prtMtrRightDirSet	= ( 1 << bnMtrRightDir );	// forward

	// Configure Output Compare 2 to drive the left motor.
	OC2CON	= (1 << 3) | ( 1 << 2 ) | ( 1 << 1 );	// pwm set up
	OC2R	= dtcMtrStopped;
	OC2RS	= dtcMtrStopped;

	// Configure Output Compare 3.
    
    OC3CON = ( 1 << 3 ) | ( 1 << 2 ) | ( 1 << 1 );	// pwm using timer 3
	OC3R	= dtcMtrStopped;
	OC3RS	= dtcMtrStopped;

	// Configure Timer 2.
	TMR2	= 0;									// clear timer 2 count
	PR2		= 65000;                                // used for input capture time and delta time 

	// Configure Timer 3.
	TMR3	= 0;
	PR3		= 9999;

	// Start timers and output compare units.
	T2CON		= ( 1 << 15 ) | ( 1 << TCKPS20 )|(1 << TCKPS21);		// timer 2 prescale = 8
	OC2CONSET	= ( 1 << 15 );	// enable output compare module 2
	OC3CONSET	= ( 1 << 15 );	// enable output compare module 3
	T3CON		= ( 1 << 15 ) | ( 1 << TCKPS31 ) | ( 1 << TCKPS30); 	// timer 3 prescale = 8

	// Configure Timer 5.
	TMR5	= 0;
	PR5		= 99; // period match every 100 us
	IPC5SET	= ( 1 << 4 ) | ( 1 << 3 ) | ( 1 << 2 ) | ( 1 << 1 ) | ( 1 << 0 ); // interrupt priority level 7, sub 3
	IFS0CLR = ( 1 << 20);
	IEC0SET	= ( 1 << 20);
	
	// Start timers.
	T5CON = ( 1 << 15 ) | ( 1 << 5 ) | ( 1 << 4 ); // fTimer5 = fPb / 8
    

    
        //input capture 2 config
    // ICM <2:0> simple cap event for every rising edge
    //but 15 enables module
    //set priority bits for interrupt
    IC2CON = (1 << 15) | (1 << 7) | ( 1 << 1 ) | ( 1 << 0 ) ; //timer 2 is counter for event capture
    
    
    //IFS0<9>IEC0<9>IPC2<12:10>IPC2<9:8>
    
    IFS0CLR = (1 << 9);
    IFS0SET = (1 << 9);
    
    
    IEC0SET = (1 << 9);
    
    
    IPC2SET = (1 << 12 ) ; // setting 4 for interrupt priority 4
    
    IEC0SET = (1 << 13);
    IFS0CLR = (1 << 13);
    IFS0SET = (1 << 13);
    
    IC3CON = (1 << 15) | (1 << 7) | ( 1 << 1 ) | ( 1 << 0 ) ; //timer 2 is counter for event capture
    
    
    IPC3SET = (1 << 12 ) ; // setting 4 for interrupt priority 4
    
    
	//enable SPI
	SpiInit();
    
    InitLeds();
    
    // ADC init
    
    //	CONFIGURE ADC
	AD1PCFG	= 0XFFF8;
	// CONFIGURE AN0, AN1, and AN2 AS ANALOG INPUTS
	AD1CON1	= 0X00E4;   
	// BIT 7-5 SSRC 111 = INTERNAL COUNTER ENDS SAMPLING AND STARTS CONVERSION
	// BIT 4 CLRASM 0 = Normal Operation, buffer contents will be overwritten by the next conversion sequence
	// BIT 2 ASAM 1 = SAMPLING BEGINS immediately after conversion completes; SAMP bit is automatically set
	// BIT 1 SAMP 0 = ADC IS NOT SAMPLING
	// BUT 0 DONE 0 = STATUS BIT
	AD1CON2	= 0X0408; // 0000 0100 0000 1000     				
	// BIT 10 CSCNA 1 = SCAN INPUTS
	// BIT 2-3 SMPL 1-1 = ONE INTERRUPT AFTER EVERY THIRD CONVERSION
	AD1CON3	= 0X1FFF;
	// BIT 15 ADRC 0 = CLOCK DERIVED FROM PERIPHERAL BUS CLOCK
	// SAMC AND ADCS - I NEED TO READ MORE ABOUT TIMING TO UNDERSTAND THE FUNCTION OF THESE TWO VARIABLES
	AD1CHS	= 0X00000000;   // 32 bit SFR  
	AD1CSSL	= 0X0007;  
	// CSSL = SCAN CHANNELS 2,1 and 0
	IPC6SET	= ( 1 << 27 ) | ( 1 << 26 ); // ADC interrupt priority level 3, sub 0
	IFS1CLR	= 2;    // CLEAR ADC INTERRUPT FLAG
	IEC1SET = 2;	// ENABLE ADC INTERRUPT
	AD1CON1SET = 0X8000;	// 	TURN ADC ON



	// Enable multi-vector interrupts.
	INTEnableSystemMultiVectoredInt();
}

/* ------------------------------------------------------------ */
/***	AppInit
**
**	Synopsis:
**		AppInit()
**
**	Parameters:
**		none
**
**	Return Values:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Performs application specific initialization. Sets devices
**		and global variables for application.
*/

void AppInit() {



}


/* ------------------------------------------------------------ */
/***	Wait_ms
**
**	Synopsis:
**		Wait_ms(WORD)
**
**	Parameters:
**		WORD (range from 0 to 65535)
**
**	Return Values:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Will wait for specified number of milliseconds.  Using a 
**		word variable allows for delays up to 65.535 seconds.  The value
**		in the for statement may need to be altered depending on how your
**		compiler translates this code into AVR assembly.  In assembly, it is
**		possible to generate exact delay values by counting the number of clock
**		cycles your loop takes to execute.  When writing code in C, the compiler
**		interprets the code, and translates it into assembly.  This process is 
**		notoriously inefficient and may vary between different versions of AVR Studio
**		and WinAVR GCC.  A handy method of calibrating the delay loop is to write a 
**		short program that toggles an LED on and off once per second using this 
**		function and using a watch to time how long it is actually taking to
**		complete. 
**
*/

void Wait_ms(WORD delay) {

	WORD i;

	while(delay > 0){

		for( i = 0; i < 375; i ++){
			;;
		}
		delay -= 1;
	}
}




/************************************************************************/
