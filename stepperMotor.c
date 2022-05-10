// Fernanda Villafana Benitez
// Modified code from timers.c to complete Microprocessors Lab4
//*****************************************************************************
//
// timers.c - Timers example.
//
// Copyright (c) 2011-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the DK-TM4C123G Firmware Package.
//
//*****************************************************************************
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/rom.h"
#include "grlib/grlib.h"
#include "utils/uartstdio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "drivers/cfal96x64x16.h"
int32_t local_char;
tRectangle sRect;
volatile uint32_t ui32Loop;
uint32_t pui32ADC0Value[1];

char buf[10];
uint32_t REQ = 0;
uint32_t ISRcount = 0;
int ADCtoggle = 1;
int toggle = 1;
uint32_t stepRPM = 60;
uint32_t RPM = 60;

int mode = 0;
int position = 0;
bool reverse = 0;

const uint8_t half_step_array[8] = {0x0C, 0x04, 0x06, 0x02, 0x03, 0x01, 0x09, 0x08};
const uint8_t full_step_array[4] = {0x0C, 0x06, 0x03, 0x09};
const uint8_t wave_drive_array[4] = {0x08, 0x04, 0x02, 0x01};

#define MOTOR_OFF (0x00)
#define STEPS_PER_REV (200)
#define INIT_REV (60)
#define RPM_MIN (1)
#define RPM_MAX (200)
//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Timer (timers)</h1>
//!
//! This example application demonstrates the use of the timers to generate
//! periodic interrupts.  One timer is set up to interrupt once per second and
//! the other to interrupt twice per second; each interrupt handler will toggle
//! its own indicator on the display.
//
//*****************************************************************************

//*****************************************************************************
//
// Flags that contain the current value of the interrupt indicator as displayed
// on the CSTN display.
//
//*****************************************************************************
uint32_t g_ui32Flags;

//*****************************************************************************
//
// Graphics context used to show text on the CSTN display.
//
//*****************************************************************************
tContext sContext;
void printMenu();
void processMenu(int32_t input);

//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count) {
    //
    // Loop while there are more characters to send.
    //
    while(ui32Count--)
    {
        //
        // Write the next character to the UART.
        UARTCharPut(UART0_BASE, *pui8Buffer++);
    }
}
//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void 
__error__(char *pcFilename, uint32_t ui32Line) {
  //while(1)
}
#endif

//*****************************************************************************
//
// ISR Interupt Handlers
//
//*****************************************************************************

void Timer0IntHandler(void) {

    // Clear the timer interrupt.
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    ADCProcessorTrigger(ADC0_BASE, 3);
    ADCSequenceDataGet(ADC0_BASE, 3, pui32ADC0Value);
    
    if (mode == 1) {
      GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, full_step_array[position]);
      if (reverse == false) {
        position++;
        if (position >= 4)
          position = 0;
      }
      if (reverse == true) {
        position--;
        if (position >= -1)
          position = 3;
      }
    }
    
    if (mode == 2) {
      GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, wave_drive_array[position]);
      if (reverse == false) {
        position++;
        if (position >= 4)
          position = 0;
      }
      if (reverse == true) {
        position--;
        if (position >= -1)
          position = 3;
      }
    }
    
    if (mode == 0) {
      GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0x00);
    }
    
    /*
    if (mode == 3) {
      GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, reverse_step_array[position]);
      position++;
      if (position == 4) {
      position = 0;
     }
    }
    */
  
    // Update the interrupt status on the display.    
    IntMasterDisable();
    IntMasterEnable();
    ISRcount = 0;
}

void UARTIntHandler(void) {
    uint32_t ui32Status;

    // Get the interrrupt status.
    ui32Status = UARTIntStatus(UART0_BASE, true);

    // Clear the asserted interrupts.
    UARTIntClear(UART0_BASE, ui32Status);
    
    if (UARTCharsAvail(UART0_BASE)) {
      local_char = UARTCharGetNonBlocking(UART0_BASE);
      if (local_char != -1)
        processMenu(local_char);    
    }
}

void splashFace() {
    //
    // Initialize the graphics context.
    //
    GrContextInit(&sContext, &g_sCFAL96x64x16);
     
    //
    // Change foreground for white text.
    //
    CFAL96x64x16Init();
    GrContextFontSet(&sContext, g_psFontFixed6x8);
    GrContextForegroundSet(&sContext, ClrWhite);
    sprintf(buf, "%d",SysCtlClockGet());
    GrStringDrawCentered(&sContext, buf, -1,
                         GrContextDpyWidthGet(&sContext) / 2, 35, 0);
    
    // Delay for a bit.
    //
    SysCtlDelay(15000000);
    
    sRect.i16XMin = 0;
    sRect.i16YMin = 0;
    sRect.i16XMax = GrContextDpyWidthGet(&sContext);
    sRect.i16YMax = GrContextDpyHeightGet(&sContext);
    GrContextForegroundSet(&sContext, ClrBlack);
    GrRectFill(&sContext, &sRect);
}

void processMenu(int32_t input) {  
  if(input == 'M') {
      UARTSend("M",1);
      printMenu();
  }
  
  //increase and decrease motor RPM
  if(input == '+') {
    UARTSend("+",1);
    stepRPM++;
    RPM++;
    TimerLoadSet(TIMER0_BASE, TIMER_A, (SysCtlClockGet()) / ((stepRPM * 200)/60));
  }
  
  if(input == '-') {
    UARTSend("-",1);
    stepRPM--;
    RPM--;
    TimerLoadSet(TIMER0_BASE, TIMER_A, (SysCtlClockGet()) / ((stepRPM * 200)/60));
  }
  
  if(input == 'A') {
    UARTSend("A",1);
    mode = 1;
  }
  
  if(input == 'B') {
    UARTSend("B",1);
    mode = 2;
  }
  
  if(input == 'C') {
    UARTSend("C",1);
    mode = 0;
  }
  
  if(input == 'D') {
    UARTSend("D",1);
    mode = 4;
  }

  sprintf(buf, "%d  ", RPM);
  GrStringDraw(&sContext, buf, -1, 48, 26, 1);

}

void printMenu() {
  UARTSend("\r\n", 2);
  UARTSend("Menu Selection:", 15);
  UARTSend("\r\n", 2);
  UARTSend("+ = Increase Motor RPM", 22);
  UARTSend("\r\n", 2);
  UARTSend("- = Decrease Motor RPM", 22);
  UARTSend("\r\n", 2);
  UARTSend("A = Full Drive", 14);
  UARTSend("\r\n", 2);
  UARTSend("B = Wave Drive", 14);
  UARTSend("\r\n", 2);
}

//Calculate RPM steps
int stepCalculation(int value) {
  int steps = value * STEPS_PER_REV;
  return (steps / 4095);
}

//*****************************************************************************
//
// This example application demonstrates the use of the timers to generate
// periodic interrupts.
//
//*****************************************************************************
int main(void) {

    FPULazyStackingEnable();
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);
    
    //
    // Initialize the display driver.
    //
    CFAL96x64x16Init();
    IntMasterDisable();
    splashFace();
    //
    // Initialize the graphics context and find the middle X coordinate.
    //
    GrContextInit(&sContext, &g_sCFAL96x64x16);

    //
    // Fill the top part of the screen with blue to create the banner.
    //
    sRect.i16XMin = 0;
    sRect.i16YMin = 0;
    sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
    sRect.i16YMax = 9;
    GrContextForegroundSet(&sContext, ClrDarkBlue);
    GrRectFill(&sContext, &sRect);

    //
    // Change foreground for white text.
    //
    GrContextForegroundSet(&sContext, ClrWhite);

    //
    // Put the application name in the middle of the banner.
    //
    GrContextFontSet(&sContext, g_psFontFixed6x8);
    GrStringDrawCentered(&sContext, "Lab 7", -1,
                         GrContextDpyWidthGet(&sContext) / 2, 4, 0);
    //
    // Initialize timer status display.
    GrContextFontSet(&sContext, g_psFontFixed6x8);
    GrStringDraw(&sContext, "RPM: ", -1, 16, 26, 0);
    
	
    //UART
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                    (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                     UART_CONFIG_PAR_NONE));
    printMenu();
    
    //Timers
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, 60 * SysCtlClockGet() / stepRPM * 200);
    IntEnable(INT_TIMER0A);
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet());
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER0_BASE, TIMER_A);
    
    //ADC
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD))
    {
    }
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))
    {
    }
    GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_2);
    ADCSequenceDisable(ADC0_BASE, 3);
    ADCSequenceDisable(ADC0_BASE, 3);
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH13 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 3);

    //MOTOR
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOL))
    {
    }
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_0);
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_2);
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_3);
    
    //PUSHBUTTONS
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM))
    {
    }
    GPIOPinTypeGPIOInput(GPIO_PORTM_BASE, GPIO_PIN_0);
    GPIOPadConfigSet(GPIO_PORTM_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    
    //LED
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOG))
    {
    }
    // Configure Peripherals
    GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_2);

    // Enable processor interrupts.
    IntMasterEnable();
    
    while(1) {
      // Turn on the LED.
      GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_2, GPIO_PIN_2);
      
      SysCtlDelay(500000);
      
      // Turn off the LED.
      GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_2, 0);
      
      // Delay for a bit.
      SysCtlDelay(500000);
    }
}
