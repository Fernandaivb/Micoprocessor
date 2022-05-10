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
int toggle = 1;

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
        //
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
    
    
    //Calculate desired frequency
    if (toggle == 1) {
      REQ = pui32ADC0Value[0] /10;
    }
    else {
      REQ = pui32ADC0Value[0] * 100;
    }
    
 
    TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet() / REQ);
 
    
    // Update the interrupt status on the display.    
    IntMasterDisable();
    sprintf(buf, "%d  ", ISRcount);
    GrStringDraw(&sContext, buf, -1, 48, 26, 1);
    sprintf(buf, "%d  ", REQ);
    GrStringDraw(&sContext, buf, -1, 48, 36, 1);
    
    IntMasterEnable();
    ISRcount = 0;
}

void Timer1IntHandler(void) {
    // Clear the timer interrupt.
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    
    if (toggle == 1) {
      // Update the interrupt status on the display.
      IntMasterDisable();
      sprintf(buf, "%d ", ISRcount);
      GrStringDraw(&sContext, buf, -1, 55, 45, 1);
      IntMasterEnable();
    }
    ISRcount++;
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
  //display count value
  
  if(input == 'T') {
    UARTSend("T",1);
    GrContextForegroundSet(&sContext, ClrWhite);
    
    if(toggle == 0) {
      toggle = 1;
      sRect.i16XMin = 0;
      sRect.i16YMin = 45;
      sRect.i16XMax = GrContextDpyWidthGet(&sContext);
      sRect.i16YMax = 55;
      GrContextForegroundSet(&sContext, ClrBlack);
      GrRectFill(&sContext, &sRect);
      GrContextForegroundSet(&sContext, ClrWhite);
      GrStringDraw(&sContext, "Count: ", -1, 16, 46, 0);
    }
    
    else {
      toggle = 0;
      sRect.i16XMin = 0;
      sRect.i16YMin = 45;
      sRect.i16XMax = GrContextDpyWidthGet(&sContext);
      sRect.i16YMax = 55;
      GrContextForegroundSet(&sContext, ClrBlack);
      GrRectFill(&sContext, &sRect);
      GrContextForegroundSet(&sContext, ClrWhite);
    }
  
  }
}
void printMenu() {
  UARTSend("\r\n", 2);
  UARTSend("Menu Selection:", 15);
  UARTSend("\r\n", 2);
  UARTSend("T - Count Toggle", 19);
  UARTSend("\r\n", 2);
}


//*****************************************************************************
//
// This example application demonstrates the use of the timers to generate
// periodic interrupts.
//
//*****************************************************************************
int main(void) {
    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    FPULazyStackingEnable();

    //
    // Set the clocking to run directly from the crystal.
    //
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
    GrStringDrawCentered(&sContext, "timers", -1,
                         GrContextDpyWidthGet(&sContext) / 2, 4, 0);
    //
    // Initialize timer status display.
    GrContextFontSet(&sContext, g_psFontFixed6x8);
    GrStringDraw(&sContext, "SRV: ", -1, 16, 26, 0);
    GrStringDraw(&sContext, "REQ: ", -1, 16, 36, 0);
    GrStringDraw(&sContext, "Count: ", -1, 16, 46, 0);
    
    //UART
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    // Set GPIO A0 and A1 as UART pins.
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    
    // Configure the UART for 115,200, 8-N-1 operation.
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                         (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                          UART_CONFIG_PAR_NONE));
    
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    
    printMenu();
    
    // Enable the peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOG))
    {
    }
    
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD))
    {
    }
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))
    {
    }
    
    //Configure timers
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet());
    TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet());
    IntEnable(INT_TIMER0A);
    IntEnable(INT_TIMER1A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    //Enable timers
    TimerEnable(TIMER0_BASE, TIMER_A);
    TimerEnable(TIMER1_BASE, TIMER_A);
    
    // Configure Peripherals
    GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_2);
    GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_2);

    // Enable processor interrupts.
    IntMasterEnable();
    
    // Loop forever while the timers run.
    //
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