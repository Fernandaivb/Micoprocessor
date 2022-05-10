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
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/pin_map.h"
#include "drivers/cfal96x64x16.h"
#include "driverlib/adc.h"
#include "driverlib/comp.h"
volatile uint32_t ui32Loop;
int32_t local_char;
tRectangle sRect;
uint32_t pui32ADC0Value[1];
char buf[10];
int toggle = 1;
bool LEDtoggle = true;
int clickCount = 0;
int ancCount = 0;
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
void btnIntHandler();
void ANCIntHandler();
void enableButtons();
void enableANC();

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
    GrStringDraw(&sContext, "Fernanda", -1, 5, 46, 0);  
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
  
  if (
  input == 'L') {
    UARTSend("L",1);
    LEDtoggle = !LEDtoggle;
  }
  
  if (input == 'P') {
    UARTSend("P",1);
    splashFace();
    sRect.i16XMin = 0;
    sRect.i16YMin = 0;
    sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
    sRect.i16YMax = 9;
    GrContextForegroundSet(&sContext, ClrDarkBlue);
    GrRectFill(&sContext, &sRect);
    GrContextForegroundSet(&sContext, ClrWhite);
    GrContextFontSet(&sContext, g_psFontFixed6x8);
    GrStringDrawCentered(&sContext, "Lab 5 Interrupts", -1,
                    GrContextDpyWidthGet(&sContext) / 2, 4, 0);
    GrContextFontSet(&sContext, g_psFontFixed6x8);
	
    GrStringDraw(&sContext, "Crosses:", -1, 5, 16, 0);
    GrStringDraw(&sContext, "Presses:", -1, 5, 46, 0); 
  }
  
}
void printMenu() {
  UARTSend("\r\n", 2);
  UARTSend("Menu Selection:", 15);
  UARTSend("\r\n", 2);
  UARTSend("P - Toggle LED", 19);
  UARTSend("\r\n", 2);
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

void btnIntHandler() {

    // Clear the asserted interrupts.
    GPIOIntClear(GPIO_PORTM_BASE, GPIO_PIN_0 | GPIO_PIN_1 | 
                          GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4);
    
    SysCtlDelay(500000);
    
    if(GPIOPinRead(GPIO_PORTM_BASE, GPIO_PIN_0) == 0) {
      clickCount++;
      sRect.i16XMin = 0;
      sRect.i16YMin = 35;
      sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
      sRect.i16YMax = 45;
      GrContextForegroundSet(&sContext, ClrBlack);
      GrRectFill(&sContext, &sRect);
      GrContextForegroundSet(&sContext, ClrWhite);
      
      GrStringDraw(&sContext, "Last: Up", -1, 5, 36, 0);
    }
	
    if(GPIOPinRead(GPIO_PORTM_BASE, GPIO_PIN_1) == 0) {
      clickCount++;
      sRect.i16XMin = 0;
      sRect.i16YMin = 35;
      sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
      sRect.i16YMax = 45;
      GrContextForegroundSet(&sContext, ClrBlack);
      GrRectFill(&sContext, &sRect);
      GrContextForegroundSet(&sContext, ClrWhite);
      GrStringDraw(&sContext, "Last: Down", -1, 5, 36, 0);
    }
	
    if(GPIOPinRead(GPIO_PORTM_BASE, GPIO_PIN_2) == 0) {
      clickCount++;
      sRect.i16XMin = 0;
      sRect.i16YMin = 35;
      sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
      sRect.i16YMax = 45;
      GrContextForegroundSet(&sContext, ClrBlack);
      GrRectFill(&sContext, &sRect);
      GrContextForegroundSet(&sContext, ClrWhite);
      GrStringDraw(&sContext, "Last: Left", -1, 5, 36, 0);
    }
	
    if(GPIOPinRead(GPIO_PORTM_BASE, GPIO_PIN_3) == 0) {
      clickCount++;
      sRect.i16XMin = 0;
      sRect.i16YMin = 35;
      sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
      sRect.i16YMax = 45;
      GrContextForegroundSet(&sContext, ClrBlack);
      GrRectFill(&sContext, &sRect);
      GrContextForegroundSet(&sContext, ClrWhite);
      GrStringDraw(&sContext, "Last: Right", -1, 5, 36, 0);
    }
	
    if(GPIOPinRead(GPIO_PORTM_BASE, GPIO_PIN_4) == 0) {
      clickCount++;
      sRect.i16XMin = 0;
      sRect.i16YMin = 35;
      sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
      sRect.i16YMax = 45;
      GrContextForegroundSet(&sContext, ClrBlack);
      GrRectFill(&sContext, &sRect);
      GrContextForegroundSet(&sContext, ClrWhite);
      GrStringDraw(&sContext, "Last: Select", -1, 5, 36, 0);
    }
    
    GrContextForegroundSet(&sContext, ClrWhite);
    sprintf(buf, "%d  ", clickCount);
    GrStringDraw(&sContext, buf, -1, 60, 46, 1);
}

void ANCIntHandler() {
	// Clear the asserted interrupts.
    ComparatorIntClear(COMP_BASE, 0);
    
    if(ComparatorValueGet(COMP_BASE, 0) == true) {
      ancCount++;
      
      sRect.i16XMin = 0;
      sRect.i16YMin = 25;
      sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
      sRect.i16YMax = 35;
      GrContextForegroundSet(&sContext, ClrBlack);
      GrRectFill(&sContext, &sRect);
      GrContextForegroundSet(&sContext, ClrWhite);
      
      GrStringDraw(&sContext, "Rising", -1, 5, 26, 0);
      
    }
    
    if(ComparatorValueGet(COMP_BASE, 0) == false) {
      ancCount++;
      
      sRect.i16XMin = 0;
      sRect.i16YMin = 25;
      sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
      sRect.i16YMax = 35;
      GrContextForegroundSet(&sContext, ClrBlack);
      GrRectFill(&sContext, &sRect);
      GrContextForegroundSet(&sContext, ClrWhite);
      
      GrStringDraw(&sContext, "Falling", -1, 5, 26, 0);
      
    }
    
    sprintf(buf, "%d   ", ancCount);
    GrStringDraw(&sContext, buf, -1, 60, 16, 1);
    
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
    GrStringDrawCentered(&sContext, "Lab5", -1,
                         GrContextDpyWidthGet(&sContext) / 2, 4, 0);
    //
    // Initialize timer status display.
    GrContextFontSet(&sContext, g_psFontFixed6x8);
    GrStringDraw(&sContext, "Crosses:", -1, 5, 16, 0);
    GrStringDraw(&sContext, "Presses:", -1, 5, 46, 0);  
	
	//Enable LED
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);    
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOG))
    {
    }
    GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_2);
	
	//Enable UART
    IntRegister(INT_UART0, UARTIntHandler);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);

	// Enable buttons, ANC, and Master Interrupts
    enableButtons();
    enableANC();
    IntMasterEnable();
	
    while(1) {
	  while(LEDtoggle) {
		// Turn on the LED.
		GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_2, GPIO_PIN_2);
      
		SysCtlDelay(500000);
      
		// Turn off the LED.
		GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_2, 0);
      
		// Delay for a bit.
		SysCtlDelay(500000);
	  }
    }
}

void enableButtons() {
    IntRegister(INT_GPIOM, btnIntHandler);

    // Enable GPIO port for the switches
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
	
    // Check if peripheral access is enabled
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM));
    
    // Enable GPIO pin for each of the switches (SW0-4). Set direction as output, and enable
    GPIOPinTypeGPIOInput(GPIO_PORTM_BASE, GPIO_PIN_0 | GPIO_PIN_1 | 
                          GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4);
   
    GPIOPadConfigSet(GPIO_PORTM_BASE, GPIO_PIN_0 | GPIO_PIN_1 | 
                       GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4, 
                          GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    
    GPIOIntTypeSet(GPIO_PORTM_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | 
                             GPIO_PIN_3 | GPIO_PIN_4, GPIO_FALLING_EDGE);    
    
    IntEnable(INT_GPIOM);
    GPIOIntEnable(GPIO_PORTM_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | 
                                                      GPIO_PIN_3 | GPIO_PIN_4);
}

void enableANC() {
    IntRegister(INT_COMP0, ANCIntHandler);
	
    // Enable GPIO port for the Analog Comparator
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    // Check if peripheral access is enabled
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC));

    // Enable GPIO pin for Comparator 0 (C0). Set direction as input, and enable
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_7);

    // Enable the COMP module.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_COMP0);

    // Wait for the COMP module to be ready.
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_COMP0))
    {
    }
    
    // Configure comparator 0.
    ComparatorConfigure(COMP_BASE, 0,
                         (COMP_TRIG_NONE | COMP_INT_BOTH |
                           COMP_ASRCP_REF | COMP_OUTPUT_NORMAL));

    // Configure the internal voltage reference.
    ComparatorRefSet(COMP_BASE, COMP_REF_1_65V);
    
    IntEnable(INT_COMP0);
    ComparatorIntEnable(COMP_BASE, 0); 
}
