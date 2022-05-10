//*****************************************************************************
//
// uart_echo.c - Example for reading data from and writing data to the UART in
//               an interrupt driven fashion.
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
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"
#include "driverlib/rom.h"
#include "grlib/grlib.h"
#include "drivers/cfal96x64x16.h"
#define timerOn .95
#define timerOff .05 
tContext sContext;
tRectangle sRect;
int loopCount;
char buf[10];
int32_t local_char;
uint32_t pui32ADC0Value[4];



volatile uint32_t ui32Loop;
//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>UART Echo (my_uart_echo)</h1>
//!
//! This example application utilizes the UART to echo text.  The first UART
//! (connected to the USB debug virtual serial port on the evaluation board)
//! will be configured in 115,200 baud, 8-n-1 mode.  All characters received on
//! the UART are transmitted back to the UART.
//
//*****************************************************************************

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
/*void
UARTIntHandler(void)
{
    uint32_t ui32Status;

    //
    // Get the interrrupt status.
    //
    ui32Status = UARTIntStatus(UART0_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    UARTIntClear(UART0_BASE, ui32Status);

    //
    // Loop while there are characters in the receive FIFO.
    //
    while(UARTCharsAvail(UART0_BASE))
    {
        //
        // Read the next character from the UART and write it back to the UART.
        //
        UARTCharPutNonBlocking(UART0_BASE,
                                   UARTCharGetNonBlocking(UART0_BASE));
    }
}
*/

//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
void
UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
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

void
printMenu() 
{
  UARTSend("\r\n", 2);
  UARTSend("Menu Selection:", 15);
  UARTSend("\r\n", 2);
  UARTSend("M - print this menu", 19);
  UARTSend("\r\n", 2);
  UARTSend("L - toggle Loop count display", 29);
  UARTSend("\r\n", 2);
  UARTSend("F - toggle Flood character enabled", 34);
  UARTSend("\r\n", 2);
  UARTSend("D - toggle flashing LED", 23);
  UARTSend("\r\n", 2);
  UARTSend("A - Print all Analog", 25);
  UARTSend("\r\n", 2);
  UARTSend("B - Print Analog 1", 25);
  UARTSend("\r\n", 2);
  UARTSend("C - Print Analog 2", 25);
  UARTSend("\r\n", 2);
  UARTSend("E - Print Analog 3", 25);
  UARTSend("\r\n", 2);
  UARTSend("Z - Print Graph 1", 25);
  UARTSend("\r\n", 2);
  UARTSend("Y - Print Graph 2", 25);
  UARTSend("\r\n", 2);
  UARTSend("X - Print Graph 3", 25);
  UARTSend("\r\n", 2);
}  

void splashFace();
void analog1();
void analog2();
void analog3();
void redGraph();
void greenGraph();
void blueGraph();

void
processMenu(int32_t input)
{
  int flag_loop=0;
    
  if(input == 'M')
      printMenu();
  
  if(input == 'D')
  {
    UARTSend("D",1);
    while(true) {
      //
      // Turn on the LED.
      //
      GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_2, GPIO_PIN_2);
      //
      // Delay for a bit.
      //
      SysCtlDelay(500000);
      //
      // Turn off the LED.
      //
      GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_2, 0);

      //
      // Delay for a bit.
      //
      SysCtlDelay(500000);
      
      if(UARTCharGetNonBlocking(UART0_BASE) == 'D')
      {
        break;
      }
    }
  }
  
  if(input == 'F')
  {
    while(true) {
      SysCtlDelay(5000000);
      UARTSend("K",1);
      
      if(UARTCharGetNonBlocking(UART0_BASE) == 'F')
      {
        break;
      }
    }
  }
  
  
  if(input == 'L')
  {
    UARTSend("L",1);

    while(flag_loop==0) {
      GrContextForegroundSet(&sContext, ClrWhite);
      sprintf(buf,"Val = %d",loopCount);
      GrStringDrawCentered(&sContext, buf, -1,
                         GrContextDpyWidthGet(&sContext) / 2, 20, true);
    
        if(UARTCharGetNonBlocking(UART0_BASE) == 'L'){
       
            sRect.i16XMin = 0;
            sRect.i16YMin = 0;
            sRect.i16XMax = GrContextDpyWidthGet(&sContext);
            sRect.i16YMax = GrContextDpyHeightGet(&sContext);
            GrContextForegroundSet(&sContext, ClrBlack);
            GrRectFill(&sContext, &sRect);
            
           flag_loop=1; 
        }        
    }
  }

  if(input == 'A')
  {
    UARTSend("A",1);

    while(flag_loop==0) {
      //
      // Trigger the ADC conversion.
      //
      ADCProcessorTrigger(ADC0_BASE, 1);
      //
      // Wait for conversion to be completed.
      //
      while(!ADCIntStatus(ADC0_BASE, 1, false))
      {
      }
      //
      // Clear the ADC interrupt flag.
      //
      ADCIntClear(ADC0_BASE, 1);

      //
      // Read ADC Value.
      //
      ADCSequenceDataGet(ADC0_BASE, 1, pui32ADC0Value);
      
      GrContextForegroundSet(&sContext, ClrWhite);
      sprintf(buf, "AIN0 = %d    ", pui32ADC0Value[0]);
      GrStringDraw(&sContext, buf, -1, GrContextDpyWidthGet(&sContext) / 6, 20, true);
      sprintf(buf, "AIN0 = %d    ", pui32ADC0Value[1]);
      GrStringDraw(&sContext, buf, -1, GrContextDpyWidthGet(&sContext) / 6, 30, true);
      sprintf(buf, "AIN0 = %d    ", pui32ADC0Value[2]);
      GrStringDraw(&sContext, buf, -1, GrContextDpyWidthGet(&sContext) / 6, 40, true);
   
      if(UARTCharGetNonBlocking(UART0_BASE) == 'A'){
        sRect.i16XMin = 0;
        sRect.i16YMin = 0;
        sRect.i16XMax = GrContextDpyWidthGet(&sContext);
        sRect.i16YMax = GrContextDpyHeightGet(&sContext);
        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &sRect);
            
        flag_loop=1; 
        }        
    }
  }
  
  if(input == 'B')
  {
    UARTSend("B",1);

    while(flag_loop==0) {
      analog1();
      
      if(UARTCharGetNonBlocking(UART0_BASE) == 'B'){
        sRect.i16XMin = 0;
        sRect.i16YMin = 0;
        sRect.i16XMax = GrContextDpyWidthGet(&sContext);
        sRect.i16YMax = GrContextDpyHeightGet(&sContext);
        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &sRect);
            
        flag_loop=1; 
        }        
    }
  }
  
  if(input == 'C')
  {
    UARTSend("C",1);

    while(flag_loop==0) {
      analog2();
   
      if(UARTCharGetNonBlocking(UART0_BASE) == 'C'){
        sRect.i16XMin = 0;
        sRect.i16YMin = 0;
        sRect.i16XMax = GrContextDpyWidthGet(&sContext);
        sRect.i16YMax = GrContextDpyHeightGet(&sContext);
        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &sRect);
            
        flag_loop=1; 
        }        
    }
  }
  
  if(input == 'E')
  {
    UARTSend("E",1);

    while(flag_loop==0) {
      analog3();
   
      if(UARTCharGetNonBlocking(UART0_BASE) == 'E'){
        sRect.i16XMin = 0;
        sRect.i16YMin = 0;
        sRect.i16XMax = GrContextDpyWidthGet(&sContext);
        sRect.i16YMax = GrContextDpyHeightGet(&sContext);
        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &sRect);
            
        flag_loop=1; 
        }        
    }
  }
  
  
  if (input == 'Z') 
  {
    UARTSend("Z",1);
    while(flag_loop == 0)
    {
      //
      // Trigger the ADC conversion.
      //
      ADCProcessorTrigger(ADC0_BASE, 1);
      //
      // Wait for conversion to be completed.
      //
      while(!ADCIntStatus(ADC0_BASE, 1, false))
      {
      }
      //
      // Clear the ADC interrupt flag.
      //
      ADCIntClear(ADC0_BASE, 1);
      //
      // Read ADC Value.
      //
      ADCSequenceDataGet(ADC0_BASE, 1, pui32ADC0Value);

      redGraph();
      
      /*
      if(UARTCharGetNonBlocking(UART0_BASE) == 'Z'){
        UARTSend("Z",1);
          analog1();
      }
      */
         
      GrContextForegroundSet(&sContext, ClrWhite);
      if(UARTCharGetNonBlocking(UART0_BASE) == 'Z')
      {
        sRect.i16XMin = 0;
        sRect.i16YMin = 10;
        sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
        sRect.i16YMax = 26;
        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &sRect);
			
        flag_loop = 1;
       }
      }
  }
  
  
  if (input == 'X') 
  {
    UARTSend("X",1);
    while(flag_loop == 0)
    {
      //
      // Trigger the ADC conversion.
      //
      ADCProcessorTrigger(ADC0_BASE, 1);
      //
      // Wait for conversion to be completed.
      //
      while(!ADCIntStatus(ADC0_BASE, 1, false))
      {
      }
      //
      // Clear the ADC interrupt flag.
      //
      ADCIntClear(ADC0_BASE, 1);
      //
      // Read ADC Value.
      //
      ADCSequenceDataGet(ADC0_BASE, 1, pui32ADC0Value);

      greenGraph();
         
      GrContextForegroundSet(&sContext, ClrWhite);
      if(UARTCharGetNonBlocking(UART0_BASE) == 'X')
      {
        sRect.i16XMin = 0;
        sRect.i16YMin = 10;
        sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
        sRect.i16YMax = 26;
        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &sRect);
			
        flag_loop = 1;
       }
      }
  }  
  
  
if (input == 'Y') {
    UARTSend("X",1);
    while(flag_loop == 0)
    {
      //
      // Trigger the ADC conversion.
      //
      ADCProcessorTrigger(ADC0_BASE, 1);
      //
      // Wait for conversion to be completed.
      //
      while(!ADCIntStatus(ADC0_BASE, 1, false))
      {
      }
      //
      // Clear the ADC interrupt flag.
      //
      ADCIntClear(ADC0_BASE, 1);
      //
      // Read ADC Value.
      //
      ADCSequenceDataGet(ADC0_BASE, 1, pui32ADC0Value);

      blueGraph();
         
      GrContextForegroundSet(&sContext, ClrWhite);
      if(UARTCharGetNonBlocking(UART0_BASE) == 'Y')
      {
        sRect.i16XMin = 0;
        sRect.i16YMin = 10;
        sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
        sRect.i16YMax = 26;
        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &sRect);
			
        flag_loop = 1;
       }
      }
  }    
  
if (input == 'W') 
  {
    UARTSend("W",1);
    while(flag_loop == 0)
    {
      //
      // Trigger the ADC conversion.
      //
      ADCProcessorTrigger(ADC0_BASE, 1);
      //
      // Wait for conversion to be completed.
      //
      while(!ADCIntStatus(ADC0_BASE, 1, false))
      {
      }
      //
      // Clear the ADC interrupt flag.
      //
      ADCIntClear(ADC0_BASE, 1);
      //
      // Read ADC Value.
      //
      ADCSequenceDataGet(ADC0_BASE, 1, pui32ADC0Value);
      
      redGraph();
      blueGraph();
      greenGraph();

         
      GrContextForegroundSet(&sContext, ClrWhite);
      if(UARTCharGetNonBlocking(UART0_BASE) == 'W')
      {
        sRect.i16XMin = 0;
        sRect.i16YMin = 10;
        sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
        sRect.i16YMax = 26;
        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &sRect);
			
        flag_loop = 1;
       }
      }
  }    
  
}

void splashFace()
{
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
    GrStringDrawCentered(&sContext, "Fernanda", -1,
                         GrContextDpyWidthGet(&sContext) / 2, 35, 0);
    //
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

void redGraph()
{
  sRect.i16XMin = 0;
  sRect.i16YMin = 15;
  sRect.i16XMax = (pui32ADC0Value[0] / 42);
  sRect.i16YMax = 30;
  GrContextForegroundSet(&sContext, ClrRed);
  GrRectFill(&sContext, &sRect);
  
  sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
  GrContextForegroundSet(&sContext, ClrBlack);
  GrRectFill(&sContext, &sRect);
}

void blueGraph()
{
  sRect.i16XMin = 0;
  sRect.i16YMin = 30;
  sRect.i16XMax = (pui32ADC0Value[1] / 42);
  sRect.i16YMax = 45;
  GrContextForegroundSet(&sContext, ClrBlue);
  GrRectFill(&sContext, &sRect);
  
  sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
  GrContextForegroundSet(&sContext, ClrBlack);
  GrRectFill(&sContext, &sRect);
}

void greenGraph()
{

  sRect.i16XMin = 0;
  sRect.i16YMin = 45;
  sRect.i16XMax = (pui32ADC0Value[2] / 42);
  sRect.i16YMax = 60;
  GrContextForegroundSet(&sContext, ClrGreen);
  GrRectFill(&sContext, &sRect);
  
  sRect.i16XMax = GrContextDpyWidthGet(&sContext) - 1;
  GrContextForegroundSet(&sContext, ClrBlack);
  GrRectFill(&sContext, &sRect);
}

void analog1() {
  //
  // Trigger the ADC conversion.
  //
  ADCProcessorTrigger(ADC0_BASE, 1);
  //
  // Wait for conversion to be completed.
  //
  while(!ADCIntStatus(ADC0_BASE, 1, false))
  {
  }
  //
  // Clear the ADC interrupt flag.
  //
  ADCIntClear(ADC0_BASE, 1);
  //
  // Read ADC Value.
  //
  ADCSequenceDataGet(ADC0_BASE, 1, pui32ADC0Value);
      
  GrContextForegroundSet(&sContext, ClrWhite);
  sprintf(buf, "AIN0 = %d    ", pui32ADC0Value[0]);
  GrStringDraw(&sContext, buf, -1, GrContextDpyWidthGet(&sContext) / 6, 40, true);
}

void analog2(){
  //
  // Trigger the ADC conversion.
  //
  ADCProcessorTrigger(ADC0_BASE, 1);
  //
  // Wait for conversion to be completed.
  //
  while(!ADCIntStatus(ADC0_BASE, 1, false))
  {
  }
  //
  // Clear the ADC interrupt flag.
  //
  ADCIntClear(ADC0_BASE, 1);
  //
  // Read ADC Value.
  //
  ADCSequenceDataGet(ADC0_BASE, 1, pui32ADC0Value);
      
  GrContextForegroundSet(&sContext, ClrWhite);
  sprintf(buf, "AIN0 = %d    ", pui32ADC0Value[1]);
  GrStringDraw(&sContext, buf, -1, GrContextDpyWidthGet(&sContext) / 6, 20, true);
}
     
void analog3(){
  //
  // Trigger the ADC conversion.
  //
  ADCProcessorTrigger(ADC0_BASE, 1);
  //
  // Wait for conversion to be completed.
  //
  while(!ADCIntStatus(ADC0_BASE, 1, false))
  {
  }
  //
  // Clear the ADC interrupt flag.
  //
  ADCIntClear(ADC0_BASE, 1);
  //
  // Read ADC Value.
  //
  ADCSequenceDataGet(ADC0_BASE, 1, pui32ADC0Value);
      
  GrContextForegroundSet(&sContext, ClrWhite);
  sprintf(buf, "AIN0 = %d    ", pui32ADC0Value[2]);
  GrStringDraw(&sContext, buf, -1, GrContextDpyWidthGet(&sContext) / 6, 20, true);
}

//*****************************************************************************
//
// This example demonstrates how to send a string of data to the UART.
//
//*****************************************************************************
int
main(void)
{
    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    FPULazyStackingEnable();
    splashFace();
    //
    // Set the clocking to run directly from the crystal.
    //
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    //
    // Initialize the display driver.
    //
    //CFAL96x64x16Init();

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
    GrStringDrawCentered(&sContext, "my-uart-echo", -1,
                         GrContextDpyWidthGet(&sContext) / 2, 4, 0);

    //
    // Initialize the display and write some instructions.
    //
    GrStringDrawCentered(&sContext, "Connect a", -1,
                         GrContextDpyWidthGet(&sContext) / 2, 30, false);
    GrStringDrawCentered(&sContext, "terminal", -1,
                         GrContextDpyWidthGet(&sContext) / 2, 40, false);
    GrStringDrawCentered(&sContext, "to UART0.", -1,
                         GrContextDpyWidthGet(&sContext) / 2, 50, false);
    GrStringDrawCentered(&sContext, "115000,N,8,1", -1,
                         GrContextDpyWidthGet(&sContext) / 2, 60, false);

    //
    // Enable the peripherals used by this example.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    
    //enable G port for the LED.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    //enable GPIO pin for LED
    GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_2);

    //
    // Enable processor interrupts.
    // Disable UART interrupts function 1
    //IntMasterEnable();

    //
    // Set GPIO A0 and A1 as UART pins.
    //
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    
    //enable D port of Analog pin
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    // Wait for peripheral to be ready.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD))
    {
    }   

    //Enable part of Analog pin (input)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    //
    //
    // Wait for the ADC0 module to be ready.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))
    {
    }
    
    //
    // Configure Pin as ADC type
    //
    GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_2 || GPIO_PIN_3 || GPIO_PIN_4);
    
    //
    //Disable ADC Sequence
    //
    ADCSequenceDisable(ADC0_BASE, 1);
    //
    //Configure Sequence 1
    //
    ADCSequenceConfigure(ADC0_BASE, 1,  ADC_TRIGGER_PROCESSOR, 0);
    //
    //Configure sequencer steps
    //
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH7 | ADC_CTL_IE);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH12 | ADC_CTL_IE);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH13 | ADC_CTL_IE |
                             ADC_CTL_END);
    //
    // Since sample sequence 1 is now configured, it must be enabled.
    //
    ADCSequenceEnable(ADC0_BASE, 1);
 
    //
    // Configure the UART for 115,200, 8-N-1 operation.
    //
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    //
    // Enable the UART interrupt.
    //
    // Disable UART interrupts functions 2 and 3
    //IntEnable(INT_UART0);
    //UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);

    //
    // Loop forever echoing data through the UART.
    //
    while(1)
    { loopCount++;
      if (UARTCharsAvail(UART0_BASE)) {
        local_char = UARTCharGetNonBlocking(UART0_BASE);
        processMenu(local_char);     
      }
       
    }
    
    //tracks the number of characters recieved
    //gNumCharRecv();
}
