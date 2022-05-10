#include "inc/hw_memmap.h"
#include "inc/hw_timer.h"
  name isr_asm
  section .text:CODE
  extern ISRcount
  public isr_asm_start
isr_asm_start:
  push {lr} ;
  
  MOV R1, #1
  MOV32 R0, 0x40031000
  STR R1, [R0, #0x24]
  MOV32 R2, ISRcount
  LDR R3, [R2]
  ADD R3, R3, #1
  STR R3, [R2]
   
  pop {pc} ; return
  end
  