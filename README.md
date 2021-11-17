# IOS-S310121-R091121-DEVEL
Devolopmet of IOS S310121-R091121 (EmuTOS support)

 **WARNING!**
 
 For development only - DO NOT USE!




**Changelog:**

S310121-R091121_DEVEL0 

                  Added EmuTOS support:
                  
                  - Added new opcodes for LBA disk access: SELLBA, WRITELBA, READLBA, ERRLBA;
                    NOTE: when disk is accessed this way, the FAT based disk access opcodes (SELDISK,
                          SELTRACK, SELSECT, WRITESECT, ERRDISK, READSECT, FLUSHBUFF) must not be 
                          used,and vice versa!
                          
S310121-R091121_DEVEL1 

                  Added EmuTOS support:
                  
                  - Added Systick programmable timer [1..255]ms and a new SETTICK opcode to 
                     set/change the Systck timer;
                     
                  - Added the IRQ for the Serial 2 Rx too;
                  
                  - Changed SETIRQ opcode to add Systick and Serial 2 Rx IRQ enable;
                  
                  NOTE ABOUT IRQ USAGE: When using the CPU interrupt the IOS opcode operations must be 
                  treated as atomic operations, so CPU interrupts must be disabled before and re-enabled 
                  after
                  
                  
