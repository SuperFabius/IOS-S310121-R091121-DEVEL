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
                  
                  NOTE ABOUT IRQ USAGE: When using the CPU interrupts the IOS opcode operations must 
                  be treated as atomic operations to avoid to "break" the opcode call sequence, so 
                  CPU interrupts must be disabled before and re-enabled after.
                  
                  NOTE ABOUT THE SD WITH EMUTOS: the partition on the SD must be a FAT16 max 2GB large 
                  (it is possible to format a FAT16 up to 4GB, but here a "legacy" max 2GB FAT16 is 
                  required).
                  
                  NOTE ABOUT RAM WITH EMUTOS: 1024KB RAM are currently required to run EmuTOS.
                  
 S310121-R091121_DEVEL2
 
                  Added EmuTOS support:
                  - Now EmuTOS is selectable in the same way of CP/M-68K, and associated with 
                     the "Disk Set 1" (but in this case is the whole SD);
                     
                  NOTE: The file DS1NAM.DAT must be copied into the SD root for a correct OS naming.
                  
                  
S310121-R091121_DEVEL3
 
                  Added EmuTOS support:
                  - The EmuTOS selection now uses the EMUTOS.ADR (mandatory) file to set the loading
                     starting address and the execution address as done in the Autoboot mode;
                     

S310121-R091121_DEVEL4
 
                  Changed the "system boot menu" behavior: now choosing option 5 (Change Disk Set...)
                   selects the option 4 too (Load OS from...) if a disk set was changed/selected;
