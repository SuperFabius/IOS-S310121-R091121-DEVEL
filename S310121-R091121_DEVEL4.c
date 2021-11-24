/* ------------------------------------------------------------------------------

S310121-R091121_DEVEL4 - IOS - I/O Subsystem for the 68008 CPU (8MHz)
 
Supported HW ref: A091020-R140221 (68k-MBC);
                  A240721-R270921 (SPP Adapter);
                  A071218-R290319 (uTerm);
                  A221218-R090419 (uCom);
                  and following HW revisions until stated otherwise.


Notes:

1: Supported CPU: 68008 @ 8MHz

3: Embedded FW: S180221-R150521 (sLoad)

4: Utilities:   S160420 (Hex formatting utility)


.........................................................................................
 
WARNING! WARNING! WARNING! WARNING! WARNING! WARNING! WARNING! WARNING! WARNING! WARNING! 
WARNING! WARNING! WARNING! WARNING! WARNING! WARNING! WARNING! WARNING! WARNING! WARNING! 
 
 
        * * * WARNING: THIS IS A TESTING VERSION ONLY FOR DEVELOPMENT! * * *
 
 
WARNING! WARNING! WARNING! WARNING! WARNING! WARNING! WARNING! WARNING! WARNING! WARNING! 
WARNING! WARNING! WARNING! WARNING! WARNING! WARNING! WARNING! WARNING! WARNING! WARNING! 
.........................................................................................
 
  

IMPORTANT NOTE:

The "Virtual I/O engine" is tuned to work with a CPU @ 8MHz. 
Do not attempt to use a different CPU clock.
To use a different CPU clock the "Virtual I/O engine" may require a new tune-up.



---------------------------------------------------------------------------------



CHANGELOG:


S310121           First revision;
S310121-R130421   Corrected the Autoboot boot mode loading parameters;
                  Corrected some comments;
S310121-R150421   Changed the text of the "Baud Recovery" and "Bus Error" messages;
S310121-R250521   Improved the Autoboot mode: now it is possible specify optionally  
                   custom loading parameters in the text file AUTOBIN.ADR;
                  Updated sLoad S180221-R150521;
                  Changed the behavior of the Baud Recovery: now can be triggered only 
                   if at least one serial port is set to a not default (115200) value;
                  Added the SERIAL 2 TX Opcode in the list in the STORE OPCODE comments;
S310121-R010721   Changed the slew rate of RC0 (CLK) as some 68008 CPU seems not to like the
                   "high" setting for this signal causing erratic behaviors;
S310121-R220721   Corrected the uTerm reset at boot time (MCU_RTS now set as output inside MCC);
S310121-R231021   Added support for the SPP Adapter (A240721-R270921) with new opcodes: SETSPP, 
                   WRSPP, GETSPP;
                  Please note that now when the GPIO is set to operate as an SPP port all 
                   the GPIO write Opcodes (GPIOA Write, GPIOB Write, IODIRA Write, IODIRB Write, 
                   GPPUA Write, GPPUB Write) are ignored/disabled;
                  I2C speed now at 400KHz;
 
S310121-R091121_DEVEL0                                                                      ****************************************
                  Added EmuTOS support:
                  - Added new opcodes for LBA disk access: SELLBA, WRITELBA, READLBA, ERRLBA;
                    NOTE: when disk is accessed this way, the FAT based disk access opcodes (SELDISK,
                          SELTRACK, SELSECT, WRITESECT, ERRDISK, READSECT, FLUSHBUFF) must not be used, 
                          and vice versa!
 
S310121-R091121_DEVEL1                                                                      ****************************************
                  Added EmuTOS support:
                  - Added Systick programmable timer [1..255]ms and a new SETTICK opcode to 
                     set/change the Systck timer;
                  - Added the IRQ for the Serial 2 Rx too;
                  - Changed SETIRQ opcode to add Systick and Serial 2 Rx IRQ enable;
 
S310121-R091121_DEVEL2                                                                      ****************************************
                  Added EmuTOS support:
                  - Now EmuTOS is selectable in the same way of CP/M-68K, and associated with 
                     the "Disk Set 1" (but in this case is the whole SD);
 
S310121-R091121_DEVEL3                                                                      ****************************************
                  Added EmuTOS support:
                  - The EmuTOS selection now uses the EMUTOS.ADR (mandatory) file to set the loading
                     starting address and the execution address as done in the Autoboot mode;
 
S310121-R091121_DEVEL4                                                                      ****************************************
                  Changed the "system boot menu" behavior: now choosing option 5 (Change Disk Set...) selects the
                   option 4 too (Load OS from...) if a disk set was changed/selected;


---------------------------------------------------------------------------------

Credits:

PetitFS link: http://elm-chan.org/fsw/ff/00index_p.html
 
I2C Check routines link: https://www.microchip.com/forums/FindPost/1118405 
 
---------------------------------------------------------------------------------

PetitFS license:

Petit FatFs Module Source Files R0.03a               (C)ChaN, 2019


FILES

  pff.h      Common include file for Petit FatFs and application module.
  pff.c      Petit FatFs module.
  diskio.h   Common include file for Petit FatFs and disk I/O module.
  diskio.c   Skeleton of low level disk I/O module.

  Low level disk I/O module is not included in this archive because the Petit
  FatFs module is only a generic file system layer and not depend on any
  specific storage device. You have to provide a low level disk I/O module that
  written to control your storage device.



AGREEMENTS

 Petit FatFs module is an open source software to implement FAT file system to
 small embedded systems. This is a free software and is opened for education,
 research and commercial developments under license policy of following terms.

  Copyright (C) 2019, ChaN, all right reserved.

 * The Petit FatFs module is a free software and there is NO WARRANTY.
 * No restriction on use. You can use, modify and redistribute it for
   personal, non-profit or commercial use UNDER YOUR RESPONSIBILITY.
 * Redistributions of source code must retain the above copyright notice.



---------------------------------------------------------------------------------

MPLABX Info:

Compiled with MPLABX v5.50 + XC8 v2.32 + MCC v5.0.3

    Others Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.7
        Device            :  PIC18F47Q10

--------------------------------------------------------------------------------- */

// ------------------------------------------------------------------------------
//
//  Headers
//
// ------------------------------------------------------------------------------


#include <string.h>
#include "mcc_generated_files/mcc.h"
#include "mcc_generated_files/examples/i2c2_master_example.h"
#include "pff.h"
#include "diskio.h"     // *****************************************************************************************************************
#include "diskio_PIC18F47Q10.h"
#include "rom.h"


// ------------------------------------------------------------------------------
//
//  Debug switches (0 = OFF, 1 = ON))
//
// ------------------------------------------------------------------------------


#define     DEBUG_L       0                                 // Boot load debug
#define     DEBUG_VW      0                                 // Virtual Write I/O debug
#define     DEBUG_VR      0                                 // Virtual Read I/O debug
#define     DEBUG_VI      0                                 // Virtual IRQ ACK I/O debug
#define     DEBUG_RDSECT  0                                 // READSECT opcode debug
#define     DEBUG_SELDSK  0                                 // SELDISK opcode debug
#define     DEBUG_FLUSH   0                                 // flushBuffer() function debug (deferred write)
#define     DEBUG_FLUSHB  0                                 // FLUSHBUFF opcode debug
#define     DEBUG_WRSECT  0                                 // WRITESECT opcode debug
#define     DEBUG_GPIOPAR 0                                 // Print GPIO parameters
#define     DEBUG_AUTOBIN 0                                 // Print Autoboot parameters
#define     DEBUG_READLBA 0                                 // READLBA opcode debug ***************************************************
#define     DEBUG_WRLBA   0                                 // WRITELBA opcode debug ***************************************************
#define     DEBUG_UNKIRQ  0                                 // Unknown IRQ error triggered with the User key ***************************


// ------------------------------------------------------------------------------
//
// Hardware definitions for A091020-R140221 
//
// ------------------------------------------------------------------------------


// Data bus direction & pullups
#define D0_7_DIR        TRISD                               // D0-D7 data bus direction: 1 = input, 0 = output
#define D0_7_PULLUP     WPUD                                // D0-D7 data bus pullup: 1 = on, 0 = off

// CLC1 & CLC2 defined digital networks definitions for "virtual signals" DT_EN_ & S_STEP_
#define Set_DT_EN__Low    CLC2POL = CLC2POL & ~_CLC2POL_LC2POL_MASK       // Set DT_EN_ LOW (output not inverted)
#define Set_DT_EN__High   CLC2POL = CLC2POL | _CLC2POL_LC2POL_MASK        // Set DT_EN_ HIGH (output inverted)
#define Set_S_STEP__Low   CLC1POL = CLC1POL & ~_CLC1POL_LC1G2POL_MASK     // Set S_STEP_ LOW (gate 2 output not inverted)
#define Set_S_STEP__High  CLC1POL = CLC1POL | _CLC1POL_LC1G2POL_MASK      // Set S_STEP_ HIGH (gate 2 output  inverted)
#define DTACK_ CLC1CONbits.LC1OUT


// ------------------------------------------------------------------------------
//
// Hardware definitions for RTC Module Option (see DS3231 datasheet)
//
// ------------------------------------------------------------------------------


#define   DS3231_RTC    0x68                                // DS3231 I2C address
#define   DS3231_SECRG  0x00                                // DS3231 Seconds Register
#define   DS3231_STATRG 0x0F                                // DS3231 Status Register


// ------------------------------------------------------------------------------
//
// Hardware definitions for the GPE Option (see MCP23017 datasheet)
//
// ------------------------------------------------------------------------------


#define   GPIOEXP_ADDR  0x20                                // I2C module address
#define   IODIRA_REG    0x00                                // MCP23017 internal register IODIRA
#define   IODIRB_REG    0x01                                // MCP23017 internal register IODIRB
#define   GPPUA_REG     0x0C                                // MCP23017 internal register GPPUA
#define   GPPUB_REG     0x0D                                // MCP23017 internal register GPPUB
#define   GPIOA_REG     0x12                                // MCP23017 internal register GPIOA
#define   GPIOB_REG     0x13                                // MCP23017 internal register GPIOB


// ------------------------------------------------------------------------------
//
// Others file names and related parameters
//
// ------------------------------------------------------------------------------


#define             AUTOFN              "AUTOBOOT.BIN"      // User executable binary file
#define             AUTOADR             "AUTOBOOT.ADR"      // Optional file to specify custom loading parameters
#define             DS_OSNAME           "DSxNAM.DAT"        // File with the OS name for Disk Set "x" (from DS0NAM.DAT to DS9NAM.DAT)
#define             EHBASFN             "EBASIC.BIN"        // Enhanced 68K Basic binary file
#define             EMUDISK             "DSxNyy.DSK"        // Generic disk name (from DS0N00.DSK to DS9N99.DSK)
#define             CPM68FN             "CPM68.BIN"         // CP/M-68K binary file
#define             EMUTOSFN            "EMUTOS.BIN"        // EmuTOS binary file ******************************************************
#define             EMUTOSADR           "EMUTOS.ADR"        // File to specify custom loading parameters foe EMUTOS.BIN ****************
const uint32_t      AUTLOADADR          =   0x00400;        // Default loading address for the AUTOFN file 
const uint32_t      AUTSTARADR          =   0x00400;        // Default starting address for the AUTOFN file execution 
const uint32_t      EHBASLDADR          =   0x00400;        // Loading address for the EHBASFN file
const uint32_t      EHBASSTADR          =   0x00400;        // Starting address for the EHBASFN file execution
const uint32_t      CPM68LDADR          =   0x00400;        // Loading address for the CPM68FN file
const uint32_t      CPM68STADR          =   0x00400;        // Starting address for the CPM68FN file execution
//const uint32_t      ETOSLDADR           =   0x02140;        // Loading address for the EMUTOSFN file ***********************************
//const uint32_t      ETOSSTADR           =   0x02140;        // Starting address for the EMUTOSFN file execution ************************


// ------------------------------------------------------------------------------
//
// 68008 CPU opcodes and IRQ vectors definitions
//
// ------------------------------------------------------------------------------


// CPU OpCodes
const uint16_t     STOP_OPC             =   0x4E72;
const uint16_t     STOP_OPC_DATA        =   0x2700;
const uint16_t     NOP_OPC              =   0x4E71;
const uint16_t     TRAP_OPC             =   0x4E40;
const uint16_t     MOVEA_L_A0_OPC       =   0x207C;
const uint16_t     MOVE_B_A0_INC_OPC    =   0x10FC;
const uint16_t     MOVE_W_A0_INC_OPC    =   0x30FC;
const uint16_t     MOVE_L_A0_INC_OPC    =   0x20FC;
const uint16_t     JMP_W_OPC            =   0x4EF8;

// Vectors (68008 CPU initialization)
const uint32_t     RES_SSP_VECT         =   0x01000;
const uint32_t     RES_PC_VECT          =   0x00400;

// IRQ vectors (68008 CPU)
#define            SER1RXIRQ_VECTOR     0x40                // Set the IRQ vector for the Serial 1 Rx event (User Vector 0x40)   **************************** 
#define            SER2RXIRQ_VECTOR     0x41                // Set the IRQ vector for the Serial 2 Rx event (User Vector 0x41)  ****************************
#define            SYSTICKIRQ_VECTOR    0x42                // Set the IRQ vector for the Systick event (User Vector 0x42)      ****************************
//#define            UNKNOWNIRQ_VECTOR    0x43                // Set the IRQ vector for the Unknown IRQ event (User Vector 0x43) ******************* ?????


// ------------------------------------------------------------------------------
//
// Functions definitions
//
// ------------------------------------------------------------------------------


// TMR0
void Timer0_ISR(void);
uint32_t millis(void);
void WaitAndBlink(void (* LongPressedUserKeyCall)(void));

// Lite/Full HW setup routines
inline uint8_t CheckLiteConfig(void);
inline void SetNegatedIO_M(void);

// I2C check routines
static  i2c2_operations_t  NackAddressHandler(void *Ptr);
uint8_t I2C_check( uint8_t  deviceAddress);
//void I2C_AddressScan(void);

// RTC Module routines
uint8_t decToBcd(uint8_t val);
uint8_t bcdToDec(uint8_t val);
void readRTC(uint8_t *seconds, uint8_t *minutes, uint8_t *hours, uint8_t *day, uint8_t *month, uint8_t *year, uint8_t *tempC);
void writeRTC(uint8_t seconds, uint8_t minutes, uint8_t hours, uint8_t day, uint8_t month, uint8_t year);
uint8_t autoSetRTC(void);
void printDateTime(uint8_t readSourceFlag);
void print2digit(uint8_t data);
uint8_t isLeapYear(uint8_t yearXX);
inline void ChangeRTC();
uint8_t toInt(char str[2]);

// 68008 CPU boot routines
void SetDefaultBaud(void);
uint16_t calcSPxBRG(uint8_t serSpeed);
uint8_t DATAEE_Update(uint16_t bAdd, uint8_t bData);
void busFatalErr(uint8_t errCode);
inline void singleStepDrive(void);
void skipWriteCycles(void);
void feedReadCycle(uint8_t byte);
void printMsg1(void);

// SD Disk routines
void openFileSD(const char *  fileNameSD);
void printOsName(uint8_t currentDiskSet);
uint8_t mountSD(FATFS* fatFs);
uint8_t openSD(const char* fileName);
uint8_t readSD(void* buffSD, uint16_t* numReadBytes);
uint8_t writeSD(void* buffSD, uint16_t* numWrittenBytes);
uint8_t seekSD(uint16_t sectNum);
void printErrSD(uint8_t opType, uint8_t errCode, const char* fileName);
void waitKey(void);

// Execute Write Opcode
inline void execWriteOpcode(void);
void flushBuffer(void);

// Execute Read opcode
inline void execReadOpcode(void);


// ------------------------------------------------------------------------------
//
//  Constants
//
// ------------------------------------------------------------------------------


const uint8_t           maxDtackRetry       =       250;        // Max retries waiting for DTACK_ = 1
const uint8_t           maxWriteCycles      =       4;          // Max consecutive write bus cycles
const char              compTimeStr[]           = __TIME__;    // Compile timestamp string
const char              compDateStr[]           = __DATE__;    // Compile datestamp string
const uint8_t           daysOfMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

const unsigned char     bootModeAddr = 10;          // Internal EEPROM address for boot mode storage
const unsigned char     autoexecFlagAddr  = 11;     // Internal EEPROM address for AUTOEXEC flag storage
const unsigned char     diskSetAddr  = 12;          // Internal EEPROM address for the current Disk Set [0..9]
const unsigned char     Ser1Addr    = 13;           // Internal EEPROM address for Serial 1 speed
const unsigned char     Ser2Addr    = 14;           // Internal EEPROM address for Serial 2 speed

const uint8_t           maxDiskSet   = 2;           // Number of configured Disk Sets **********************************************
const uint8_t           maxDiskNum   = 99;          // Max number of virtual disks


// ------------------------------------------------------------------------------
//
//  Global variables
//
// ------------------------------------------------------------------------------


// General purpose variables
volatile uint32_t       millisCounter;              // Used for the millis() function with the TMR0 ISR
uint32_t                timeStamp;                  // Timestamp for the Systick timer
uint8_t                 ioAddress;                  // Virtual I/O address. Only four possible addresses are valid [0x00..0x03]
uint8_t                 ioData;                     // Data byte used for the I/O operation
char                    inChar;                     // Input char from serial
uint8_t                 ioOpcode    =   0xFF;       // I/O operation code or Opcode (0xFF means "No Operation")
uint16_t                ioByteCnt;                  // Exchanged bytes counter during an I/O operation
uint8_t                 CPUIntRx1En  =   0;         // IRQ enable on serial 1 Rx flag
uint8_t                 CPUIntRx2En  =   0;         // IRQ enable on serial 2 Rx flag **************************************************
uint8_t                 CPUIntTickEn = 0;           // IRQ enable for the Systick timer ************************************************
uint8_t                 TickIRQstatus = 0;          // Status (active/not active) of the Systick IRQ          ******************************
uint8_t                 sysTickTime = 5;            // Period in milliseconds of the Systick interrupt (if enabled) ********************
uint8_t                 LiteConfFlg =   0;          // Flag set to 1 if Lite configuration is detected (0 = Full))
uint8_t                 RAM_EN_Enabled  = 1;        // Default value to set *active* the RAM_EN signal
uint8_t                 RAM_EN_Disabled  = 0;       // Default value to set *not active* the RAM_EN signal
uint8_t                 bootSelFlag = 0;            // Flag to enter into the boot mode selection menu [0..1]
uint8_t                 bootMode       = 0;         // Set the program to load at boot (from flash or SD) [0..maxBootMode]
uint8_t                 maxBootMode   = 3;          // Maximum allowed value for bootMode
char                    maxSelChar    = '7';        // Maximum allowed ASCII value (boot mode selection menu)
uint8_t                 iByte;                      // Temporary variable
uint8_t                 jByte;                      // Temporary variable
uint8_t                 zByte;                      // Temporary variable
uint8_t                 ser1Speed;                  // Serial 1 speed [0..6] (0 = 300, 1 = 1200, 2 =2400, 3 = 9600, 4 = 19200,
                                                    // 5 = 57600, 6 = 115200)
uint8_t                 ser2Speed;                  // Serial 2 speed [0..6] (0 = 300, 1 = 1200, 2 =2400, 3 = 9600, 4 = 19200,
                                                    // 5 = 57600, 6 = 115200)

// SD (HD emulation) variables
FATFS                   filesysSD;                  // Filesystem object (PetitFS library)
uint8_t                 bufferSD[512];              // I/O buffer for SD disk operations (store a 512 bytes sector)
uint8_t                 pendingBuffWrite = 0;       // Flag set at 1 if there is a pending logical sector buffer write operation
                                                    //  (used for the deferred write strategy to optimize the physical write 
                                                    //  operations to SD when the "128 bytes sector emulation" is active)
uint8_t                 readSDrequired;             // Flag set at 1 if a logical sector read from SD is required, 0 otherwise 
                                                    //  (used for sector de-blocking
                                                    //  optimization inside the READSECT Opcode and for pre-read optimization inside the
                                                    //  WRITESECT Opcode)
uint8_t                 bufferNotValid = 1;         // Flag set at 1 if the 512 bytes buffer content is not valid, 0 if valid
const char *            fileNameSD;                 // Pointer to the string with the currently used file name
uint8_t                 errCodeSD;                  // Store error codes from the PetitFS
uint16_t                numReadBytes;               // Number of read bytes after a readSD() call
uint16_t                numWriBytes;                // Number of written bytes after a writeSD() call
uint8_t                 sect128Flag = 1;            // If set (= 1) the "128 bytes sector emulation" is active
uint8_t                 diskSet;                    // Store the current "disk set" [0..99]
uint16_t                trackSel;                   // Store the current track number [0..511]
uint8_t                 sectSel;                    // Store the current sector number [0..(*)]
                                                    // (*) The maximum valid sector number depends on whether the 
                                                    //  "128 bytes sector" emulation is active or not.
                                                    //  If it is active the maximum value is 127, if not it is 31
uint8_t                 sectSel512;                 // Sector number (512 bytes) currently stored into the buffer
uint16_t                logicSect;                  // Current logical sector (512 byte) number
uint16_t                buffSect;                   // Logical sector number of the logical sector currently stored into the buffer
uint16_t                sectBuffIndex;              // Index of the current data byte inside the sector buffer
uint8_t                 autoexecFlag;               // Set to 1 if AUTOEXEC must be executed at CP/M cold boot, 0 otherwise
char                    OsName[11] = DS_OSNAME;     // String used for file holding the OS name
char                    diskName[11] = EMUDISK;     // String used for virtual disk file name
uint8_t                 diskErr = 0;                // SELDISK, SELSECT, SELTRACK, WRITESECT, READSECT, FLUSHBUFF or SDMOUNT
                                                    //  resulting error code
char *                  pchar;                      // Used for the Autoboot boot mode
uint32_t                sectorLBA = 0;              // Sector number as LBA ********************************************************
uint8_t                 diskErrLBA = 0;             // SELLBA, WRITELBA, READLBA resulting error code ******************************

// GPIO expansion module (MCP23017) variables
uint8_t                 moduleGPIO     = 0;         // Set to 1 if the module is found, 0 otherwise
uint8_t                 SPPmode        = 0;         // Set to 1 if the GPIO port is used as a standard SPP (SPP Adapter)
uint8_t                 SPPautofd      = 0;         // Store the status of the AUTOFD Control Line (active Low) of the SPP

// DS3231 RTC variables
uint8_t                 foundRTC;                   // Set to 1 if RTC is found, 0 otherwise
uint8_t                 seconds, minutes, hours, day, month, year;
uint8_t                 tempC;

// CPU bootload variables
uint32_t                execStarAddr;               // Boot program execution starting address
uint32_t                loadStarAddr;               // Boot program loading address
uint16_t                iCount;                     // Temporary variable (counter)
uint32_t                iLong;                      // Temporary variable (counter)
uint8_t                 emptyFile = 1;              // Set to 1 for an empty file
const uint8_t *         BootImage;                  // Pointer to selected payload array (image) to boot (from flash)
uint32_t                BootImageSize  = 0;         // Size of the selected payload array (image) to boot (from flash)
uint32_t                BootStrAddr;                // Starting address of the selected program to boot (from flash or SD)


// ------------------------------------------------------------------------------
// ------------------------------------------------------------------------------


void main(void)
{
  // Initialize the device
  SYSTEM_Initialize();

  // Set TMR0 for millis())
  millisCounter = 0;                                      // Clear milliseconds counter
  TMR0_SetInterruptHandler(Timer0_ISR);                   // Set the TMR0 interrupt ISR

  // Enable the Global Interrupts
  INTERRUPT_GlobalInterruptEnable();

  // Enable the Peripheral Interrupts
  INTERRUPT_PeripheralInterruptEnable();
   
  // Set Slew Rate Control registers
  SLRCONA = 0b11001110;                                   // Set Slew Rate High (bit = 0) for: RAM_EN_, DTACK_, SS_ (RA0, RA4, RA5)
  SLRCONC = 0b11001111;                                   // Set Slew Rate High (bit = 0) for: SCK, MOSI (RC4, RC5)
  SLRCOND = 0x00;                                         // Set Slew Rate High (bit = 0) for D0-D7 (RD0-7)
  SLRCONE = 0b11111101;                                   // Set Slew Rate High (bit = 0) for HALT_ (RE1)
  
  // Check Full/Lite HW configuration and set the MCU accordingly
  LiteConfFlg = CheckLiteConfig();
  if (LiteConfFlg)
  {
    SetNegatedIO_M();                                     // Set the IO_/M signal to work negated (IO/M_)) on CLC1
    RAM_EN_Enabled = 0;                                   // Set RAM_EN to work negated (RAM_EN_)
    RAM_EN_Disabled = 1;
  }
  
  // Reset uTerm (A071218-R250119) if present
  MCU_RTS_SetLow();                                       // Reset uTerm (A071218-R250119)
  __delay_ms(100); 
  MCU_RTS_SetHigh();
  __delay_ms(500);                                        // Give to uTerm the time to exit from reset
    
  // DEBUG
  #if DEBUG_GPIOPAR > 0 
    printf("\r\n\n* DEBUG: GPIOPAR * INLVLA  = 0x%02X - SLRCONA  = 0x%02X\r\n", INLVLA, SLRCONA);
    printf("* DEBUG: GPIOPAR * INLVLC  = 0x%02X - SLRCONC  = 0x%02X\r\n", INLVLC, SLRCONC);
    printf("* DEBUG: GPIOPAR * INLVLD  = 0x%02X - SLRCOND  = 0x%02X\r\n", INLVLD, SLRCOND);
    printf("* DEBUG: GPIOPAR * INLVLE  = 0x%02X - SLRCONE  = 0x%02X\r\n", INLVLE, SLRCONE);
  #endif
  
  // Check USER Key for boot mode changes 
  if (!USER_KEY_GetValue()) bootSelFlag = 1;
  
  // Read the stored Disk Set. If not valid set it to 0
  diskSet = DATAEE_ReadByte(diskSetAddr);
  if (diskSet >= maxDiskSet) 
  {
    DATAEE_Update(diskSetAddr, 0);
    diskSet =0;
  }
  
  // Read the stored serial 1 and 2 speed and set both the SPxBRG registers. If not valid set it to 115200 baud.
  ser1Speed = DATAEE_ReadByte(Ser1Addr);                  // Serial 1
  if (ser1Speed > 6) 
  {
    ser1Speed = 6;
    DATAEE_Update(Ser1Addr, ser1Speed);
  }
  ser2Speed = DATAEE_ReadByte(Ser2Addr);                  // Serial 2
  if (ser2Speed > 6) 
  {
    ser2Speed = 6;
    DATAEE_Update(Ser2Addr, ser2Speed);
  }
  SP1BRGH = HIGH_BYTE(calcSPxBRG(ser1Speed));             // Set EUSART1 and EUSART2 baud rates (SPxBRG)
  SP1BRGL = LOW_BYTE(calcSPxBRG(ser1Speed));
  SP2BRGH = HIGH_BYTE(calcSPxBRG(ser2Speed));
  SP2BRGL = LOW_BYTE(calcSPxBRG(ser2Speed));
    
  // Print basic system info
  printf("\r\n\n68k-MBC - A091020-R140221\r\nIOS - I/O Subsystem - S310121-R091121_DEVEL4\r\n\n");
  if (LiteConfFlg) printf("IOS: Lite HW configuration detected\n\r");
  else printf("IOS: Full HW configuration detected\n\r");
  
  // CPU power-on reset
  __delay_ms(200);                                        // Let the CPU complete the power-on reset
  singleStepDrive();
  
  // Print RTC and GPIO informations if found
  foundRTC = autoSetRTC();                                // Check if RTC is present and initialize it as needed
  if (I2C_check(GPIOEXP_ADDR) == 0) moduleGPIO = 1;       // Check if the GPIO expander (GPE) is present
  if (moduleGPIO) printf("IOS: Found GPE Option\r\n");
  
  // Print CP/M Autoexec flag status
  printf("IOS: CP/M Autoexec is ");
  if (DATAEE_ReadByte(autoexecFlagAddr) > 1) DATAEE_Update(autoexecFlagAddr, 0); // Reset AUTOEXEC flag to OFF if invalid
  autoexecFlag = DATAEE_ReadByte(autoexecFlagAddr);       // Read the previously stored AUTOEXEC flag
  if (autoexecFlag) printf("ON\r\n");
  else printf("OFF\r\n");
    
  // ----------------------------------------
  // BOOT SELECTION AND SYS PARAMETERS MENU
  // ----------------------------------------

  // Boot selection and system parameters menu if requested
  mountSD(&filesysSD); mountSD(&filesysSD);               // Try to mount the SD volume
  bootMode = DATAEE_ReadByte(bootModeAddr);               // Read the previous stored boot mode
  if ((bootSelFlag == 1 ) || (bootMode > maxBootMode))
  // Enter in the boot selection menu if USER key was pressed at startup 
  // or an invalid bootMode code was read from the internal EEPROM
  {
    while (eusart1RxCount > 0) EUSART1_Read();            // Flush input serial Rx buffer
    printf("\r\nIOS: Select boot mode or system parameters:\r\n");
    if (bootMode > maxBootMode)
    // Previous invalid boot mode. Set is to 0.
    // This typically happens after a flash operation, when all the EEPROM is erased (set to 0xff).
    {
      DATAEE_Update(bootModeAddr, 0);
      bootMode = 0;
    }
    printf(" 0: No change (");
    printf("%u)\r\n", bootMode + 1);
    printf(" 1: sLoad\r\n");
    printf(" 2: Enhanced 68K Basic\r\n");
    printf(" 3: Autoboot\r\n");
    printf(" 4: Load OS from ");
    printOsName(diskSet);
    printf("\r\n 5: Change ");
    printOsName(diskSet);
    printf("\r\n");
    printf(" 6: Change serial ports speed\r\n");
    printf(" 7: Change CP/M Autoexec (->");
    if (!autoexecFlag) printf("ON");
    else printf("OFF");
    printf(")\r\n");

    // If RTC module is present add a menu choice for it
    if (foundRTC)
    {
      printf(" 8: Change RTC time/date\r\n");
      maxSelChar = '8';
    }

    // Ask a choice
    printf("\r\n");
    printf("Enter your choice >");
    do
    {
      WaitAndBlink(SetDefaultBaud);                       // Wait a key while User led blinks and check the 
                                                          //  "User key long pressed" event; in that case set serial 1 and 2 speed
                                                          //  at the default value
      inChar = EUSART1_Read();
    }               
    while ((inChar < '0') || (inChar > maxSelChar));
    printf("%c  Ok\r\n", inChar);

    // Make further actions for the previous choice (when it is needed)
    switch (inChar)
    {
      case '5':                                           // Change current Disk Set
        printMsg1();
        iByte = diskSet;
        do
        {
          // Print the OS name of the next Disk Set
          iByte = (iByte + 1) % maxDiskSet;
          printf("\r ->");
          printOsName(iByte);
          printf("                 \r");
          while (eusart1RxCount > 0) EUSART1_Read();      // Flush serial Rx buffer
          WaitAndBlink(NULL);                             // Wait a key while User led blinks
          inChar = EUSART1_Read();
        }
        while ((inChar != 13) && (inChar != 27));         // Continue until a CR or ESC is pressed
        printf("\r\n\n");
        if (inChar == 13)                                 // Set and store the new Disk Set if required
        {
          diskSet = iByte;
          DATAEE_Update(diskSetAddr, iByte);
          inChar = '4';                                   // Then select the option '4' too ********************************************
        }
      break;
      
      case '6':                                           // Change serial ports speed
        zByte = 0;                                        // Clear the "changed speed" counter
        for (jByte = 1; jByte < 3; jByte++)
        {
          printMsg1();
          if (jByte == 1)
            {
              iByte = DATAEE_ReadByte(Ser1Addr);          // Read current serial 1 speed
            }
            else
            {
              iByte = DATAEE_ReadByte(Ser2Addr);          // Read current serial 2 speed
            }
          iByte--;
          do
          {
            iByte++;
            if (iByte > 6) iByte = 0;
            printf("\r                   ");
            printf("\r ->Serial %u: ", jByte);
            switch (iByte)
            {
              case 0:
                printf("300");
              break;
              
              case 1:
                printf("1200");
              break;
               
              case 2:
                printf("2400"); 
              break;
              
              case 3:
                printf("9600");
              break;
            
              case 4:
                printf("19200");
              break;
            
              case 5:
                printf("57600");
              break;
            
              case 6:
                printf("115200");
              break;
            }
            while (eusart1RxCount > 0) EUSART1_Read();    // Flush serial Rx buffer
            WaitAndBlink(NULL);                           // Wait a key while User led blinks
            inChar = EUSART1_Read();
          }
          while ((inChar != 13) && (inChar != 27));       // Continue until a CR or ESC is pressed
          if (inChar == 13)
          // Store new serial 1 and 2 speeds (if needed))
          {
            printf("  Ok");
            if (jByte == 1)
            {
              zByte = zByte + DATAEE_Update(Ser1Addr, iByte); // Update serial 1 speed
            }
            else
            {
              zByte = zByte + DATAEE_Update(Ser2Addr, iByte); // Update serial 2 speed
            }
          }
          printf("\n");
        }
        printf("\r\n");
        if (zByte > 0) printf("Changed speeds will be effective after next reboot!\r\n\n");
        break;
      
      case '7':                                           // Toggle CP/M AUTOEXEC.SUB execution on cold boot
        autoexecFlag = !autoexecFlag;                     // Set new value
        DATAEE_Update(autoexecFlagAddr, autoexecFlag);    // Save new value to the internal EEPROM
      break;

      case '8':                                           // Change RTC Date/Time
        ChangeRTC();
      break;
    };
    USER_LED_SetLow();
    
    // Save boot program if changed
    bootMode = inChar - '1';                              // Calculate the new bootMode from inChar. Note that bootMode store only
                                                          //  a value to select the program to load and boot. Value 0 corresponds
                                                          //  to the menu choice '1' and so on. Max allowed value is "maxBootMode"
                                                          //  that corresponds to the highest possible program boot selection.
                                                          //  Others (higher) menu selections are ignored because are system 
                                                          //  parameters setting.
    if (bootMode <= maxBootMode)
    // Save to the internal EEPROM if really required
    {
      DATAEE_Update(bootModeAddr, bootMode);
    } 
    else bootMode = DATAEE_ReadByte(bootModeAddr);        // Reload boot mode if it was selected 0 or > (maxBootMode+1) (no change or 
                                                          // system parameter)
  }

  // Print current Disk Set and OS name (if OS boot is enabled)
  if (bootMode == 3)
  {
    printf("IOS: Current ");
    printOsName(diskSet);
    printf("\r\n");
  }
   
  // ----------------------------------------
  // BOOT LOAD INITIALIZATION
  // ----------------------------------------

  // Set the loading starting address and the execution starting address of the binary executable to be loaded at boot, and its size if 
  // stored in the flash.
  
  switch (bootMode)
  {
    case 0:                                               // Load sLoad from flash
      BootImage = image_A_;                               // Set the image A
      BootImageSize = sizeof(image_A_);                   // Set the size
      loadStarAddr = image_A_StrAddr;                     // Set the loading starting address
      execStarAddr = image_A_StrAddr;                     // Set the execution (after loading) starting address
      break;
    
    case 1:                                               // Load the Enhanced 68K Basic from SD
      fileNameSD = EHBASFN;
      loadStarAddr = EHBASLDADR;
      execStarAddr = EHBASSTADR;
    break;
        
    case 2:                                               // Load AUTOBOOT.BIN from SD (load an user executable binary file)
           
      // Set the loading parameters to use
      loadStarAddr = AUTLOADADR;                          // Set default value (loading address)
      execStarAddr = AUTSTARADR;                          // Set default value (execution address)
      fileNameSD = AUTOADR;                               // Set the file name for the optional custom loading parameters
      if (openSD(AUTOADR) == 0)
      // The optional file <AUTOADR> is present, so read the custom parameters for loading and executing
      {
        readSD(bufferSD, &numReadBytes);                   // Read the parameters line
        if (numReadBytes > 6)                              // Expected at least 7 ASCII chars string ("0x0 0x0")
        // Read string seems valid...
        {
          loadStarAddr = (uint32_t) strtol((char *) bufferSD, &pchar, 16); // Get the first parameter (loading address)
          execStarAddr = (uint32_t) strtol(pchar, NULL, 16);      // Get the second parameter (execution address)
        }
        
        #if DEBUG_AUTOBIN > 0
          else printf("\n\r* DEBUG: AUTOBOOT - String too short (%u chars). Using default values\n\n\r", numReadBytes);
        #endif

      }
        
      #if DEBUG_AUTOBIN > 0
        else printf("\n\r* DEBUG: AUTOBOOT - No file. Using default values\n\n\r");
      #endif

      printf("IOS: Autoboot load 0x%05lx - exec 0x%05lx \n\r", loadStarAddr, execStarAddr);
      fileNameSD = AUTOFN;                                // Set the file name for loading
    break;

    case 3:                                               // Load an OS from current Disk Set on SD
      switch (diskSet)
      {
        case 0:                                           // CP/M-68K
          fileNameSD = CPM68FN;
          loadStarAddr = CPM68LDADR;
          execStarAddr = CPM68STADR;
        break;
        
        // ******************************************************************************************************************* NEW
        case 1:                                           // EmuTOS
          fileNameSD = EMUTOSADR;                         // Set the loading parameters file name
          openFileSD(fileNameSD);
          readSD(bufferSD, &numReadBytes);                // Read the parameters line
          loadStarAddr = (uint32_t) strtol((char *) bufferSD, &pchar, 16);  // Get the first parameter (loading address)
          execStarAddr = (uint32_t) strtol(pchar, NULL, 16);                // Get the second parameter (execution address)
          printf("IOS: EmuTOS load 0x%05lx - exec 0x%05lx \n\r", loadStarAddr, execStarAddr);
          fileNameSD = EMUTOSFN;                          // Set the file name to load
        break;
        // ******************************************************************************************************************* END
        
      }
    break;
  }
        
  // ----------------------------------------
  // 68008 BOOT LOAD EXECUTION
  // ----------------------------------------
  
  // Execute the boot load using a binary executable image stored in flash memory or SD
  
  // Print the message of the boot load
  printf("IOS: Loading boot program");
  if (bootMode > 0)
  // Load from SD, so print the binary file name
  {
    printf(" (%s)", fileNameSD);
    openFileSD(fileNameSD);
  }
  printf("...");
  
  // DEBUG
  #if DEBUG_L > 0 
    printf("\n\r");
  #endif
  
  // 68008 CPU initialization sequence
  feedReadCycle(HIGH_BYTE(HIGH_WORD(RES_SSP_VECT)));      // Feed SSP (please note that this is only to start the "feeding" loading sequence)
  feedReadCycle(LOW_BYTE(HIGH_WORD(RES_SSP_VECT)));
  feedReadCycle(HIGH_BYTE(LOW_WORD(RES_SSP_VECT)));
  feedReadCycle(LOW_BYTE(LOW_WORD(RES_SSP_VECT)));
        
  feedReadCycle(HIGH_BYTE(HIGH_WORD(RES_PC_VECT)));       // Feed PC (please note that this is only to start the "feeding" loading sequence)
  feedReadCycle(LOW_BYTE(HIGH_WORD(RES_PC_VECT)));
  feedReadCycle(HIGH_BYTE(LOW_WORD(RES_PC_VECT)));
  feedReadCycle(LOW_BYTE(LOW_WORD(RES_PC_VECT)));
     
  feedReadCycle(HIGH_BYTE(MOVEA_L_A0_OPC));               // Execute A0 = 0 (A0 is used as RAM pointer for the loading sequence) for SSP and PC
  feedReadCycle(LOW_BYTE(MOVEA_L_A0_OPC));
  feedReadCycle(0x00);
  feedReadCycle(0x00);
  feedReadCycle(0x00);
  feedReadCycle(0x00);
        
  feedReadCycle(HIGH_BYTE(MOVE_L_A0_INC_OPC));            // Load Reset SSP vector into RAM (this is the true vector)
  feedReadCycle(LOW_BYTE(MOVE_L_A0_INC_OPC));
  feedReadCycle(HIGH_BYTE(HIGH_WORD(RES_SSP_VECT)));
  feedReadCycle(LOW_BYTE(HIGH_WORD(RES_SSP_VECT)));
  feedReadCycle(HIGH_BYTE(LOW_WORD(RES_SSP_VECT)));
  feedReadCycle(LOW_BYTE(LOW_WORD(RES_SSP_VECT)));
        
  feedReadCycle(HIGH_BYTE(MOVE_L_A0_INC_OPC));            // Load Reset PC vector into RAM (this is the true vector)
  feedReadCycle(LOW_BYTE(MOVE_L_A0_INC_OPC));
  skipWriteCycles();
  feedReadCycle(HIGH_BYTE(HIGH_WORD(execStarAddr)));
  feedReadCycle(LOW_BYTE(HIGH_WORD(execStarAddr)));
  feedReadCycle(HIGH_BYTE(LOW_WORD(execStarAddr)));
  feedReadCycle(LOW_BYTE(LOW_WORD(execStarAddr)));
    
  feedReadCycle(HIGH_BYTE(MOVEA_L_A0_OPC));               // Execute A0 = loadStarAddr (A0 is used as RAM pointer for the loading sequence)
  feedReadCycle(LOW_BYTE(MOVEA_L_A0_OPC));                // ATTENTION: if <loadStarAddr> = 0x00000 the previous stored vectors (SSP and PC)
  skipWriteCycles();                                      //            will be overwritten by the first 8 bytes of the loaded file.
  feedReadCycle(HIGH_BYTE(HIGH_WORD(loadStarAddr)));      //            This can be used as a way to change these vectors at loading time.
  feedReadCycle(LOW_BYTE(HIGH_WORD(loadStarAddr)));
  feedReadCycle(HIGH_BYTE(LOW_WORD(loadStarAddr)));
  feedReadCycle(LOW_BYTE(LOW_WORD(loadStarAddr)));
    
  // Load the boot executable into the RAM (starting from the address in A0) from SD or flash
  if (bootMode > 0)
  // Load from SD
  {
    do
    // If an error occurs repeat until error disappears (or the user forces a reset)
    {
      do
      // Read a sector of the SD and load it into RAM
      {
        errCodeSD = readSD(bufferSD, &numReadBytes);      // Read current sector of the SD
        if (numReadBytes > 0) emptyFile = 0;              // Check for an empty file
        for (iCount = 0; iCount < numReadBytes; iCount = iCount + 4)
        // Load the read sector into RAM (4 bytes each iteration)
        //
        // NOTE: after the load of the last sector of the binary file, up to 3 "garbage" bytes can be appended into RAM
        //       if the total number of the bytes to load is not an exact multiple of 4.
        {
         
          // DEBUG
          #if DEBUG_L > 0 
          printf("* DEBUG: iCount(4x) = %u\n\r", iCount);
          #endif

          // Load 4 bytes into RAM (A0 is used as RAM pointer for the loading sequence)
          feedReadCycle(HIGH_BYTE(MOVE_L_A0_INC_OPC));
          feedReadCycle(LOW_BYTE(MOVE_L_A0_INC_OPC));
          skipWriteCycles();
          feedReadCycle(bufferSD[iCount]);
          feedReadCycle(bufferSD[iCount + 1]);
          feedReadCycle(bufferSD[iCount + 2]);
          feedReadCycle(bufferSD[iCount + 3]);
        }
      }
      while ((numReadBytes == 512) && (!errCodeSD));      // If numReadBytes < 512 -> EOF reached
      if (errCodeSD)
      {
        printErrSD(2, errCodeSD, fileNameSD);
        waitKey();                                        // Wait a key to repeat
        seekSD(0);                                        // Reset the sector pointer
      }
    }
    while (errCodeSD);
    if (emptyFile)
    // Empty file error
    {
      printf("\r\n\nIOS: Empty file - Load aborted!");
      while (1);
    }
  }
  else
  // Load from flash
  {
    
      // DEBUG
      #if DEBUG_L > 0 
      printf("* DEBUG: BootImageSize = %u bytes\n\r", BootImageSize);
      #endif

    for (iLong = 0; iLong < BootImageSize; iLong++)
    // Load into RAM (1 byte each iteration)
    {
      // DEBUG
      #if DEBUG_L > 0 
      printf("* DEBUG: iLong(1x) = %u\n\r", iLong);
      #endif

      // Load one byte into RAM (A0 is used as RAM pointer for the loading sequence)
      feedReadCycle(HIGH_BYTE(MOVE_B_A0_INC_OPC));
      feedReadCycle(LOW_BYTE(MOVE_B_A0_INC_OPC));
      skipWriteCycles();
      feedReadCycle(0x00);
      feedReadCycle(BootImage[iLong]);
    }
  }
  
  // Append a STOP instruction
  feedReadCycle(HIGH_BYTE(STOP_OPC));
  feedReadCycle(LOW_BYTE(STOP_OPC));
  skipWriteCycles();
  feedReadCycle(HIGH_BYTE(STOP_OPC_DATA));
  feedReadCycle(LOW_BYTE(STOP_OPC_DATA));
  printf(" done\n\r");
    
  // Reset the 68008 CPU and let it run
  HALT__LAT = 0;                                          // Reset the CPU
  RESET__LAT = 0;
  
  Set_S_STEP__High;                                       // Set S-STEP not active
  __delay_ms(1);                                          // Let the CPU complete the reset (just to be sure...)
    
  HALT__LAT = 1;                                          // Resume the CPU from reset
  RESET__LAT = 1;
  printf("IOS: 68008 CPU is running from now\n\n\r");
 
  while (1)
  // Virtual Engine loop
  {
    if (DTACK_ == 1)
    // I/O operation requested
    {
      if (R_W__GetValue() == 0)
      // I/O WRITE bus operation requested
      //
      // ----------------------------------------
      // VIRTUAL I/O WRITE OPERATIONS ENGINE
      // ----------------------------------------
      //
      {
        // Read D7-D0 and A1-A0
        ioAddress = (uint8_t) (A1_GetValue() << 1) + A0_GetValue();
        ioData = PORTD;
                
        // DEBUG
        #if DEBUG_VW > 0 
          printf("* DEBUG: WRITE I/O * ioData = 0x%02X - ioAddress = 0x%02X\n\r", ioData, ioAddress);
        #endif

        // Execute a write operation
        switch (ioAddress)
        {
          //
          // NOTE: The I/O addressees seen by the CPU are equal to IOBASE + (AD1-AD0), where IOBASE = 0xFFFFC
          //
          case 0:
            // .........................................................................................................
            //
            // AD1-AD0 = 0x0 (I/O write address = 0xFFFFC): EXECUTE WRITE OPCODE.
            //
            // Execute the previously stored I/O write opcode with the current data.
            // The code of the I/O write operation (Opcode) must be previously stored with a STORE OPCODE operation.
            // .........................................................................................................
            //
                    
            execWriteOpcode();
          break;

          case 1:
            // .........................................................................................................
            //
            // AD1-AD0 = 0x1 (I/O write address = 0xFFFFD): STORE OPCODE.
            //
            // Store (write) an "I/O operation code" (Opcode) and reset the exchanged bytes counter.
            //
            // NOTE 1: An Opcode can be a write or read Opcode, if the I/O operation is read or write.
            // NOTE 2: the STORE OPCODE operation must always precede an EXECUTE WRITE OPCODE or EXECUTE READ OPCODE 
            //         operation.
            // NOTE 3: For multi-byte read opcode (as DATETIME) read sequentially all the data bytes without to send
            //         a STORE OPCODE operation before each data byte after the first one.
            // .........................................................................................................
            //
            // .........................................................................................................
            //
            // Currently defined Opcodes for I/O write operations:
            //
            //   Opcode     Name            Exchanged bytes
            // -------------------------------------------------
            // Opcode 0x00  USER LED        1
            // Opcode 0x01  SERIAL 1 TX     1
            // Opcode 0x02  SETIRQ          1
            // Opcode 0x03  GPIOA Write     1
            // Opcode 0x04  GPIOB Write     1
            // Opcode 0x05  IODIRA Write    1
            // Opcode 0x06  IODIRB Write    1
            // Opcode 0x07  GPPUA Write     1
            // Opcode 0x08  GPPUB Write     1
            // Opcode 0x09  SELDISK         1
            // Opcode 0x0A  SELTRACK        2
            // Opcode 0x0B  SELSECT         1  
            // Opcode 0x0C  WRITESECT       128/512
            // Opcode 0x0F  SETTICK         1 ******************************************************************************************
            // Opcode 0x10  SERIAL 2 TX     1
            // Opcode 0x11  SETSPP          1
            // Opcode 0x12  WRSPP           1
            // Opcode 0x13  SELLBA          4 ******************************************************************************************
            // Opcode 0x14  WRITELBA        512 **************************************************************************************** 
            // Opcode 0xFF  No operation    1
            //
            //
            // Currently defined Opcodes for I/O read operations:
            //
            //   Opcode     Name            Exchanged bytes
            // -------------------------------------------------
            // Opcode 0x80  USER KEY        1
            // Opcode 0x81  GPIOA Read      1
            // Opcode 0x82  GPIOB Read      1
            // Opcode 0x83  GETSPP          1
            // Opcode 0x84  DATETIME        7
            // Opcode 0x85  ERRDISK         1
            // Opcode 0x86  READSECT        128/512
            // Opcode 0x87  SDMOUNT         1
            // Opcode 0x88  FLUSHBUFF       1 
            // Opcode 0x89  READLBA         512 ****************************************************************************************
            // Opcode 0x90  ERRLBA          1 ****************************************************************************************
            // Opcode 0xFF  No operation    1
            //
            // See details in the various Opcodes implementation.
            //
                    
            ioOpcode = ioData;                            // Store the I/O operation code (Opcode)
            ioByteCnt = 0;                                // Reset the exchanged bytes counter
          break;
                  
          case 2:

            // NOT USED - RESERVED
                  
          break;

          case 3:

            // NOT USED - RESERVED
                  
          break;
        }
                
        // Control bus sequence to resume the CPU (I/O bus write cycle)
        // !!!Time critical section!!!
        INTERRUPT_GlobalInterruptDisable();               // !!! Start of a time critical section. No interrupt allowed
        Set_DT_EN__High;                                  // !!! Set DT_EN_ not active to let the CPU exit from the wait state and 
                                                          // !!!  complete the bus cycle
        NOP();                                            // !!! Wait 3 * 62.5ns
        NOP();                                            // !!!
        NOP();                                            // !!!
        Set_DT_EN__Low;                                   // !!! Enable DT_EN_ again to request a wait state on (before) the next bus cycle 
                                                          // !!!  if needed
        INTERRUPT_GlobalInterruptEnable();                // !!! End of a time critical section. Interrupt resumed
      }
      else
      {
        if (INTA_GetValue()==0)
        // I/O READ bus operation requested
        //
        // ----------------------------------------
        // VIRTUAL I/O READ OPERATIONS ENGINE
        // ---------------------------------------- 
        //  
        {
          ioAddress = (uint8_t) (A1_GetValue() << 1) + A0_GetValue(); // Read A1-A0
          ioData = 0;                                     // Clear input data buffer
          switch (ioAddress)
          {
            //
            // NOTE: The I/O addressees seen by the CPU are equal to IOBASE + (AD1-AD0), where IOBASE = 0xFFFFC
            //
            case 0:
              // .........................................................................................................
              //
              // AD1-AD0 = 0x0 (I/O read address = 0xFFFFC): EXECUTE READ OPCODE.
              //
              // Execute the previously stored I/O read operation with the current data.
              // The code of the I/O operation (Opcode) must be previously stored with a STORE OPCODE operation.
              //
              // NOTE: For multi-byte read opcode (as DATETIME) read sequentially all the data bytes without to send
              //       a STORE OPCODE operation before each data byte after the first one.
              // .........................................................................................................
              //
            
              execReadOpcode();
            break;
          
            case 1:
              // .........................................................................................................
              //
              // AD1-AD0 = 0x1 (I/O read address = 0xFFFFD): SERIAL 1 RX.
              //
              // Execute a Serial I/O Read operation on serial port 1.
              // .........................................................................................................
              //
              //
              // SERIAL 1 RX:
              //
              //                I/O DATA:    D7 D6 D5 D4 D3 D2 D1 D0
              //                            ---------------------------------------------------------
              //                             D7 D6 D5 D4 D3 D2 D1 D0    ASCII char read from serial
              //
              // NOTE 1: If there is no input char, a value 0xFF is forced as input char.
              // NOTE 2: The IPL1 signal is always reset (set to LOW) after this I/O operation.
              // NOTE 3: This I/O do not require any previous STORE OPCODE operation.
              //
            
              ioData = 0xFF;
              if (eusart1RxCount > 0)
              {
                ioData = EUSART1_Read();
              }
              IPL1__SetHigh();                            // IPL1 = HIGH: Reset the IRQ signal (if used)
            break;

            case 2:
              // .........................................................................................................
              //
              // AD1-AD0 = 0x2 (I/O read address = 0xFFFFE): SYSFLAGS.
              //
              // Read various system flags from the SYSFLAGS register.
              // .........................................................................................................
              //
              //
              // SYSFLAGS (Various system flags):
              //
              //                I/O DATA:    D7 D6 D5 D4 D3 D2 D1 D0
              //                            ---------------------------------------------------------
              //                              X  X  X  X  X  X  X  0    AUTOEXEC not enabled
              //                              X  X  X  X  X  X  X  1    AUTOEXEC enabled
              //                              X  X  X  X  X  X  0  X    DS3231 RTC not found
              //                              X  X  X  X  X  X  1  X    DS3231 RTC found
              //                              X  X  X  X  X  0  X  X    Serial 1 RX buffer empty
              //                              X  X  X  X  X  1  X  X    Serial 1 RX char available
              //                              X  X  X  X  0  X  X  X    Full HW configuration detected
              //                              X  X  X  X  1  X  X  X    Lite HW configuration detected
              //                              X  X  X  0  X  X  X  X    "128 bytes sector emulation" off
              //                              X  X  X  1  X  X  X  X    "128 bytes sector emulation" on
              //                              X  X  0  X  X  X  X  X    Serial 2 RX buffer empty
              //                              X  X  1  X  X  X  X  X    Serial 2 RX char available
              //
              // NOTE 1: Currently only D0-D5 are used.
              // NOTE 2: This I/O do not require any previous STORE OPCODE operation.
              //
              
              ioData = (uint8_t) (autoexecFlag | (foundRTC << 1) | ((eusart1RxCount > 0) << 2) | (LiteConfFlg << 3) | 
                       (sect128Flag << 4) | ((eusart2RxCount > 0) << 5));
            break;

            case 3:
              // .........................................................................................................
              //
              // AD1-AD0 = 0x3 (I/O read address = 0xFFFFF): SERIAL 2 RX.
              //
              // Execute a Serial I/O Read operation on serial port 2.
              // .........................................................................................................
              //
              //
              // SERIAL 2 RX:
              //
              //                I/O DATA:    D7 D6 D5 D4 D3 D2 D1 D0
              //                            ---------------------------------------------------------
              //                             D7 D6 D5 D4 D3 D2 D1 D0    ASCII char read from serial
              //
              // NOTE 1: If there is no input char, a value 0xFF is forced as input char.
              // NOTE 2: This I/O do not require any previous STORE OPCODE operation.
              // NOTE 3: Interrupt is not currently supported on serial 2.
              //
            
              ioData = 0xFF;
              if (eusart2RxCount > 0)
              {
                ioData = EUSART2_Read();
              }
              IPL1__SetHigh();                            // IPL1 = HIGH: Reset the IRQ signal (if used) *********************************
            break;
          }
                    
          // DEBUG
          #if DEBUG_VR > 0 
            printf("* DEBUG: READ I/O * ioData = 0x%02X - ioAddress = 0x%02X\n\r", ioData, ioAddress);
          #endif
                    
          // Put current output on data bus
          HALT__LAT = 0;                                  // Set HALT_ active
          RAM_EN_LAT = RAM_EN_Disabled;                   // Disable RAM
          D0_7_DIR = 0x00  ;                              // Set D0-D7 data bus as output
          LATD = ioData;                                  // Output the output byte on the data bus
          Set_DT_EN__High;                                // Let the 68008 CPU exit from the wait state and complete the current read bus cycle
          NOP();                                          // Wait (at lest 6 * 62.5ns) for CPU in halt state 
          NOP();
          NOP(); 
          NOP(); 
          NOP(); 
          NOP();
    
          // Now the CPU is in bus halt state.
          // Bus control sequence to exit from halt and resume the CPU
          D0_7_DIR = 0xFF ;                               // Set D0-D7 data bus as input (pullup are already active)
          RAM_EN_LAT = RAM_EN_Enabled;                    // Enable RAM again
          Set_DT_EN__Low;                                 // Now it is possible re-enable the wait state request for next bus cycle
          HALT__LAT = 1;                                  // Disable HALT_ signal and let the CPU go to next bus cycle
        }
        else
        // I/O INTERRUPT ACKNOLEDGE bus operation requested
        //
        // -------------------------------------------
        // VIRTUAL I/O INTERRUPT ACK OPERATIONS ENGINE
        // -------------------------------------------
        //
        {
          //********************************************************************************************************************* CHANGED
          
          //ioData = 0x40;                                  // Currently it is fixed to trigger the User Vector 0x40 ************************
          
          // Compute the output on the data bus (IRQ vector)
          // NOTE: the order inside the nested "if" defines the IRQ priority.
          if (TickIRQstatus) 
          // Systick timer IRQ
          {
            ioData = SYSTICKIRQ_VECTOR;                   // Set IRQ vector
            TickIRQstatus = 0;                            // Reset the Systick IRQ status to not active
            IPL1__SetHigh();                              // Reset the IRQ signal (IPL1 = HIGH)
          }
          else
            if ((eusart1RxCount > 0) && CPUIntRx1En)
            // Serial 1 Rx IRQ
            {
              ioData = SER1RXIRQ_VECTOR;                  // Set IRQ vector
            }
            else
              if ((eusart2RxCount > 0) && CPUIntRx2En)
              // Serial 2 Rx IRQ
              {
                ioData = SER2RXIRQ_VECTOR;                // Set IRQ vector
              }
              else
              // Unknown IRQ event! The cause of the generation of the IRQ ACK bus operation is unknown, so the related ISR
              //  should display a fatal error as "Unknown IRQ cause" and halt the system. * * This event should never occur * *.
              {
                //ioData = UNKNOWNIRQ_VECTOR;               // Set IRQ vector *********************************************************** ?????
                //IPL1__SetHigh();                          // Reset the IRQ signal (IPL1 = HIGH)
                printf("\n\n\rIOS: Unknown IRQ - Aborted");
                while (1);
              }

          // DEBUG
          #if DEBUG_VI > 0 
            printf("* DEBUG: IRQ ACK I/O * ioData = 0x%02X - ioAddress = 0x%02X\n\r", ioData, ioAddress);
          #endif

          // Put current output on data bus
          HALT__LAT = 0;                                  // Set HALT_ active
          RAM_EN_LAT = RAM_EN_Disabled;                   // Disable RAM
          D0_7_DIR = 0x00  ;                              // Set D0-D7 data bus as output
          LATD = ioData;                                  // Output the output byte on the data bus
          Set_DT_EN__High;                                // Let the 68008 CPU exit from the wait state and complete the current read bus cycle
          NOP();                                          // Wait (at lest 6 * 62.5ns) for CPU in halt state 
          NOP();
          NOP(); 
          NOP(); 
          NOP(); 
          NOP();
    
          // Now the CPU is in bus halt state.
          // Bus control sequence to exit from halt and resume the CPU
          D0_7_DIR = 0xFF ;                               // Set D0-D7 data bus as input (pullup are already active)
          RAM_EN_LAT = RAM_EN_Enabled;                    // Enable RAM again
          Set_DT_EN__Low;                                 // Now it is possible re-enable the wait state request for next bus cycle
          HALT__LAT = 1;                                  // Disable HALT_ signal and let the CPU go to next bus cycle
        }
      }
    }
    
    // DEBUG
    #if DEBUG_UNKIRQ > 0 
      if (!USER_KEY_GetValue() && (CPUIntRx1En | CPUIntRx2En | CPUIntTickEn)) IPL1__SetLow();
    #endif
    
    if ((eusart1RxCount > 0) && CPUIntRx1En) IPL1__SetLow(); // Generate an IPL1 IRQ on Serial 1 Rx (if enabled)
    if ((eusart2RxCount > 0) && CPUIntRx2En) IPL1__SetLow(); // Generate an IPL1 IRQ on Serial 2 Rx (if enabled) ***********************
    if (CPUIntTickEn)
    // Systick IRQ is enabled. Check if the IRQ signal must be activated
    {
      if ((millis() - timeStamp) > sysTickTime)
      // <sysTickTime> milliseconds are elapsed, so a Systick interrupt is required
      {
        IPL1__SetLow();                                   // Generate an IPL1 IRQ
        TickIRQstatus = 1;                                // Set the Systick IRQ status to active
        timeStamp = millis();                             // Update the timestamp
      }
    }
  }
  //******************************************************************************************************************************** CHANGED END
}


// ------------------------------------------------------------------------------
// ------------------------------------------------------------------------------


// ------------------------------------------------------------------------------
//
// TMR0
//
// ------------------------------------------------------------------------------

void Timer0_ISR(void)
// Timer0 ISR (used for millis())
{
  millisCounter++;
}

// ------------------------------------------------------------------------------

uint32_t millis(void)
// Equivalent to Arduino millis()
{
  uint32_t temp;
  
  PIE0bits.TMR0IE = 0;                                    // Disable TMR0 interrupt during counter reading
  temp = millisCounter;
  PIE0bits.TMR0IE = 1;                                    // Re-enable TMR0 interrupt
  return temp;
}

// ------------------------------------------------------------------------------

void WaitAndBlink(void (* LongPressedUserKeyCall)(void))
// Wait a char from serial 1 while User led blinks using a timestamp.
// A function pointer can be passed as argument for a function callback that will be executed if 
// the User key remains pressed at least 3 seconds ("long User key pressed" event).
// If a NULL pointer is passed as argument no action is done on "long User key pressed" event.
{
  uint8_t   UserKeyLongPressed = 1;                       // Flag for the "User key long pressed" event
  uint32_t  timeStamp1, timeStamp2;                       // Timestamps
  
  timeStamp2 = millis();
  while(eusart1RxCount < 1)
  {
    if ((millis() - timeStamp2) < 3000)
    // Check is User key remains pressed for 3s more
    {
      if (USER_KEY_GetValue()) UserKeyLongPressed = 0;
    }
    
    if ((millis() - timeStamp1) > 300)
    // Blink User led
    {
      USER_LED_Toggle();
      timeStamp1 = millis();
    }
    if (((millis() - timeStamp2) > 3000) && (UserKeyLongPressed) && (LongPressedUserKeyCall != NULL) && ((ser1Speed != 6) || (ser2Speed != 6)))
    // Call the given function if the User key was pressed for 3s more and at least one serial port 
    //  has a nor default speed 
    {
      LongPressedUserKeyCall();
    }
  }
}


// ------------------------------------------------------------------------------
//
// Lite/Full HW setup routines
//
// ------------------------------------------------------------------------------


inline uint8_t CheckLiteConfig()
// Check if the 68k-MBC has a Lite configuration (3 IC board, SJ1-3 shorted)
// The 68000 CPU must be held in reset during this check
// Returned values: 1 for Lite configuration, 0 for Full configuration
{
  return IO_M_GetValue();
}

// ------------------------------------------------------------------------------

inline void SetNegatedIO_M()
// Set the signal IO_/M to operate as IO/M_ (negated) inside CLC1
// This is required in the Lite configuration mode
{
  // Set: LC1G1D3N disabled; LC1G1D2N disabled; LC1G1D4N disabled; LC1G1D1T enabled; LC1G1D3T disabled; LC1G1D2T disabled; 
  //      LC1G1D4T disabled; LC1G1D1N disabled; 
  CLC1GLS0 = 0x02;
}


// ------------------------------------------------------------------------------
//
// I2C check routines
//
// CREDITS: See Microchip Forum https://www.microchip.com/forums/FindPost/1118405 
//
// ------------------------------------------------------------------------------


static  i2c2_operations_t  NackAddressHandler(void *Ptr)
// This is a private  callback handler for use with I2C_check(...) function.
//
// Argument: *Ptr   Callback payload pointer is used for address of nackFlag status value.
{    
  // Do something to notice that Address Nack have occurred
	uint8_t * nackFlag = Ptr;
  *nackFlag = 1;
  return I2C2_STOP;                                       // Let driver proceed with Stop signal
}

// ------------------------------------------------------------------------------

uint8_t I2C_check( uint8_t  deviceAddress)
// Function to check if I2C device answer when called.
//
// Argument:
//  deviceAddress is I2C 7-bit address of a slave.
//  Return value == 0   I2C slave is present and responding.
//               == 1   NACK returned, slave is not connected, or do not respond.
{
	static uint8_t   NackFlag; 
	static uint8_t   dummy[2] = {0x00, 0x18};
  NackFlag = 0;                                           // Reset the flag,
  while(!I2C2_Open(deviceAddress));                       // sit here until we get the bus..
  I2C2_SetBuffer(&dummy[1],2);                            // This should work with i2c_master.c with no change
	I2C2_SetAddressNackCallback(NackAddressHandler, &NackFlag); // Pointer to Flag as payload pointer
  I2C2_MasterOperation(0);                                // Start a I2C Write operation, same as I2C_MasterWrite
  while(I2C2_BUSY ==  I2C2_Close());                      // Sit here until finished.
  return NackFlag;
}

// ------------------------------------------------------------------------------


// ------------------------------------------------------------------------------
//
// RTC Module routines
//
// ------------------------------------------------------------------------------


uint8_t decToBcd(uint8_t val)
// Convert a binary byte to a two digits BCD byte
{
  return( (val/10*16) + (val%10) );
}

// ------------------------------------------------------------------------------

uint8_t bcdToDec(uint8_t val)
// Convert binary coded decimal to normal decimal numbers
{
  return( (val/16*10) + (val%16) );
}

// ------------------------------------------------------------------------------

void readRTC(uint8_t *seconds, uint8_t *minutes, uint8_t *hours, uint8_t *day, uint8_t *month, uint8_t *year, uint8_t *tempC)
// Read current date/time binary values and the temperature (2 complement) from the DS3231 RTC
{
  uint8_t    data[18];                                    // Temporary I2C buffer
    
  // Read from RTC and convert to binary
  I2C2_ReadDataBlock(DS3231_RTC, DS3231_SECRG, data, 18); // Read a block of 18 bytes
  *seconds = bcdToDec(data[0] & 0x7f);
  *minutes = bcdToDec(data[1]);
  *hours = bcdToDec(data[2] & 0x3f);
  *day = bcdToDec(data[4]);
  *month = bcdToDec(data[5]);
  *year = bcdToDec(data[6]);
  *tempC = data[17];
}

// ------------------------------------------------------------------------------

void writeRTC(uint8_t seconds, uint8_t minutes, uint8_t hours, uint8_t day, uint8_t month, uint8_t year)
// Write given date/time binary values to the DS3231 RTC
{
  uint8_t    data[8];                                     // Temporary I2C buffer

  // Set the I2C buffer with the needed values to write into the RTC
  data[0] = DS3231_SECRG;                                 // Using the I2C1_WriteNBytes() function the first location of the buffer
                                                          //  array is the register to use
  data[1] = decToBcd(seconds);
  data[2] = decToBcd(minutes);
  data[3] = decToBcd(hours);
  data[4] = 1;                                            // Day of week not used (always set to 1 = Sunday)
  data[5] = decToBcd(day);
  data[6] = decToBcd(month);
  data[7] = decToBcd(year);
  
  // Write new values into the RTC
  I2C2_WriteNBytes(DS3231_RTC, data, 8);
}

// ------------------------------------------------------------------------------

uint8_t autoSetRTC()
// Check if the DS3231 RTC is present and set the date/time at compile date/time if 
// the RTC "Oscillator Stop Flag" is set (= date/time failure).
// Return value: 0 if RTC not present, 1 if found.
{
  uint8_t   OscStopFlag;
  char      str[2];

  if (I2C_check(DS3231_RTC)) return 0;                    // RTC not found
  printf("IOS: Found RTC DS3231 Module (");
  printDateTime(1);
  printf(")\r\n");

  // Print the temperature from the RTC sensor
  printf("IOS: RTC DS3231 temperature sensor: %uC\r\n", tempC);
  
  // Check the "Oscillator Stop Flag"  
  OscStopFlag = I2C2_Read1ByteRegister(DS3231_RTC, DS3231_STATRG) & 0x80; // Read the "Oscillator Stop Flag"
  if (OscStopFlag)
  // RTC oscillator stopped. RTC must be set at compile date/time
  {
    // Convert compile time strings to numeric values
    strncpy(str, compTimeStr+6, 2);                       // Seconds
    seconds = toInt(str);
    strncpy(str, compTimeStr+3, 2);                       // Minutes
    minutes = toInt(str);
    strncpy(str, compTimeStr, 2);                         // Hours
    hours = toInt(str);
    strncpy(str, compDateStr+4, 2);                       // Day
    day = toInt(str);
    switch (compDateStr[0])                               // Month
    {
      case 'J': if (compDateStr[1] == 'a') month = 1;
                else if (compDateStr[2] == 'n') month = 6;
                else month = 7;
                break;
      case 'F': month = 2; break;
      case 'A': month = compDateStr[2] == 'r' ? 4 : 8; break;
      case 'M': month = compDateStr[2] == 'r' ? 3 : 5; break;
      case 'S': month = 9; break;
      case 'O': month = 10; break;
      case 'N': month = 11; break;
      case 'D': month = 12; break;
    }
    strncpy(str, compDateStr+9, 2);                       // Year
    year = toInt(str);
    
    // Ask for RTC setting al compile date/time
    printf("IOS: RTC clock failure!\r\n");
    printf("\nDo you want set RTC at IOS compile time (");
    printDateTime(0);
    printf(")? [Y/N] >");
    do
    {
      inChar = EUSART1_Read();
    }
    while ((inChar != 'y') && (inChar != 'Y') && (inChar != 'n') &&(inChar != 'N'));
    printf("%c\r\n", inChar);
 
    // Set the RTC at the compile date/time and print a message
    if ((inChar == 'y') || (inChar == 'Y'))
    {
      writeRTC(seconds, minutes, hours, day, month, year);
      printf("IOS: RTC set at compile time - Now: ");
      printDateTime(1);
      printf("\r\n");
    }

    // Reset the "Oscillator Stop Flag" (32KHz output left enabled)
    I2C2_Write1ByteRegister(DS3231_RTC, DS3231_STATRG, 0x08);
  }
  return 1;
}

// ------------------------------------------------------------------------------

void printDateTime(uint8_t readSourceFlag)
// Print to serial the current date/time from the global variables.
//
// Flag readSourceFlag [0..1] usage:
//    If readSourceFlag = 0 the RTC read is not done
//    If readSourceFlag = 1 the RTC read is done (global variables are updated)
{
  if (readSourceFlag) readRTC(&seconds, &minutes, &hours, &day,  &month,  &year, &tempC);
  print2digit(day);
  printf("/");
  print2digit(month);
  printf("/");
  print2digit(year);
  printf(" ");
  print2digit(hours);
  printf(":");
  print2digit(minutes);
  printf(":");
  print2digit(seconds);
}

// ------------------------------------------------------------------------------

void print2digit(uint8_t data)
// Print a byte [0..99] using 2 digit with leading zeros if needed
{
  if (data < 10) printf("0");
  printf("%u", data);
}

// ------------------------------------------------------------------------------

uint8_t isLeapYear(uint8_t yearXX)
// Check if the year 2000+XX (where XX is the argument yearXX [00..99]) is a leap year.
// Returns 1 if it is leap, 0 otherwise.
// This function works in the [2000..2099] years range. It should be enough...
{
  if (((2000 + yearXX) % 4) == 0) return 1;
  else return 0;
}

// ------------------------------------------------------------------------------

inline void ChangeRTC()
// Change manually the RTC Date/Time from keyboard
{
  uint8_t    leapYear;                                    // Set to 1 if the selected year is bissextile, 0 otherwise [0..1]
  uint8_t    tempByte;                                    // Temporary variable (buffer)

  // Read RTC
  readRTC(&seconds, &minutes, &hours, &day,  &month,  &year, &tempC);

  // Change RTC date/time from keyboard
  tempByte = 0;
  printf("\r\nIOS: RTC manual setting:\r\n");
  printf("\r\nPress T/U to increment +10/+1 or CR to accept\r\n");
  do
  {
    do
    {
      printf(" ");
      switch (tempByte)
      {
        case 0:
          printf("Year -> ");
          print2digit(year);
        break;
        
        case 1:
          printf("Month -> ");
          print2digit(month);
        break;

        case 2:
          printf("             ");
          //Serial.write(13);
          printf("\r Day -> ");
          print2digit(day);
        break;

        case 3:
          printf("Hours -> ");
          print2digit(hours);
        break;

        case 4:
          printf("Minutes -> ");
          print2digit(minutes);
        break;

        case 5:
          printf("Seconds -> ");
          print2digit(seconds);
        break;
      }
      do
      {
        WaitAndBlink(NULL);                               // Wait a key while User led blinks
        inChar = EUSART1_Read();
      }
      while ((inChar != 'u') && (inChar != 'U') && (inChar != 't') && (inChar != 'T') && (inChar != 13));
      
      if ((inChar == 'u') || (inChar == 'U'))
      // Change units
        switch (tempByte)
        {
          case 0:
            year++;
            if (year > 99) year = 0;
          break;

          case 1:
            month++;
            if (month > 12) month = 1;
          break;

          case 2:
            day++;
            if (month == 2)
            {
              if (day > (daysOfMonth[month - 1] + isLeapYear(year))) day = 1;
            }
            else
            {
              if (day > (daysOfMonth[month - 1])) day = 1;
            }
          break;

          case 3:
            hours++;
            if (hours > 23) hours = 0;
          break;

          case 4:
            minutes++;
            if (minutes > 59) minutes = 0;
          break;

          case 5:
            seconds++;
            if (seconds > 59) seconds = 0;
          break;
        }
      if ((inChar == 't') || (inChar == 'T'))
        
      // Change tens
        switch (tempByte)
        {
          case 0:
            year = year + 10;
            if (year > 99) year = year - (year / 10) * 10; 
          break;

          case 1:
            if (month > 10) month = month - 10;
            else if (month < 3) month = month + 10;
          break;

          case 2:
            day = day + 10;
            if (day > (daysOfMonth[month - 1] + isLeapYear(year))) day = day - (day / 10) * 10;
            if (day == 0) day = 1;
          break;

          case 3:
            hours = hours + 10;
            if (hours > 23) hours = hours - (hours / 10 ) * 10;
          break;

          case 4:
            minutes = minutes + 10;
            if (minutes > 59) minutes = minutes - (minutes / 10 ) * 10;
          break;

          case 5:
            seconds = seconds + 10;
            if (seconds > 59) seconds = seconds - (seconds / 10 ) * 10;
          break;
        }
      printf("\r");
    }
    while (inChar != 13);
    tempByte++;
  }
  while (tempByte < 6);  

  // Write new date/time into the RTC
  writeRTC(seconds, minutes, hours, day, month, year);
  printf(" ...done      \r\n");
  printf("IOS: RTC date/time updated (");
  printDateTime(1);
  printf(")\r\n");
}

// ------------------------------------------------------------------------------

uint8_t toInt(char str[2])
// Convert a 2 char ascii number [00..99] to uint8_t
{
  if (str[0] == ' ') str[0] = '0';                        // Correction for 1..9 days
  return (str[1] - '0') + (str[0] - '0') * 10;
}


// ------------------------------------------------------------------------------
//
// 68008 CPU boot routines
//
// ------------------------------------------------------------------------------


void SetDefaultBaud(void)
// Set the default baud rates (115200) for both the two serial ports and wait for a reboot
// while User led blinks quickly (Baud recovery)
{
  uint32_t  timeStamp;
  
  DATAEE_Update(Ser1Addr, 6);                             // Set EUSART1 and EUSART2 baud rates at 115200
  DATAEE_Update(Ser2Addr, 6);
  printf("\r\n\nIOS: Baud recovery - Please reboot now!");
  while (1)
  {
    if ((millis() - timeStamp) > 100)
    {
      USER_LED_Toggle();
      timeStamp = millis();
    }
  }
}

// ------------------------------------------------------------------------------

uint16_t calcSPxBRG(uint8_t serSpeed)
// Calculate the the value of the 16bit register SPxBRG for a given speed code [0..6]]
{
  uint8_t regValueH, regValueL;
  switch (serSpeed)
  {
    case 0:                                               // 300
      regValueH = 0xd0;
      regValueL = 0x54;
    break;
              
    case 1:                                               // 1200
      regValueH = 0x34;
      regValueL = 0x14;
    break;
               
    case 2:                                               // 2400
      regValueH = 0x1a;
      regValueL = 0x0a;
    break;
              
    case 3:                                               // 9600
      regValueH = 0x06;
      regValueL = 0x82;
    break;
            
    case 4:                                               // 19200
      regValueH = 0x03;
      regValueL = 0x40;
    break;
            
    case 5:                                               // 57600
      regValueH = 0x01;
      regValueL = 0x15;
    break;
            
    case 6:                                               // 115200
      regValueH = 0x00;
      regValueL = 0x8a;
    break;
  }
  return (uint16_t)((regValueH << 8) + regValueL);
}

// ------------------------------------------------------------------------------

uint8_t DATAEE_Update(uint16_t bAdd, uint8_t bData)
// Wite a byte in an EEPROM location only if it is different from the current value
// Return value = 0:  new value = current
//              = 1:  new value != current
{
  uint8_t i = 0;
  if (bData != DATAEE_ReadByte(bAdd))
  {
    DATAEE_WriteByte(bAdd, bData);
    i = 1;
  }
  return i;
}

// ------------------------------------------------------------------------------

void printMsg1()
// Print a message
{
  printf("\r\nPress CR to accept, ESC to exit or any other key to change\r\n\n");
}

// ------------------------------------------------------------------------------

void busFatalErr(uint8_t errCode)
// Print a bus error code and stop any operation.
{
  printf("\n\rIOS: CPU bus fatal error code: %u - Aborted\n\r", errCode);
  while (1);
}

// ------------------------------------------------------------------------------

inline void singleStepDrive()
// Set the IOS single step drive mode and resume the CPU from the reset.
{
  Set_S_STEP__Low;                                        // Set S-STEP active, so a wait state is enabled for every bus cycle
  Set_DT_EN__Low;                                         // Set DT_EN_ active, so a wait state is generated on the first next bus cycle
  HALT__LAT = 1;                                          // Let the CPU exit from RESET (HALT_ = 0 and RESET_ = 0)
  RESET__LAT = 1;
  // CPU bus will enter in a wait state on the first next (read) bus cycle
}

// ------------------------------------------------------------------------------

void feedReadCycle(uint8_t byte)
// Feed a byte on the CPU bus using a read bus cycle.
// WARNING: the CPU must be in the IOS single step drive mode.
// NOTE: for a 68008 @ 8MHz a single S state (half the clock period) time is equal to the execution of a PIC NOP instruction (62.5ns).
// For more info about the 68008 CPU bus timing see the 68008 datasheet.
{
  uint8_t         i;
    
  // Wait until the CPU bus is in a wait state (DTACK_ = 1)
  i = 0;
  do
  {
    if (i > maxDtackRetry) busFatalErr(1);                // Abort operations if too much retries done (code 1)
    i++;
  }
  while (DTACK_ == 0);
    
  // Now the CPU is in a bus wait state. Check that the CPU is in a read bus cycle
  if (R_W__GetValue() == 0) busFatalErr(2);               // Abort operations if it isn't a read bus cycle (code 2)
    
  // DEBUG
  #if DEBUG_L > 0 
    printf("* DEBUG: Byte to feed: 0x%02X\n\r", byte);
  #endif
    
  // Now the CPU is in a bus wait state (during a read bus cycle)
  HALT__LAT = 0;                                          // Set HALT_ active
  RAM_EN_LAT = RAM_EN_Disabled;                           // Disable RAM
  D0_7_DIR = 0x00  ;                                      // Set D0-D7 data bus as output
  LATD = byte;                                            // Output the byte on the data bus
  Set_DT_EN__High;                                        // Let the 68008 CPU exit from the wait state and complete the current read bus cycle
  NOP();                                                  // Wait (at lest 6 * 62.5ns) for CPU in halt state 
  NOP();
  NOP(); 
  NOP(); 
  NOP(); 
  NOP();
    
  // Now the CPU is in bus halt state
  D0_7_DIR = 0xFF ;                                       // Set D0-D7 data bus as input (pullup are already active)
  RAM_EN_LAT = RAM_EN_Enabled;                            // Enable RAM
  Set_DT_EN__Low;                                         // Now it is possible re-enable the wait state request for next bus cycle
  HALT__LAT = 1;                                          // Disable HALT_ signal and let the CPU go to next bus cycle
  // Now CPU will go in a bus wait state on the next (read or write) bus cycle
}

// ------------------------------------------------------------------------------

void skipWriteCycles()
// Let the CPU execute up to <maxWriteCycles> consecutive bus write cycles.
// WARNING: the CPU must be in the IOS single step drive mode.
// NOTE: for a 68008 @ 8MHz a single S state (half the clock period) time is equal to the execution of a PIC NOP instruction (62.5ns).
// For more info about the 68008 CPU bus timing see the 68008 datasheet.
{
  uint8_t         i, j;
    
  // Wait until the CPU bus is in a wait state (DTACK_ = 1)
  i = 0;
  do
  {
    if (i > maxDtackRetry) busFatalErr(3);                 // Abort operations if too much retries done (code 3)
    i++;
  }
  while (DTACK_ == 0);
    
  // Now the CPU is in a bus wait state. Return if it is a read cycle or go further if is write cycle 
  if (R_W__GetValue() == 0)
  // It is a write bus cycle
  {
    // Let the CPU execute a write bus cycle
    j = 1;
    do
    {
      // Execute a write bus cycle
      if (j > maxWriteCycles) busFatalErr(5);             // Abort operations if too much consecutive write bus cycles (code 5)
      j++;
            
      // DEBUG
      #if DEBUG_L > 0 
        printf("* DEBUG: Executed Write Bus Cycle\n\r");
      #endif

      // !!!Time critical section!!!
      INTERRUPT_GlobalInterruptDisable();                 // !!! Start of a time critical section. No interrupt allowed
      Set_DT_EN__High;                                    // !!! Set DT_EN_ not active to let the CPU exit from the wait state and 
                                                          // !!!  complete the bus cycle
      NOP();                                              // !!! Wait 3 * 62.5ns
      NOP();                                              // !!!
      NOP();                                              // !!!
      Set_DT_EN__Low;                                     // !!! Enable DT_EN_ again to request a wait state on (before) the 
                                                          // !!!  next bus cycle if needed
      INTERRUPT_GlobalInterruptEnable();                  // !!! End of a time critical section. Interrupt resumed
            
      // Wait until the CPU bus is in a wait state (DTACK_ = 1)
      i = 0;
      do
      {
        if (i > maxDtackRetry) busFatalErr(4);            // Abort operations if too much retries done (code 4)
        i++;
      }
      while (DTACK_ == 0);
            
      // Now the CPU is in a bus wait state.
    }
  while (R_W__GetValue() == 0);
  }
}


// ------------------------------------------------------------------------------
//
// SD Disk routines (FAT16 and FAT32 filesystems supported) using the PetitFS library.
// For more info about PetitFS see here: http://elm-chan.org/fsw/ff/00index_p.html
//
// ------------------------------------------------------------------------------


void openFileSD(const char *  fileNameSD)
// Mount a volume on SD and open the fileNameSD file
{
  // Mount a volume on SD
  if (mountSD(&filesysSD))
  // Error mounting. Try again
  {
    errCodeSD = mountSD(&filesysSD);
    if (errCodeSD)
    // Error again. Repeat until error disappears (or the user forces a reset)
    do
    {
      printErrSD(0, errCodeSD, NULL);
      waitKey();                                          // Wait a key to repeat
      mountSD(&filesysSD);                                // New double try
      errCodeSD = mountSD(&filesysSD);
    }
    while (errCodeSD);
  }

  // Open the selected file to load
  errCodeSD = openSD(fileNameSD);
  if (errCodeSD)
  // Error opening the required file. Repeat until error disappears (or the user forces a reset)
  do
  {
    printErrSD(1, errCodeSD, fileNameSD);
    waitKey();                                            // Wait a key to repeat
    errCodeSD = openSD(fileNameSD);
    if (errCodeSD != 3)
    // Try to do a two mount operations followed by an open
    {
      mountSD(&filesysSD);
      mountSD(&filesysSD);
      errCodeSD = openSD(fileNameSD);
    }
  }
  while (errCodeSD);
}

// ------------------------------------------------------------------------------

void printOsName(uint8_t currentDiskSet)
// Print the current Disk Set number and the OS name, if it is defined.
// The OS name is inside the file defined in DS_OSNAME
{
  printf("Disk Set %u", currentDiskSet);
  OsName[2] = currentDiskSet + 48;                        // Set the Disk Set
  openSD(OsName);                                         // Open file with the OS name
  readSD(bufferSD, &numReadBytes);                        // Read the OS name
  if (numReadBytes > 0)
  // Print the OS name
  {
    printf(" (%s)", (const char *)bufferSD);
  }
}

// ------------------------------------------------------------------------------

uint8_t mountSD(FATFS* fatFs)
// Mount a volume on SD: 
// *  "fatFs" is a pointer to a FATFS object (PetitFS library)
// The returned value is the resulting status (0 = ok, otherwise see printErrSD())
{
  setSpiSpeed(SPI_LOW);                                   // Set a low SPI speed (100kHz) for the SD initialization
  return pf_mount(fatFs);
  setSpiSpeed(SPI_HIGH);                                  // Now set SPI to 8MHz for normal operations
}

// ------------------------------------------------------------------------------

uint8_t openSD(const char* fileName)
// Open an existing file on SD:
// *  "fileName" is the pointer to the string holding the file name (8.3 format)
// The returned value is the resulting status (0 = ok, otherwise see printErrSD())
{
  return pf_open(fileName);
}

// ------------------------------------------------------------------------------

uint8_t readSD(void* buffSD, uint16_t* numReadBytes)
// Read one SD sector (512 bytes) starting from the current logical sector of the opened file on SD:
// *  "BuffSD" is the pointer to the buffer;
// *  "numReadBytes" is the pointer to the variables that store the number of read bytes;
//     if < 512 (including = 0) an EOF was reached).
// The returned value is the resulting status (0 = ok, otherwise see printErrSD())
//
// NOTE:  Past current sector boundary, the next logical sector will be pointed. So to read a whole file it is sufficient 
//        call readSD() consecutively until EOF is reached
{
  UINT  numBytes;
  uint8_t  errcode;
  errcode = pf_read(buffSD, 512, &numBytes);
  *numReadBytes = (uint16_t) numBytes;
  return errcode;
}

// ------------------------------------------------------------------------------

uint8_t writeSD(void* buffSD, uint16_t* numWrittenBytes)
// Write one SD sector (512 bytes) starting from the current logical sector (512 bytes) of the opened file on SD:
// *  "BuffSD" is the pointer to the buffer;
// *  "numWrittenBytes" is the pointer to the variables that store the number of written bytes;
//     if < 512 (including = 0) an EOF was reached.
// The returned value is the resulting status (0 = ok, otherwise see printErrSD())
//
// NOTE 1: Past current logical sector boundary, the next logical sector will be pointed. So to write a whole file it is sufficient 
//         call writeSD() consecutively until EOF is reached
//
// NOTE 2: To finalize the current write operation a writeSD(NULL, &numWrittenBytes) must be called as last action
{
  UINT  numBytes;
  uint8_t  errcode;
  if (buffSD != NULL)
  {
    errcode = pf_write(buffSD, 512, &numBytes);
  }
  else
  {
    errcode = pf_write(0, 0, &numBytes);
  }
  *numWrittenBytes = (uint16_t) numBytes;
  return errcode;
}

// ------------------------------------------------------------------------------

uint8_t seekSD(uint16_t sectNum)
// Set the pointer of the current logical sector for the current opened file on SD:
// *  "sectNum" is the logical sector number to set. First sector is 0.
// The returned value is the resulting status (0 = ok, otherwise see printErrSD())
//
// NOTE: "secNum" is in the range [0..16383], and the logical sector addressing is continuous inside a "disk file";
//       16383 = (512 * 32) - 1, where 512 is the number of emulated tracks, 32 is the number of emulated sectors (512 bytes))
//
{
  uint8_t i;
  return pf_lseek(((unsigned long) sectNum) << 9);
}

// ------------------------------------------------------------------------------

void printErrSD(uint8_t opType, uint8_t errCode, const char* fileName)
// Print the error occurred during a SD I/O operation:
//  * "OpType" is the operation that generated the error (0 = mount, 1= open, 2 = read,
//     3 = write, 4 = seek);
//  * "errCode" is the error code from the PetitFS library (0 = no error);
//  * "fileName" is the pointer to the file name or NULL (no file name)
//
// ........................................................................
//
// Errors legend (from PetitFS library) for the implemented operations:
//
// ------------------
// mountSD():
// ------------------
// NOT_READY
//     The storage device could not be initialized due to a hard error or no medium.
// DISK_ERR
//     An error occurred in the disk read function.
// NO_FILESYSTEM
//     There is no valid FAT partition on the drive.
//
// ------------------
// openSD():
// ------------------
// NO_FILE
//     Could not find the file.
// DISK_ERR
//     The function failed due to a hard error in the disk function, a wrong FAT structure or an internal error.
// NOT_ENABLED
//     The volume has not been mounted.
//
// ------------------
// readSD() and writeSD():
// ------------------
// DISK_ERR
//     The function failed due to a hard error in the disk function, a wrong FAT structure or an internal error.
// NOT_OPENED
//     The file has not been opened.
// NOT_ENABLED
//     The volume has not been mounted.
// 
// ------------------
// seekSD():
// ------------------
// DISK_ERR
//     The function failed due to an error in the disk function, a wrong FAT structure or an internal error.
// NOT_OPENED
//     The file has not been opened.
//
// ........................................................................
{
  if (errCode)
  {
    printf("\r\nIOS: SD error ");
    printf("%u",errCode);
    printf(" (");
    switch (errCode)
    // See PetitFS implementation for the codes
    {
      case 1: printf("DISK_ERR"); break;
      case 2: printf("NOT_READY"); break;
      case 3: printf("NO_FILE"); break;
      case 4: printf("NOT_OPENED"); break;
      case 5: printf("NOT_ENABLED"); break;
      case 6: printf("NO_FILESYSTEM"); break;
      default: printf("UNKNOWN"); 
    }
    printf(" on ");
    switch (opType)
    {
      case 0: printf("MOUNT"); break;
      case 1: printf("OPEN"); break;
      case 2: printf("READ"); break;
      case 3: printf("WRITE"); break;
      case 4: printf("SEEK"); break;
      default: printf("UNKNOWN");
    }
    printf(" operation");
    if (fileName)
    // Not a NULL pointer, so print file name too
    {
      printf(" - File: ");
      printf(fileName);
    }
    printf(")\r\n\n");
  }
}

// ------------------------------------------------------------------------------

void waitKey()
// Wait a key to continue
{
  while (eusart1RxCount > 0) EUSART1_Read(); // Flush serial Rx buffer
  printf("IOS: Check SD and press a key to repeat\r\n\n");
  while(eusart1RxCount < 1);
}


// ------------------------------------------------------------------------------
//
// EXECUTE WRITE OPCODE (I/O write port address = 0x00)
//
// ------------------------------------------------------------------------------


inline void execWriteOpcode()
{
  switch (ioOpcode)
  // Execute the requested I/O WRITE Opcode. The 0xFF value is reserved as "No operation".
  {
    case  0x00:
      // USER LED:      
      //                I/O DATA:    D7 D6 D5 D4 D3 D2 D1 D0
      //                            ---------------------------------------------------------
      //                              x  x  x  x  x  x  x  0    USER Led off
      //                              x  x  x  x  x  x  x  1    USER Led on
          
      if (ioData & 0x01) USER_LED_SetHigh();
      else USER_LED_SetLow();
    break;
    
    case  0x01:
      // SERIAL 1 TX:     
      //                I/O DATA:    D7 D6 D5 D4 D3 D2 D1 D0
      //                            ---------------------------------------------------------
      //                             D7 D6 D5 D4 D3 D2 D1 D0    ASCII char to be sent to serial 1
          
      putch(ioData);
    break;
    
    case  0x02:
      // SETIRQ:
      // Set or reset the flag to enable/disable the activation of the 68008 IPL1 IRQ signal (Full HW conf. only)
      //
      //                I/O DATA:    D7 D6 D5 D4 D3 D2 D1 D0
      //                            ---------------------------------------------------------
      //                              x  x  x  x  x  x  x  0    Serial 1 Rx IRQ off
      //                              x  x  x  x  x  x  x  1    Serial 1 Rx IRQ on
      //                              x  x  x  x  x  x  0  x    Serial 2 Rx IRQ off
      //                              x  x  x  x  x  x  1  x    Serial 2 Rx IRQ on
      //                              x  x  x  x  x  0  x  x    Systick IRQ off
      //                              x  x  x  x  x  1  x  x    Systick IRQ on
      //
      //
      // Vector table for the available IRQ:
      //
      //                               IRQ name          | IRQ Vector | RAM address
      //                      ---------------------------+------------+--------------
      //                               Serial 1 Rx       |    0x40    |   0x00100
      //                               Serial 2 Rx       |    0x41    |   0x00104
      //                                 Systick         |    0x42    |   0x00108
      //
      //
      // NOTE 1: The default value for all the supported IRQ after a reset/power on is 0 (all IRQ disabled).
      // NOTE 2: Currently the events that can set the IPL1 signal are the "Serial 1 Rx" (a char is present 
      //         on the read buffer) and the Systick timer
      // NOTE 3: Supported only for the Full HW configuration option!

      if (!LiteConfFlg)
      {
        if (ioData & 1) CPUIntRx1En = 1;                  // Enable the IPL1 IRQ on serial 1 Rx
        if (ioData & 1<<1) CPUIntRx2En = 1;               // Enable the IPL1 IRQ on serial 2 Rx ****************************************
        if (ioData & 1<<2) CPUIntTickEn = 1;              // Enable the IRQ on Systick timer *******************************************
      }
    break;

    case  0x03:
      // GPIOA Write (GPE Option):
      //
      //                I/O DATA:    D7 D6 D5 D4 D3 D2 D1 D0
      //                            ---------------------------------------------------------
      //                             D7 D6 D5 D4 D3 D2 D1 D0    GPIOA value (see MCP23017 datasheet)
      //
      // NOTE 1: If the GPIO is set to operate as an SPP port this Opcode is ignored.
      // NOTE 2: If the GPIO expansion module is not found this Opcode is ignored.
          
      if (moduleGPIO && !SPPmode)
      {
        I2C2_Write1ByteRegister(GPIOEXP_ADDR, GPIOA_REG, ioData);
      }
    break;
        
    case  0x04:
      // GPIOB Write (GPE Option): 
      //   
      //                I/O DATA:    D7 D6 D5 D4 D3 D2 D1 D0
      //                            ---------------------------------------------------------
      //                             D7 D6 D5 D4 D3 D2 D1 D0    GPIOB value (see MCP23017 datasheet)
      //
      // NOTE 1: If the GPIO is set to operate as an SPP port this Opcode is ignored.
      // NOTE 2: If the GPIO expansion module is not found this Opcode is ignored.
          
      if (moduleGPIO && !SPPmode)
      {
        I2C2_Write1ByteRegister(GPIOEXP_ADDR, GPIOB_REG, ioData);
      }
    break;
        
    case  0x05:
      // IODIRA Write (GPE Option):
      //
      //                I/O DATA:    D7 D6 D5 D4 D3 D2 D1 D0
      //                            ---------------------------------------------------------
      //                             D7 D6 D5 D4 D3 D2 D1 D0    IODIRA value (see MCP23017 datasheet)
      //
      // NOTE 1: If the GPIO is set to operate as an SPP port this Opcode is ignored.
      // NOTE 2: If the GPIO expansion module is not found this Opcode is ignored.
          
      if (moduleGPIO && !SPPmode)
      {
        I2C2_Write1ByteRegister(GPIOEXP_ADDR, IODIRA_REG, ioData);
      }
    break;
        
    case  0x06:
      // IODIRB Write (GPE Option):
      //
      //                I/O DATA:    D7 D6 D5 D4 D3 D2 D1 D0
      //                            ---------------------------------------------------------
      //                             D7 D6 D5 D4 D3 D2 D1 D0    IODIRB value (see MCP23017 datasheet)
      //
      // NOTE 1: If the GPIO is set to operate as an SPP port this Opcode is ignored.
      // NOTE 2: If the GPIO expansion module is not found this Opcode is ignored.
          
      if (moduleGPIO && !SPPmode)
      {
        I2C2_Write1ByteRegister(GPIOEXP_ADDR, IODIRB_REG, ioData);
      }
    break;
        
    case  0x07:
      // GPPUA Write (GPE Option):
      //
      //                I/O DATA:    D7 D6 D5 D4 D3 D2 D1 D0
      //                            ---------------------------------------------------------
      //                             D7 D6 D5 D4 D3 D2 D1 D0    GPPUA value (see MCP23017 datasheet)
      //
      // NOTE 1: If the GPIO is set to operate as an SPP port this Opcode is ignored.
      // NOTE 2: If the GPIO expansion module is not found this Opcode is ignored.
          
      if (moduleGPIO && !SPPmode)
      {
        I2C2_Write1ByteRegister(GPIOEXP_ADDR, GPPUA_REG, ioData);
      }
    break;
        
    case  0x08:
      // GPPUB Write (GPIO Exp. Mod. ):
      //
      //                I/O DATA:    D7 D6 D5 D4 D3 D2 D1 D0
      //                            ---------------------------------------------------------
      //                             D7 D6 D5 D4 D3 D2 D1 D0    GPPUB value (see MCP23017 datasheet)
      //
      // NOTE 1: If the GPIO is set to operate as an SPP port this Opcode is ignored.
      // NOTE 2: If the GPIO expansion module is not found this Opcode is ignored.
        
      if (moduleGPIO && !SPPmode)
      {
        I2C2_Write1ByteRegister(GPIOEXP_ADDR, GPPUB_REG, ioData);
      }
    break;
       
    case  0x09:
      // DISK EMULATION
      // SELDISK - select the emulated disk number (binary). 100 disks are supported [0..99]:
      //
      //                I/O DATA:    D7 D6 D5 D4 D3 D2 D1 D0
      //                            ---------------------------------------------------------
      //                             D7 D6 D5 D4 D3 D2 D1 D0    DISK number (binary) [0..99]
      //
      //
      // Opens the "disk file" corresponding to the selected disk number, doing some checks.
      // A "disk file" is a binary file that emulates a disk using a LBA-like logical sector number.
      // Every "disk file" must have a dimension of 8388608 bytes, corresponding to 16384 LBA-like logical sectors
      //  (each physical sector is 512 bytes long), corresponding to 512 tracks of 32 sectors of 512 bytes each 
      //  (see SELTRACK and SELSECT opcodes).
      //
      // Errors are stored into "errDisk" (see ERRDISK opcode).
      //
      //
      // ...........................................................................................
      //
      // "Disk file" filename convention:
      //
      // Every "disk file" must follow the syntax "DSsNnn.DSK" where
      //
      //    "s" is the "disk set" and must be in the [0..9] range (always one numeric ASCII character)
      //    "nn" is the "disk number" and must be in the [00..99] range (always two numeric ASCII characters)
      //
      // ...........................................................................................
      //          
      //
      // NOTE 1: The maximum disks number may be lower due the limitations of the used OS (e.g. CP/M supports
      //         a maximum of 16 disks)
      // NOTE 2: Because SELDISK opens the "disk file" used for disk emulation, before using WRITESECT or READSECT
      //         a SELDISK must be performed at first.
      // NOTE 3: The FLUSHBUFF opcode is internally executed before changing the current disk to avoid possible 
      //         data loss if the "128 bytes sector emulation" is active and there is a deferred write operation pending.
      
      if (ioData <= maxDiskNum)
      // Valid disk number. Set the name of the file to open as virtual disk, and open it
      {
        // Change current virtual disk must
        {
          
          // DEBUG
          #if DEBUG_SELDSK > 0 
            printf("\r\n* DEBUG: SELDISK * selected disk = %u\r\n", ioData);
          #endif

          flushBuffer();                                  // Flush (write) the buffer to the SD if needed
          diskName[2] = diskSet + 48;                     // Set the current "disk set"
          diskName[4] = (ioData / 10) + 48;               // Set the disk number
          diskName[5] = ioData - ((ioData / 10) * 10) + 48;
          if (!diskErr)
          {
            diskErr = openSD(diskName);                   // Open the "disk file" corresponding to the given disk number and "disk set"
          }
          bufferNotValid = 1;                             // Changed disk, so mark the buffer content as invalid to force a physical 
                                                          //  read from SD
        }
      }
      else diskErr = 16;                                  // Illegal disk number
    break;
    
    case  0x0A:
      // DISK EMULATION
      // SELTRACK - select the emulated track number (word split in 2 bytes in sequence: DATA 0 and DATA 1):
      //
      //                I/O DATA 0:  D7 D6 D5 D4 D3 D2 D1 D0
      //                            ---------------------------------------------------------
      //                             D7 D6 D5 D4 D3 D2 D1 D0    Track number (binary) LSB [0..255]
      //
      //                I/O DATA 1:  D7 D6 D5 D4 D3 D2 D1 D0
      //                            ---------------------------------------------------------
      //                             D7 D6 D5 D4 D3 D2 D1 D0    Track number (binary) MSB [0..1]
      //
      //
      // Stores the selected track number into "trackSel" for "disk file" access.
      // A "disk file" is a binary file that emulates a disk using a LBA-like logical sector number.
      // The SELTRACK and SELSECT operations convert the legacy track/sector address into a LBA-like logical 
      //  sector number used to set the logical sector address inside the "disk file".
      // A control is performed on current track number for valid values.
      // Errors are stored into "diskErr" (see ERRDISK opcode).
      //
      //
      // NOTE 1: Allowed track numbers are in the range [0..511] (512 tracks)
      // NOTE 2: Before a WRITESECT or READSECT operation at least a SELSECT or a SELTRAK operation
      //         must be performed
      
      if (ioByteCnt == 0)
      // LSB
      {
        trackSel = ioData;
      }
      else
      // MSB
      {
        trackSel = (((uint16_t) ioData) << 8) | LOW_BYTE(trackSel);
        if (trackSel < 512)
        // Track numbers valid
        {
          diskErr = 0;                                    // No errors
        }
        else
        // Track invalid number
        {
          diskErr = 17;                                   // Illegal track number
        }
        ioOpcode = 0xFF;                                  // All done. Set ioOpcode = "No operation"
      }
      ioByteCnt++;
    break;

    case  0x0B:
      // DISK EMULATION
      // SELSECT - select the emulated sector number (binary):
      //
      //                  I/O DATA:  D7 D6 D5 D4 D3 D2 D1 D0
      //                            ---------------------------------------------------------
      //                             D7 D6 D5 D4 D3 D2 D1 D0  Sector number (binary) [0..(*)]
      //
      //
      // Stores the selected sector number into "sectSel" for "disk file" access.
      // A "disk file" is a binary file that emulates a disk using a LBA-like logical sector number.
      // The SELTRACK and SELSECT operations convert the legacy track/sector address into a LBA-like logical 
      //  sector number used to set the logical sector address inside the "disk file".
      // A control is performed current sector number for valid values.
      // Errors are stored into "diskErr" (see ERRDISK opcode).
      //
      // (*) The maximum valid sector number depends on whether the "128 bytes sector emulation" is active or not.
      //     If it is active the maximum value is 127, if not it is 31.
      //
      // NOTE:   Before a WRITESECT or READSECT operation at least a SELSECT or a SELTRAK operation
      //         must be performed

      sectSel = ioData;
      if ((sect128Flag && (sectSel > 127)) || (!sect128Flag && (sectSel > 31)))
      // Sector number not valid
      {
        diskErr = 18;                                     // Illegal sector number
      }
      else
      // Sector valid number
      {
        diskErr = 0;                                      // No errors
      }
    break;
    
    case  0x0C:
      // DISK EMULATION
      // WRITESECT - write 128/512 data bytes sequentially into the current emulated disk/track/sector:
      //
      //                 I/O DATA n:   D7 D6 D5 D4 D3 D2 D1 D0
      //                              ---------------------------------------------------------
      //                 I/O DATA 0    D7 D6 D5 D4 D3 D2 D1 D0   <1st Data Bytes> (First byte)
      //
      //                      |               |                         |
      //                      |               |                         |
      //                      |               |                         |            
      //                      |               |                         |
      //
      //             I/O DATA 127/511  D7 D6 D5 D4 D3 D2 D1 D0   <128/512th Data Bytes> (Last byte)
      //
      //
      // Writes the current emulated sector (128/512 bytes) of the current disk/track/sector, one data byte each call. 
      // Both 512 or 128 bytes sector sizes are supported. Because the physical sector size on the SD is 512 bytes,
      //  if it is selected an emulated sector dimension of 128 bytes a sector blocking will be performed transparently.
      // The selection between 128 or 512 bytes sector size is done by IOS at boot time (see <sect128Flag>).
      // When the "128 bytes sector emulation" is active, the blocking algorithm optimizes the physical pre-read operations 
      //  from SD, so a physical pre-read is performed only when really needed.
      // When the "128 bytes sector emulation" is active, the WRITESEC opcode uses a "deferred write" strategy, so a sector 
      //  is written into the SD only when really needed. More, the phisycal sector write is not done by
      //  the WRITESECT opcode, but by the FLUSHBUFF opcode, that can be "internally called" if needed.
      // All the 128/512 calls must be always performed sequentially to have a WRITESECT operation correctly done. 
      // If an error occurs during the WRITESECT operation, all subsequent write data will be ignored and
      //  the write finalization will not be done.
      // If an error occurs calling any DISK EMULATION opcode immediately before the WRITESECT 
      //  opcode call, all the write data will be ignored and the WRITESECT operation will not be performed.
      // Errors are stored into "diskErr" (see ERRDISK opcode).
      //
      // NOTE 1: Before a WRITESECT operation at least a SELTRACK or a SELSECT should be always performed
      // NOTE 2: Remember to open the right "disk file" at first using the SELDISK opcode
      // NOTE 3: The "logical sectors" are the 512 bytes sectors inside a "disk file" forming a virtual disk on SD. They are
      //         identified with a 14 bit LBA-like logical sector address created as TTTTTTTTTSSSSS (binary), where
      //         TTTTTTTTT (binary) is the track number and SSSSS (binary) is the emulated 512 bytes sector number.
      // NOTE 4: When the "128 bytes sector emulation" is active, a 128 bytes emulated sector is "blocked" inside a 512 bytes 
      //         emulated sector (a 512 bytes emulated sector contains four 128 bytes emulated sectors).
      // NOTE 5: The "emulated sector" is the sector seen by the CPU, identified by the disk/track/sector numbers, 
      //         and can be 512 or 128 bytes large.
      // NOTE 6: When the "128 bytes sector emulation" is not active, the write finalization on SD "disk file" is executed 
      //         only on the 512th data byte exchange, so be sure that exactly 512 data bytes are exchanged.
 
      if (ioByteCnt == 0)
      // Write operation preparation. First byte of 128/512, so set the right file pointer for the SD to the 
      //  current emulated track/sector.
      // If the "128 bytes sector emulation" is active; do a logical sector (512 bytes) pre-read before 
      //  for sector blocking, if required
      {
        if (((sect128Flag && (sectSel < 128)) || (!sect128Flag && (sectSel < 32))) && !diskErr)
        // Sector and track numbers valid and no previous error; set the LBA-like logical sector
        {
          if (sect128Flag) sectSel512 = sectSel / 4;      // Correction for the "128 bytes sector emulation" if active
          else sectSel512 = sectSel;
          logicSect = (trackSel << 5) | sectSel512;       // Calculate the new current logical sector (512 bytes) number 
                                                          // inside the "disk file" on SD using a 14 bit "disk file" LBA-like 
                                                          //  logical sector address created as TTTTTTTTTSSSSS (binary)
          if (sect128Flag)
          // "128 bytes sector emulation" active, so do a logical sector (512 bytes) pre-read before write if required,
          //  and set the index to the sector buffer for sector blocking
          {
            sectBuffIndex = 128 * (sectSel % 4);          // Set the index of the first data byte of the sector buffer
            readSDrequired = (logicSect != buffSect) || bufferNotValid;  // Set the flag at 1 if a read from SD is required, 0 otherwise.
                                                          // Note: a change on the current disk sets <buffSect> as invalid, 
                                                          //       so forcing a read from SD
            if (readSDrequired)
            // Logical sector pre-read required (for sector blocking)
            {
              
              // DEBUG
              #if DEBUG_WRSECT > 0 
                printf("\n\r* DEBUG: WRITESECT * pre-read needed - LBA = 0x%02X ", logicSect);
              #endif

              flushBuffer();                              // Flush (write) the buffer to the SD if needed
              if (!diskErr) diskErr = seekSD(logicSect);
              if (!diskErr) diskErr = readSD(bufferSD, &numReadBytes); 
              buffSect = logicSect;                       // Update the logical sector currently in the buffer
              if (!diskErr && (numReadBytes < 512)) 
              // Reached an unexpected EOF
              {  
                bufferNotValid = 1;                       // Set the buffer content as invalid
                diskErr = 19;                             // Set the proper error code
              }
            }
            pendingBuffWrite = 1;                         // Starting to write a sector, so set the pending write flag...
            bufferNotValid = 0;                           // ...and clear the buffer not valid flag
          }
          else
          // Using 512 bytes sectors: pre-read, sector blocking and deferred write are not required
          {
            sectBuffIndex = 0;
            diskErr = seekSD(logicSect);                  // Set logical sector for the write operation
          }
          
          // DEBUG
          #if DEBUG_WRSECT > 0 
            printf("\n\r* DEBUG: WRITESECT * trackSel = %u - sectSel = %u - sectSel512 = %u\n\r", trackSel, sectSel, sectSel512);
          #endif
          
        }
      }
      if (!diskErr)
      // No previous error (e.g. track or sector), so store the sector into the buffer
      // If the "128 bytes sector emulation" is active a deferred sector write strategy is used to optimize write accesses to the SD
      {
        bufferSD[sectBuffIndex] = ioData;                 // Store current data byte in the buffer array
  
        if ((sect128Flag && (ioByteCnt >= 127)) || (!sect128Flag && (ioByteCnt >= 511)))
        // Last data byte. Write the buffer into the SD only if the "128 bytes sector emulation" is not active
        {
          if (!sect128Flag)
          {  
            diskErr = writeSD(bufferSD, &numWriBytes);    // Write buffer to SD
            if (!diskErr && (numWriBytes < 512)) diskErr = 19;    // Reached an unexpected EOF
            if (!diskErr) diskErr = writeSD(NULL, &numWriBytes);  // Finalize the write operation
          }
          ioOpcode = 0xFF;                                // All done. Set ioOpcode = "No operation"
        }
      }
      ioByteCnt++;                                        // Increment the counter of the exchanged data bytes
      sectBuffIndex++;                                    // Increment the index of the current data byte inside the sector buffer
    break;
    
    // ****************************************************************************************************************************** NEW
    case  0x0F:
      // SETTICK - set the Systick timer time (milliseconds)
      //
      //                I/O DATA:    D7 D6 D5 D4 D3 D2 D1 D0
      //                            ---------------------------------------------------------
      //                             D7 D6 D5 D4 D3 D2 D1 D0    Systick time (binary) [1..255]
      //
      // Set/change the time (millisecond) used for the Systick timer.
      // At reset time the default value is 5ms (-> 200Hz).
      //
      // NOTE 1: If the time is 0 milliseconds the set operation is ignored
      // NOTE 2: The Systick timer must be then enabled using the SETIRQ opcode

      if (ioData >0) sysTickTime = ioData;
    break;
    // ****************************************************************************************************************************** END
        
    case  0x10:
      // SERIAL 2 TX:     
      //                I/O DATA:    D7 D6 D5 D4 D3 D2 D1 D0
      //                            ---------------------------------------------------------
      //                             D7 D6 D5 D4 D3 D2 D1 D0    ASCII char to be sent to serial 2
          
      EUSART2_Write(ioData);
    break;
    
    case  0x11:
      // SPP EMULATION
      // SETSPP - set the GPIO port into SPP mode:     
      //
      //                I/O DATA:    D7 D6 D5 D4 D3 D2 D1 D0
      //                            ---------------------------------------------------------
      //                              x  x  x  x  x  x  x  0    AUTOFD disabled
      //                              x  x  x  x  x  x  x  1    AUTOFD enabled
      //
      // The SETSPP opcode is used when an SPP Adapter board is connected to the GPIO port to work as a Standard Parallel Port.
      //
      // The following actions are performed:
      // - The SPP mode flag is set;
      // - The GPIO port is set (direction and pullup) to operate as a SPP port;
      // - The STROBE (active low) Control Line of the SPP port is set to High;
      // - D0 is used to set the status of AUTOFD (active Low) Contro Line of the SPP port (AUTOFD = !D0);
      // - The printer is initialized with a pulse on the INIT (active Low) Control line of the SPP port.
      //
      // GPIO port / SPP port signals table:
      //
      //                       GPIO Port |  SPP Port              | Dir
      //                      -------------------------------------------
      //                         GPA0    |  STROBE (active Low)   | Out
      //                         GPA1    |  AUTOFD (active Low)   | Out
      //                         GPA2    |  INIT (active Low)     | Out
      //                         GPA3    |  ACK (active Low)      | In
      //                         GPA4    |  BUSY (active High)    | In
      //                         GPA5    |  PAPEREND (active High)| In
      //                         GPA6    |  SELECT (active High)  | In
      //                         GPA7    |  ERROR (active Low)    | In
      //                         GPB0    |  D0                    | Out
      //                         GPB1    |  D1                    | Out
      //                         GPB2    |  D2                    | Out
      //                         GPB3    |  D3                    | Out
      //                         GPB4    |  D4                    | Out
      //                         GPB5    |  D5                    | Out
      //                         GPB6    |  D6                    | Out
      //                         GPB7    |  D7                    | Out
      //
      // NOTE 1: When the GPIO is set to operate as an SPP port all the GPIO write opcodes (GPIOA Write, GPIOB Write, IODIRA Write, 
      //         IODIRB Write, GPPUA Write, GPPUB Write) are ignored/disabled.
      // NOTE 2: If the GPIO expansion module (GPE) is not found this opcode is ignored.
      // NOTE 3: When the SPP mode is activated cannot be disabled anymore (the only way is reset the board).
      
      if (moduleGPIO)                                     // Only if GPE was found
      {
        SPPmode = 1;                                      // Set the SPP mode flag
        SPPautofd = (!ioData) & 0x01;                     // Store the value of the AUTOFD Control Line (active Low))
        
        // Set STROBE and INIT at 1, and AUTOFD = !D0
        I2C2_Write1ByteRegister(GPIOEXP_ADDR, GPIOA_REG, 0b00000101 | (uint8_t) (SPPautofd << 1));
        
        // Set the GPIO port to work as an SPP port (direction and pullup)
        I2C2_Write1ByteRegister(GPIOEXP_ADDR, IODIRA_REG, 0b11111000);  // 1 = input, 0 = ouput
        I2C2_Write1ByteRegister(GPIOEXP_ADDR, IODIRB_REG, 0b00000000);  // 1 = input, 0 = ouput
        I2C2_Write1ByteRegister(GPIOEXP_ADDR, GPPUA_REG, 0b11111111);   // 1 = pullup enabled, 0 = pullup disabled
        
        // Initialize the printer using a pulse on INIT
        // NOTE: The I2C protocol introduces delays greater than needed by the SPP, so no further delay is used here to generate the INIT pulse
        iByte = 0b00000001 | (uint8_t) (SPPautofd << 1);                // Change INIT bit to active (Low)
        I2C2_Write1ByteRegister(GPIOEXP_ADDR, GPIOA_REG, iByte);        // Set INIT bit to active (Low)
        iByte = iByte | 0b00000100;                                     // Change INIT bit to not active (High)
        I2C2_Write1ByteRegister(GPIOEXP_ADDR, GPIOA_REG, iByte);        // Set INIT bit to not active (High)
      }
    break;
    
    case  0x12:
      // SPP EMULATION
      // WRSPP - send a byte to the printer attached to the SPP port:     
      //
      //                I/O DATA:    D7 D6 D5 D4 D3 D2 D1 D0
      //                            ---------------------------------------------------------
      //                             D7 D6 D5 D4 D3 D2 D1 D0    byte to be sent to SPP
      //
      // If the SPP mode is enabled send a byte to the SPP. No check is done here to know if the printer is ready or not,
      // so you have to use the GETSPP opcode before for that.
      // If the SPP mode is disabled (or the GPE is not installed) this opcode is ignored.
      //
      // NOTE: to use WRSPP the SETSPP opcode should be called first to activate the SPP mode of the GPIO port.
      
      if (SPPmode)                                        // Only if SPP mode is active
      {
        // NOTE: The I2C protocol introduces delays greater than needed by the SPP, so no further delay is used here to generate the STROBE pulse
        I2C2_Write1ByteRegister(GPIOEXP_ADDR, GPIOB_REG, ioData);       // Put current data byte on GPIOB
        iByte = 0b11111100 | (uint8_t) (SPPautofd << 1);                // Change STROBE bit to active (Low)
        I2C2_Write1ByteRegister(GPIOEXP_ADDR, GPIOA_REG, iByte);        // Set STROBE bit to active (Low)
        iByte = iByte | 0b00000001;                                     // Change STROBE bit to not active (High)
        I2C2_Write1ByteRegister(GPIOEXP_ADDR, GPIOA_REG, iByte);        // Set STROBE bit to not active (High)
      }
    break;
    
    // **************************************************************************************************************************** NEW
    case  0x13:
      // DISK EMULATION (LBA)
      // SELLBA - select the current sector number as LBA (32bit split into 4 bytes in sequence: DATA 0-3):
      //
      //                I/O DATA 0:  D7 D6 D5 D4 D3 D2 D1 D0
      //                            ---------------------------------------------------------
      //                             D7 D6 D5 D4 D3 D2 D1 D0    Sector number (binary) LSB
      //
      //                I/O DATA 1:  D7 D6 D5 D4 D3 D2 D1 D0
      //                            ---------------------------------------------------------
      //                             D7 D6 D5 D4 D3 D2 D1 D0    Sector number (binary)
      //
      //                I/O DATA 2:  D7 D6 D5 D4 D3 D2 D1 D0
      //                            ---------------------------------------------------------
      //                             D7 D6 D5 D4 D3 D2 D1 D0    Sector number (binary)
      //
      //                I/O DATA 3:  D7 D6 D5 D4 D3 D2 D1 D0
      //                            ---------------------------------------------------------
      //                             D7 D6 D5 D4 D3 D2 D1 D0    Sector number (binary) MSB
      //
      //
      // Stores the selected sector number as LBA into "sectorLBA" for a subsequent disk "LBA operation" (READLBA or WRITELBA).
      // Any previous disk error code (diskErrLBA) is cleared.
      //
      // NOTE:   Before a READLBA or WRITELBA operation at least a SELLBA operation should be performed
      
      if (ioByteCnt == 0)
      // LSB (DATA 0)
      {
        sectorLBA = (uint32_t) ioData;
      }
      else
      {
        sectorLBA = sectorLBA | (((uint32_t) ioData) << (8 * ioByteCnt)); // DATA 1-3
        if (ioByteCnt > 2) 
        {  
          ioOpcode = 0xFF;                                // All done. Set ioOpcode = "No operation"
          diskErrLBA = 0;                                 // Clear any previous error code
        }
      }
      ioByteCnt++;
    break;
    // **************************************************************************************************************************** END
    
    // **************************************************************************************************************************** NEW
    case  0x14:
      // DISK EMULATION (LBA)
      // WRITELBA - write 512 data bytes sequentially into the current sector (as LBA):
      //
      //                 I/O DATA n:   D7 D6 D5 D4 D3 D2 D1 D0
      //                              ---------------------------------------------------------
      //                 I/O DATA 0    D7 D6 D5 D4 D3 D2 D1 D0   <1st Data Bytes> (First byte)
      //
      //                      |               |                         |
      //                      |               |                         |
      //                      |               |                         |            
      //                      |               |                         |
      //
      //                 I/O DATA 511  D7 D6 D5 D4 D3 D2 D1 D0   <512th Data Bytes> (Last byte)
      //
      //
      // Writes the current sector (512 bytes) one data byte each call. 
      // All the 512 calls must be always performed sequentially to have a WRITELBA operation correctly done. 
      // If an error occurs during the WRITELBA operation, all subsequent write data will be ignored and
      //  the write finalization will not be done.
      // If an error occurs calling any DISK EMULATION opcode immediately before the WRITELBA 
      //  opcode call, all the write data will be ignored and the WRITELBA operation will not be performed.
      // Errors are stored into "diskErrLBA" (see ERRLBA opcode).
      //
      // NOTE: Before a WRITELBA operation at least a SELLBA should be always performed

      if (!diskErrLBA)
      // No previous error, so write a sector 
      {
        if (ioByteCnt == 0)
        // First byte of 512, so do the preparation of the sector write operation 
        {
          diskErrLBA = disk_writep(0, sectorLBA);         // Initiate a sector write transaction.
                        
          // DEBUG
          #if DEBUG_WRLBA > 0 
            printf("* DEBUG: WRITELBA * sectorLBA = %u - diskErrLBA = %u \n\r", sectorLBA, diskErrLBA);
          #endif
             
        }
        bufferSD[ioByteCnt] = ioData;                     // Store the data into the sector buffer
        if (ioByteCnt >= 511)
        // Reached the last byte of the sector (512 bytes). Write and finalize the sector into the SD
        {
          diskErrLBA = disk_writep(bufferSD, 512);        // Write the sector into SD
          if (!diskErrLBA) diskErrLBA = disk_writep(0, 0);// Finalize the sector write transaction
          ioOpcode = 0xFF;                                // All done. Set ioOpcode = "No operation"
        }
      }
      ioByteCnt++;                                        // Increment the counter of the exchanged data bytes
    break;
    // **************************************************************************************************************************** END

  }
  if ((ioOpcode != 0x0A) && (ioOpcode != 0x0C) && (ioOpcode != 0x13) && (ioOpcode != 0x14)) ioOpcode = 0xFF;  // All done for the single byte write opcodes; *********** NEW
                                                                                                           //  set ioOpcode = "No operation"
}

// ------------------------------------------------------------------------------

void flushBuffer()
// Write the sector buffer contents (512 bytes) to the current "disk file" on SD.
// This action is performed only if the following conditions are met: the "128 bytes sector emulation" is active, 
//  there is a pending deferred write operation, content of the buffer is valid
{
  if (sect128Flag && pendingBuffWrite && !bufferNotValid)
  {
    diskErr = seekSD(buffSect);
    if (!diskErr) diskErr = writeSD(bufferSD, &numWriBytes);  // Write buffer (512 bytes) to SD
    if (!diskErr && (numWriBytes < 512)) diskErr = 19;    // Check if reached an unexpected EOF
    
    // DEBUG
    #if DEBUG_FLUSH > 0 
      printf("\r\n* DEBUG: Flushbuffer * LBA = 0x%02X - diskErr = %u - numWriBytes = %u\n\r", buffSect, diskErr, numWriBytes);
    #endif

    if (!diskErr) diskErr = writeSD(NULL, &numWriBytes);  // Finalize the write operation
    pendingBuffWrite = 0;                                 // Clear the pending write flag
    bufferNotValid = 1;                                   // Set the flag to force an eventually subsequent READSECT opcode to do
                                                          //  a read from SD in any case
  }
}

// ------------------------------------------------------------------------------
//
// EXECUTE READ OPCODE (I/O read port address = 0x00)
//
// ------------------------------------------------------------------------------


inline void execReadOpcode()
{
  switch (ioOpcode)
  // Execute the requested I/O READ Opcode. The 0xFF value is reserved as "No operation".
  {
    case  0x80:
      // USER KEY:      
      //                I/O DATA:    D7 D6 D5 D4 D3 D2 D1 D0
      //                            ---------------------------------------------------------
      //                              0  0  0  0  0  0  0  0    USER Key not pressed
      //                              0  0  0  0  0  0  0  1    USER Key pressed
            
      ioData = !USER_KEY_GetValue();                      // Read USER Key
    break;
    
    case  0x81:
      // GPIOA Read (GPE Option):
      //
      //                I/O DATA:    D7 D6 D5 D4 D3 D2 D1 D0
      //                            ---------------------------------------------------------
      //                             D7 D6 D5 D4 D3 D2 D1 D0    GPIOA value (see MCP23017 datasheet)
      //
      // NOTE: a value 0x00 is forced if the GPE Option is not present
            
      if (moduleGPIO) 
      {
        ioData = I2C2_Read1ByteRegister(GPIOEXP_ADDR, GPIOA_REG);
      }
    break;

    case  0x82:
      // GPIOB Read (GPE Option):
      //
      //                I/O DATA:    D7 D6 D5 D4 D3 D2 D1 D0
      //                            ---------------------------------------------------------
      //                             D7 D6 D5 D4 D3 D2 D1 D0    GPIOB value (see MCP23017 datasheet)
      //
      // NOTE: a value 0x00 is forced if the GPE Option is not present
            
      if (moduleGPIO) 
      {
        ioData = I2C2_Read1ByteRegister(GPIOEXP_ADDR, GPIOB_REG);
      }
    break;
    
    case  0x83:
      // SPP EMULATION
      // GETSPP - read the Status Lines of the SPP Port and the SPP emulation status:
      //
      //                  I/O DATA:  D7 D6 D5 D4 D3 D2 D1 D0
      //                            ---------------------------------------------------------
      //                              0  0  0  0  0  0  0  0    SPP emulation disabled
      //                             D7 D6 D5 D4 D3  0  0  1    SPP emulation enabled
      //
      //                  bit  | SPP Status line 
      //                  ----------------------------------
      //                   D0  | 1 (SPP emulation enabled) 
      //                   D1  | 0 (not used)
      //                   D2  | 0 (not used)
      //                   D3  | ACK (active Low)
      //                   D4  | BUSY (active High)
      //                   D5  | PAPEREND (active High)
      //                   D6  | SELECT (active High)
      //                   D7  | ERROR (active Low)
      //
      // If the SPP mode is enabled read the SPP Status Lines.
      // If the SPP mode is disabled (or the GPE is not installed) a byte of all 0s will be retrivied.
      //
      // NOTE: to use GETSPP the SETSPP opcode should be called first to activate the SPP mode of the GPIO port.
      
      
      if (SPPmode)
      {
        ioData = I2C2_Read1ByteRegister(GPIOEXP_ADDR, GPIOA_REG);       // Read GPIOA (SPP Status Lines)
        ioData = (ioData & 0b11111000) | 0b00000001;      // Set D0 = 1, D1 = D2 = 0
      }
    break;

    case  0x84:
      // DATETIME (Read date/time and temperature from the RTC. Binary values): 
      //                I/O DATA n:  D7 D6 D5 D4 D3 D2 D1 D0
      //                            ---------------------------------------------------------
      //                I/O DATA 0   D7 D6 D5 D4 D3 D2 D1 D0    seconds [0..59]     (1st data byte)
      //                I/O DATA 1   D7 D6 D5 D4 D3 D2 D1 D0    minutes [0..59]
      //                I/O DATA 2   D7 D6 D5 D4 D3 D2 D1 D0    hours   [0..23]
      //                I/O DATA 3   D7 D6 D5 D4 D3 D2 D1 D0    day     [1..31]
      //                I/O DATA 4   D7 D6 D5 D4 D3 D2 D1 D0    month   [1..12]
      //                I/O DATA 5   D7 D6 D5 D4 D3 D2 D1 D0    year    [0..99]
      //                I/O DATA 6   D7 D6 D5 D4 D3 D2 D1 D0    tempC   [-128..127] (7th data byte)
      //
      // NOTE 1: If RTC is not found all read values will be = 0
      // NOTE 2: Overread data (more then 7 bytes read) will be = 0
      // NOTE 3: The temperature (Celsius) is a byte with two complement binary format [-128..127]

      if (foundRTC)
      {
        if (ioByteCnt == 0) readRTC(&seconds, &minutes, &hours, &day, &month, &year, &tempC); // Read from RTC
        if (ioByteCnt < 7)
        // Send date/time (binary values) to Z80 bus
        {
          switch (ioByteCnt)
          {
            case 0: ioData = seconds; break;
            case 1: ioData = minutes; break;
            case 2: ioData = hours; break;
            case 3: ioData = day; break;
            case 4: ioData = month; break;
            case 5: ioData = year; break;
            case 6: ioData = tempC; break;
          }
          ioByteCnt++;
        }
        else ioOpcode = 0xFF;                             // All done. Set ioOpcode = "No operation"
      }
      else ioOpcode = 0xFF;                               // Nothing to do. Set ioOpcode = "No operation"
    break;
    
    case  0x85:
      // DISK EMULATION
      // ERRDISK - read the error code after a SELDISK, SELSECT, SELTRACK, WRITESECT, READSECT, FLUSHBUFF 
      //           or SDMOUNT operation
      //
      //                I/O DATA:    D7 D6 D5 D4 D3 D2 D1 D0
      //                            ---------------------------------------------------------
      //                             D7 D6 D5 D4 D3 D2 D1 D0    DISK error code (binary)
      //
      //
      // Error codes table:
      //
      //    error code    | description
      // ---------------------------------------------------------------------------------------------------
      //        0         |  No error
      //        1         |  DISK_ERR: the function failed due to a hard error in the disk function, 
      //                  |   a wrong FAT structure or an internal error
      //        2         |  NOT_READY: the storage device could not be initialized due to a hard error or 
      //                  |   no medium
      //        3         |  NO_FILE: could not find the file
      //        4         |  NOT_OPENED: the file has not been opened
      //        5         |  NOT_ENABLED: the volume has not been mounted
      //        6         |  NO_FILESYSTEM: there is no valid FAT partition on the drive
      //       16         |  Illegal disk number
      //       17         |  Illegal track number
      //       18         |  Illegal sector number
      //       19         |  Reached an unexpected EOF
      //
      //
      //
      //
      // NOTE 1: ERRDISK code is referred to the previous SELDISK, SELSECT, SELTRACK, WRITESECT or READSECT
      //         operation
      // NOTE 2: Error codes from 0 to 6 come from the PetitFS library implementation
      // NOTE 3: ERRDISK cannot be used to read the resulting error code after a SDMOUNT operation 
      //         (see the SDMOUNT opcode)
             
      ioData = diskErr;
    break;
    
    case  0x86:
      // DISK EMULATION
      // READSECT - read 128/512 data bytes sequentially from the current emulated disk/track/sector:
      //
      //               I/O DATA n:   D7 D6 D5 D4 D3 D2 D1 D0
      //                            ---------------------------------------------------------
      //               I/O DATA 0    D7 D6 D5 D4 D3 D2 D1 D0   <1st Data Bytes> (First byte)
      //
      //                      |               |                         |
      //                      |               |                         |
      //                      |               |                         |            
      //                      |               |                         |
      //
      //           I/O DATA 127/511  D7 D6 D5 D4 D3 D2 D1 D0   <128/512th Data Bytes> (Last byte)
      //
      //
      // Reads the current emulated sector (128/512 bytes) pointed by the current disk/track/sector numbers, 
      //  one data byte each call.
      // Both 512 or 128 bytes sector sizes are supported. Because the physical sector size on the SD is 512 bytes,
      //  if it is selected an emulated sector dimension of 128 bytes a sector de-blocking will be performed transparently.
      // The selection between 128 or 512 bytes sector size is done by IOS at boot time (see <sect128Flag>).
      // When the "128 bytes sector emulation" is active, the de-blocking algorithm optimizes the physical read operations 
      //  from SD, so a physical read is performed only when really needed.
      // All the 128/512 calls must be always performed sequentially to have a READSECT operation correctly done. 
      // If an error occurs during the READSECT operation, all subsequent read data will be = 0.
      // If an error occurs calling any DISK EMULATION opcode immediately before the READSECT 
      //  opcode call, all the read data will be will be = 0 and the READSECT operation will not be performed.
      // Errors during the READSECT operation are stored into "diskErr" (see ERRDISK opcode).
      //
      // NOTE 1: Before a READSECT operation at least a SELTRACK or a SELSECT should be always performed
      // NOTE 2: Remember to open the right "disk file" first using the SELDISK opcode
      // NOTE 3: The "logical sectors" are the 512 bytes sectors inside a "disk file" forming a virtual disk on SD. They are
      //         identified with a 14 bit LBA-like logical sector address created as TTTTTTTTTSSSSS (binary), where
      //         TTTTTTTTT (binary) is the track number and SSSSS (binary) is the emulated 512 bytes sector number.
      // NOTE 4: The "emulated sector" is the sector seen by the CPU, identified by the disk/track/sector numbers, 
      //         and can be 512 or 128 bytes large.
      // NOTE 5: When the "128 bytes sector emulation" is active, a 128 bytes emulated sector is "blocked" inside a 512 bytes 
      //         emulated sector (a 512 bytes emulated sector contains four 128 bytes emulated sectors).
      // NOTE 6: The FLUSHBUFF opcode is internally executed before a read operation to avoid possible 
      //         data loss if the "128 bytes sector emulation" is active and there is a deferred write operation pending.

      if (ioByteCnt == 0)
      // Read operation preparation. First byte of 128/512, so set the right file pointer for the SD to the current 
      //  emulated sector
      {
        if (((sect128Flag && (sectSel < 128)) || (!sect128Flag && (sectSel < 32))) && !diskErr)
        // Sector and track number valid and no previous error. Set the LBA-like logical sector
        {
          if (sect128Flag)
          {
            if (pendingBuffWrite)
            // Flush (write) the buffer to the SD if required 
            {
              
              // DEBUG
              #if DEBUG_FLUSH > 0 
                printf("\n\r* DEBUG: READSECT * flushBuffer\n\r");
              #endif

              flushBuffer();  
            }
            sectSel512 = sectSel / 4;                     // Correction for the "128 bytes sector emulation" if active
          }
          else sectSel512 = sectSel;
          logicSect = (trackSel << 5) | sectSel512;       // Calculate the new current logical sector (512 bytes) number 
                                                          // inside the "disk file" on SD using a 14 bit "disk file" LBA-like 
                                                          //  logical sector address created as TTTTTTTTTSSSSS (binary)
          readSDrequired = (logicSect != buffSect) || bufferNotValid; // Set the flag at 1 if a read from SD is required, 0 otherwise.
                                                          // Note: a change of the current disk sets <bufferNotValid> as invalid, 
                                                          //       so forcing a read from SD
          if (readSDrequired) 
          {
            if (!diskErr) diskErr = seekSD(logicSect);    // Set logical sector if a read operation is required
          }
        }
      }
      if (!diskErr)
      // No previous error (e.g. track or sector), so read a sector
      {
        if (ioByteCnt == 0)
        // First byte of 128/512, so read the logical 512 bytes sector from SD and store it into the buffer (if needed)
        {
          
          // DEBUG
          #if DEBUG_RDSECT > 0 
            printf("\n\r* DEBUG: READSECT * trackSel = %u - sectSel = %u - sectSel512 = %u\n\r", trackSel, sectSel, sectSel512);
          #endif
          
          if (readSDrequired)
          // Read the logical sector (512 bytes) from SD.
          {
            diskErr = readSD(bufferSD, &numReadBytes); 
            buffSect = logicSect;                         // Update the logical sector number currently stored into the buffer
            if (!diskErr && (numReadBytes < 512))
            // Reached an unexpected EOF
            {  
              bufferNotValid = 1;                         // Set the buffer content as invalid
              diskErr = 19;                               // Set the proper error code
            }
            else bufferNotValid = 0;
                        
            // DEBUG
            #if DEBUG_RDSECT > 0 
              printf("* DEBUG: READSECT * LBA = 0x%02X - diskErr = %u - numReadBytes = %u\n\r", logicSect, diskErr, numReadBytes);
            #endif

          }
          if (sect128Flag) 
          // "128 bytes sector emulation" is active, de-blocking required
          // (there are four 128 bytes sectors inside a physical 512 bytes sector)
          { 
            sectBuffIndex = 128 * (sectSel % 4);          // Set the index of the first data byte of the sector buffer
          }
          else
          // Using 512 bytes sectors, de-blocking is not required
          {
            sectBuffIndex = 0;
          }
        }
        if (!diskErr) ioData = bufferSD[sectBuffIndex];   // If no error, set current data byte to be put on the CPU bus
        else bufferNotValid = 1;                          // If an error occurred set the content of the buffer as invalid
      }
      if ((sect128Flag && (ioByteCnt >= 127)) || (!sect128Flag && (ioByteCnt >= 511)))
      // Reached the last byte of the sector (128/512 bytes)
      {
        ioOpcode = 0xFF;                                  // All done. Set ioOpcode = "No operation"
      }
      ioByteCnt++;                                        // Increment the counter of the exchanged data bytes
      sectBuffIndex++;                                    // Increment the index of the current data byte inside the sector buffer
    break;
    
    case  0x87:
      // DISK EMULATION
      // SDMOUNT - mount a volume on SD, returning an error code (binary):
      //
      //                 I/O DATA 0: D7 D6 D5 D4 D3 D2 D1 D0
      //                            ---------------------------------------------------------
      //                             D7 D6 D5 D4 D3 D2 D1 D0    error code (binary)
      //
      //
      //
      // NOTE 1: This opcode is "normally" not used. Only needed if using a virtual disk from a custom program
      //         loaded with sLoad or with the Autoboot mode
      // NOTE 2: For error codes explanation see the ERRDISK opcode
      // NOTE 3: The resulting error code is read as a data byte without the need of the ERRDISK opcode

      diskErr = mountSD(&filesysSD);
      ioData = diskErr;
    break;
    
    case  0x88:
      // DISK EMULATION
      // FLUSHBUFF - flush (write) the sector buffer to SD, returning an error code (binary):
      //
      //                  I/O DATA:  D7 D6 D5 D4 D3 D2 D1 D0
      //                            ---------------------------------------------------------
      //                             D7 D6 D5 D4 D3 D2 D1 D0    error code (binary)
      //
      //
      // Write the sector buffer contents (512 bytes) to the current "disk file" on SD.
      // This action is performed only if the following conditions are met: 
      //  - the "128 bytes sector emulation" is active;
      //  - there is a pending deferred write operation;
      //  - content of the buffer is valid.
      //
      // NOTE 1: This opcode sets the flag <bufferNotValid> to force an eventually subsequent READSECT opcode to do
      //         a physical read from SD in any case
      // NOTE 2: For error codes explanation see the ERRDISK opcode
      // NOTE 3: The resulting error code is read as a data byte without the need of the ERRDISK opcode

      // DEBUG
      #if DEBUG_FLUSHB > 0 
        printf("\n\r* DEBUG: FLUSHBUFF *");
      #endif

      flushBuffer();
      ioData = diskErr; 
    break;
    
    // ***************************************************************************************************************************** NEW
    case  0x89:
      // DISK EMULATION (LBA)
      // READLBA - read 512 data bytes of the current sector (as LBA) sequentially:
      //
      //               I/O DATA n:   D7 D6 D5 D4 D3 D2 D1 D0
      //                            ---------------------------------------------------------
      //               I/O DATA 0    D7 D6 D5 D4 D3 D2 D1 D0   <1st Data Bytes> (First byte)
      //
      //                      |               |                         |
      //                      |               |                         |
      //                      |               |                         |            
      //                      |               |                         |
      //
      //               I/O DATA 511  D7 D6 D5 D4 D3 D2 D1 D0   <512th Data Bytes> (Last byte)
      //
      //
      // Reads the current sector (512 bytes) from the SD one data byte each call.
      // All the 512 calls must be always performed sequentially to have a READLBA operation correctly done. 
      // If an error occurs during the READLBA operation, all subsequent read data will be = 0.
      // If an error occurs calling any DISK EMULATION opcode immediately before the READLBA 
      //  opcode call, all the read data will be will be = 0 and the READLBA operation will not be performed.
      // Errors during the READLBA operation are stored into "diskErrLBA" (see ERRLBA opcode).
      //
      // NOTE: Before a READLBA operation a SELLBA should be always performed to set the LBA

      if (!diskErrLBA)
      // No previous error, so read a sector
      {
        if (ioByteCnt == 0)
        // First byte of 512, so read the 512 bytes sector from SD and store it into the buffer
        {
          diskErrLBA = disk_readp (bufferSD, sectorLBA, 0, 512);
                        
          // DEBUG
          #if DEBUG_READLBA > 0 
            printf("* DEBUG: READLBA * sectorLBA = %u - diskErrLBA = %u \n\r", sectorLBA, diskErrLBA);
          #endif
             
        }
        if (!diskErrLBA) ioData = bufferSD[ioByteCnt];    // If no error, set current data byte to be put on the CPU bus
      }
      if (ioByteCnt >= 511)
      // Reached the last byte of the sector (512 bytes)
      {
        ioOpcode = 0xFF;                                  // All done. Set ioOpcode = "No operation"
      }
      ioByteCnt++;                                        // Increment the counter of the exchanged data bytes
    break;
    // ***************************************************************************************************************************** END
    
    // ***************************************************************************************************************************** NEW
    case  0x90:
      // DISK EMULATION (LBA)
      // ERRLBA - read the error code after a WRITELBA or READLBA operation:
      //
      //                I/O DATA:    D7 D6 D5 D4 D3 D2 D1 D0
      //                            ---------------------------------------------------------
      //                             D7 D6 D5 D4 D3 D2 D1 D0    DISK error code (binary)
      //
      //
      // Error codes table:
      //
      //    error code    | description
      // ---------------------------------------------------------------------------------------------------
      //        0         |  No error
      //        1         |  RES_ERR: disk error. Hard error occurred during the disk read/write operation 
      //                  |   (for write operation only may be the medium is write protected)      
      //        2         |  RES_NOTRDY: the device has not been initialized
      //        3         |  RES_PARERR: invalid parameter
      //
      //
      // NOTE 1: ERRLBA error code is referred to the previous WRITELBA or READLBA operation
      // NOTE 2: ERRLBA error code is cleared with a SELLBA operation

      ioData = diskErrLBA;
    break;
    // ***************************************************************************************************************************** END

  }
  if ((ioOpcode != 0x84) && (ioOpcode != 0x86) && (ioOpcode != 0x89)) ioOpcode = 0xFF;  // All done for the single byte read opcodes; ******************
                                                                                        //  set ioOpcode = "No operation"
}
