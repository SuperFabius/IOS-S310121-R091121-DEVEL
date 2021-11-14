// ****************************************************************************************
/*

DTU - Disk Test Utilisy (LBA) - S131121

Disk test program for the 68k-MBC using the LBA disk access opcodes (SELLBA, 
WRITELBA, READLBA, ERRLBA)



WARNING: write command can destroy data and the FAT structure of the whole SD!



NOTE1: Required SD module installed

NOTE2: Required IOS S310121-R091121 (or newer revisions until stated otherwise) 



Compiled with the gcc cross compiler toolchain for the 68k-MBC
(see https://hackaday.io/project/177988-68k-mbc-a-3-ics-68008-homebrew-computer/details 
and https://hub.docker.com/r/just4fun4just/68k-mbc)

*/
// ****************************************************************************************


            // Currently defined Opcodes for I/O write operations (IOS S310121-R091121):
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
            // Opcode 0x10  SERIAL 2 TX     1
            // Opcode 0x11  SETSPP          1
            // Opcode 0x12  WRSPP           1
            // Opcode 0x13  SELLBA          4 
            // Opcode 0x14  WRITELBA        512  
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
            // Opcode 0x89  READLBA         512 
            // Opcode 0x90  ERRLBA          1 
            // Opcode 0xFF  No operation    1
            //


// ****************************************************************************************


#include <stdio.h>


// Used macros
#define LOWORD(l) ((WORD)(l))
#define HIWORD(l) ((WORD)(((DWORD)(l) >> 16) & 0xFFFF))
#define LOBYTE(w) ((BYTE)(w))
#define HIBYTE(w) ((BYTE)(((WORD)(w) >> 8) & 0xFF))
typedef uint32_t DWORD;                         // DWORD = unsigned 32 bit value
typedef uint16_t WORD;                          // WORD = unsigned 16 bit value
typedef uint8_t BYTE;                           // BYTE = unsigned 8 bit value


// Opcodes definitions
#define IOBASE      0xFFFFC                     // Address base for the I/O ports
#define EXCWR_PORT  IOBASE+0                    // Address of the EXECUTE WRITE OPCODE write port
#define EXCRD_PORT  IOBASE+0                    // Address of the EXECUTE READ OPCODE read port
#define STOPC_PORT  IOBASE+1                    // Address of the STORE OPCODE write port
#define SER1RX_PORT IOBASE+1                    // Address of the SERIAL 1 RX read port 
#define SYSFLG_PORT IOBASE+2                    // Address of the SYSFLAGS read port
#define SER2RX_PORT IOBASE+3                    // Address of the SERIAL 2 RX read port
#define USRLED_OPC  0x00                        // USER LED opcode
#define SER1TX_OPC  0x01                        // SERIAL 1 TX opcode
#define SER2TX_OPC  0x10                        // SERIAL 2 TX opcode
#define SETSPP_OPC  0x11                        // SETSPP opcode
#define WRSPP_OPC   0x12                        // WRSPP opcode
#define SELLBA_OPC  0x13                        // SELLBA opcode
#define WRTLBA_OPC  0x14                        // WRITELBA opcode
#define USERKEY_OPC 0x80                        // USER KEY opcode
#define GETSPP_OPC  0x83                        // GETSPP opcode
#define DATETIM_OPC 0x84                        // DATETIME opcode
#define SDMOUNT_OPC 0x87                        // SDMOUNT opcode
#define READLBA_OPC 0x89                        // READLBA opcode
#define ERRLBA_OPC  0x90                        // ERRLBA opcode


// Definitions and constants
#define                 ESC_ASCII   27          // ESC key
#define                 BUFFSIZE    512         // Read/write file buffer size
const unsigned int      sectSize = 512;         // Sector size (bytes)


// Global variables
unsigned char           sectBuff[BUFFSIZE];     // Buffer used to store a sector (disk I/O operations)
unsigned char           i, k, inChar, verifyFlag;
unsigned long           sectLBA = 0;            // Current sector number as LBA
unsigned long           jj;
unsigned long           sectCount = 1, currSect;
unsigned int            fillNum, ii;


// Functions definitions
void iosStoreOpc(unsigned char opcode);
void iosExecWriteOpc(unsigned char iodata);
unsigned char iosExecReadOpc(void);
unsigned char iosSysflags(void);
unsigned char iosSer1Rx(void);
unsigned char mountSD();
unsigned char getKey();
unsigned long readNum();
char upperCase(unsigned char c);
char getOneDigit();
void selSectLBA(unsigned long sect);
void readSectLBA(unsigned char * buffer);
void writeSectLBA(unsigned char * buffer);
void hltCPU();
void printErr(unsigned char errCode);
unsigned char errDisk();


// ****************************************************************************************


int main(void)
{

    //
    // Disable buffering on stdout to avoid printf not flushing unless a newline (LF) is sent.
    // For more info: 
    // https://stackoverflow.com/questions/1716296/why-does-printf-not-flush-after-the-call-unless-a-newline-is-in-the-format-strin
    //
    setvbuf(stdout, NULL, _IONBF, 0);

    printf("DTU (Disk Test Utility) for 68k-MBC - S131121\r\n");
    printf("Disk test program using the LBA disk access IOS opcodes (SELLBA, WRITELBA, READLBA, ERRLBA)\r\n");

    // Try to initialize the SD mounting a FAT volume (this is the only FAT virtual disk opcode that can be called 
    //  when using the LBA disk access opcodes)
    mountSD();
    do
    {
        i = mountSD();
        if (i > 0)
        {
            printf("\r\nDisk error. Check the SD and press a key to retry or ESC to ignore >");
            inChar = getKey();
            i = mountSD();
        }
    }
    while ((i > 0) && (inChar != ESC_ASCII));
    printf("\r\n                             * * * WARNING! * * *\n");
    printf("\r* * Write command can destroy data and the FAT structure of the whole SD!  * *\n\r");

    do
    {
        printf("\r\n\nCurrent sector (as LBA) -> %lu", sectLBA);
        printf(" : Sectors to process -> %lu", sectCount);
        printf("\r\n\nCommands list:\n\n");
        printf("\r M: Mount the SD volume\n");
        printf("\r S: Set current sector (LBA)\n");
        printf("\r N: Set how many sectors (LBAs) read or write\n");
        printf("\r R: Read sectors (LBAs)\n");
        printf("\r W: Write sectors (LBAs) filling a value and verify\n");
        printf("\r E: Exit\n\n");
        do
        {
            printf("\rEnter a command [M,S,N,R,W,E] >");
            inChar = getKey();
            inChar = upperCase(inChar);
        }
        while ((inChar != 'M') && (inChar != 'S') && (inChar != 'N') 
               && (inChar != 'R') && (inChar != 'W') && (inChar != 'E'));
        putchar(inChar);
        printf("\n");
        switch  (inChar)
        {
            case 'M':
                mountSD();
                i = mountSD();
                if (!i) printf("\r\nSD volume mounted");
                else printf("\r\nDisk error!");
            break;

            case 'S':
                printf("\r                                                ");
                printf("\rEnter sector number [0..4294967295] >");
                sectLBA = readNum();
            break;
            
            case 'N':
                printf("\r                                                     ");
                printf("\rEnter sectors to process [0..4294967295] >");
                sectCount = readNum();
            break;
                
            case 'R':
                currSect = sectLBA;
                for (jj = 1; jj <= sectCount; jj++)
                {
                    selSectLBA(currSect);
                    readSectLBA(sectBuff);          // Read a sector
                    printf("\n\r* sector -> %lu *\n\r", currSect);
                    
                    // Print a sector
                    for (i = 0; i < 32; i++)
                    // Print 32 rows, 16 values each
                    {
                        for (k = 0; k < 16; k++)
                        // Print a row of 16 values
                        {   
                            ii = k + (16 * i);      // Compute the buffer index
                            printf("%02X ", sectBuff[ii]);
                        }
                        printf("    ");
                        for (k = 0; k < 16; k++)
                        {   
							ii = k + (16 * i);      // Compute the buffer index
                            if ((sectBuff[ii] > 32) && (sectBuff[ii] < 127)) putchar(sectBuff[ii]);
                            else putchar('.');
                        }
                        printf("\n\r");
                    }
                    if (errDisk())
                    {
                        printErr(errDisk());
                        break;
                    }
                    currSect++;
                    if ((sectCount - jj) > 0)
                    // There is more than one sector to read. Check if user wants to abort the read command
                    {
                        if (iosSer1Rx() != 0xff)
                        // A char was typed during the read command
                        {
                            printf("\r\nPress any key to continue or ESC to abort read command >");
                            inChar = getKey();
                            printf("\r\n");
                            if (inChar == ESC_ASCII) break;
                        }
                    }
                }
            break;
            
            case 'W':
                verifyFlag = 1;
                do
                {
                    printf("\r                                            ");
                    printf("\rEnter the value to fill [0..255] >");
                    fillNum = readNum();
                }
                while (fillNum > 255);
                printf("\n\rAre you really sure to proceed [Y/N]? >");
                do inChar = upperCase(getKey());
                while ((inChar != 'Y') && (inChar != 'N'));
                putchar(inChar);
                printf("\n");
                if (inChar != 'Y') break;
                printf("\n");
                currSect = sectLBA;
                for (jj = 1; jj <= sectCount; jj++)
                {
                    printf("\rWriting sector -> %lu\n", currSect);
                    selSectLBA(currSect);
                    for (ii = 0; ii < sectSize; ii++) sectBuff[ii] = fillNum;   // Fill the sector buffer
                    writeSectLBA(sectBuff);         // Write the current sector on disk
                    if (errDisk())
                    {
                        printErr(errDisk());
                        break;
                    }
                    printf("\rVerifing sector -> %lu\n", currSect);
                    selSectLBA(currSect);
                    readSectLBA(sectBuff);          // Read current sector
                    if (errDisk())
                    {
                        printErr(errDisk());
                        break;
                    }
                    for (ii = 0; ii < sectSize; ii++)                           // Verify the sector
                        if (sectBuff[ii] != (unsigned char) fillNum) verifyFlag = 0;
                    if (!verifyFlag)
                    {
                        printf("\r* * * * VERIFY FAILED!!!! * * * *\n");
                        break;
                    }
                    currSect++;
                    if ((sectCount - jj) > 0)
                    // There is more than one sector to write. Check if user wants to abort the write command
                    {
                        if (iosSer1Rx() != 0xff)
                        // A char was typed durung the write command
                        {
                            printf("\r\nPress any key to continue or ESC to abort the write command >");
                            inChar = getKey();
                            printf("\r\n");
                            if (inChar == ESC_ASCII) break;
                        }
                    }
                }
                if (sectCount > 0 && verifyFlag && !(errDisk())) printf("\r\n* * * * VERIFY OK!!! * * * *\n");
            break;
        
            case 'E':
                printf("\r\n\n* Program terminated - CPU halted *");
                hltCPU();
            break;
        }
    }
    while (inChar != 'E');
}


// ---------------------------------------------------------------------------------------------------------------------
//
// IOS low level interface functions
//
// ---------------------------------------------------------------------------------------------------------------------

// IOS Store Opcode
void iosStoreOpc(unsigned char opcode)
{
    // https://gcc.gnu.org/onlinedocs/gcc-4.8.5/gcc.pdf 
    // par. 6.41  Assembler Instructions with C Expression Operands
    asm("move.b %0,(0xFFFFD).l\n\t"        // move.b  <opcode>,(STOPC_PORT).l           ; Write <opcode> to STOPC_PORT
        : : "r" (opcode)); 
}

// IOS Exec Write Opcode
void iosExecWriteOpc(unsigned char iodata)
{
    // https://gcc.gnu.org/onlinedocs/gcc-4.8.5/gcc.pdf 
    // par. 6.41  Assembler Instructions with C Expression Operands
    asm("move.b %0,(0xFFFFC).l\n\t"        // move.b  <iodata>,(EXCWR_PORT).l           ; Write <iodata> to EXCWR_PORT
        : : "r" (iodata)); 
}

// IOS Exec Read Opcode
unsigned char iosExecReadOpc(void)
{
    volatile unsigned char iodata;

    // https://gcc.gnu.org/onlinedocs/gcc-4.8.5/gcc.pdf 
    // par. 6.41  Assembler Instructions with C Expression Operands
    asm("move.b (0xFFFFC).l,%0\n\t"        // move.b  (EXCRD_PORT).l, <iodata>          ; Read <iodata> from EXCRD_PORT
        : "=r" (iodata));
    return iodata;
}

// IOS read SYSFLAGS
unsigned char iosSysflags(void)
{
    volatile unsigned char iodata;

    // https://gcc.gnu.org/onlinedocs/gcc-4.8.5/gcc.pdf 
    // par. 6.41  Assembler Instructions with C Expression Operands
    asm("move.b (0xFFFFE).l,%0\n\t"        // move.b  (SYSFLG_PORT).l, <iodata>         ; Read <iodata> from SYSFLG_PORT
        : "=r" (iodata));
    return iodata;
}

// IOS SERIAL 1 RX
unsigned char iosSer1Rx(void)
{
    volatile unsigned char iodata;

    // https://gcc.gnu.org/onlinedocs/gcc-4.8.5/gcc.pdf 
    // par. 6.41  Assembler Instructions with C Expression Operands
    asm("move.b (0xFFFFD).l,%0\n\t"        // move.b  (SER1RX_PORT).l, <iodata>         ; Read <iodata> from SER1RX_PORT
        : "=r" (iodata));
    return iodata;
}

// IOS SERIAL 2 RX
unsigned char iosSer2Rx(void)
{
    volatile unsigned char iodata;

    // https://gcc.gnu.org/onlinedocs/gcc-4.8.5/gcc.pdf 
    // par. 6.41  Assembler Instructions with C Expression Operands
    asm("move.b (0xFFFFF).l,%0\n\t"        // move.b  (SER2RX_PORT).l, <iodata>         ; Read <iodata> from SER2RX_PORT
        : "=r" (iodata));
    return iodata;
}


// ---------------------------------------------------------------------------------------------------------------------
//
// IOS Opcodes functions (only the ones used here)
//
// ---------------------------------------------------------------------------------------------------------------------


// Set the GPIO port into SPP mode
void setSPP(unsigned char ctrl) 
{
    iosStoreOpc(SETSPP_OPC);
    iosExecWriteOpc(ctrl);
}

unsigned char mountSD()
// Try to mount a volume on SD. Return an error code (0 = no error)
{
    iosStoreOpc(SDMOUNT_OPC);
    return iosExecReadOpc();
}

void selSectLBA(unsigned long sect)
// Select current sector number
{
    iosStoreOpc(SELLBA_OPC);
    iosExecWriteOpc(LOBYTE(LOWORD(sect)));
    iosExecWriteOpc(HIBYTE(LOWORD(sect)));
    iosExecWriteOpc(LOBYTE(HIWORD(sect)));
    iosExecWriteOpc(HIBYTE(HIWORD(sect)));
}

void readSectLBA(unsigned char * buffer)
// Read the current sector (512 bytes long) from SD and write it into the buffer
{
    unsigned int i;
    
    iosStoreOpc(READLBA_OPC);
    for (i = 0; i < BUFFSIZE; i++) buffer[i] = iosExecReadOpc();
}

void writeSectLBA(unsigned char * buffer)
// Write the current sector (512 bytes long) from the buffer to SD
{
    unsigned int i;
    
    iosStoreOpc(WRTLBA_OPC);
    for (i = 0; i < BUFFSIZE; i++) iosExecWriteOpc(buffer[i]);
}

void printErr(unsigned char errCode)
// Print the meaning of an errDisk() error code;
//  "errCode" is the error code from errDisk()
//
// Error codes table:
//
//    error code    | description
// ---------------------------------------------------------------------------------------------------
//        0         |  No error
//        1         |  RES_ERR: hard error occurred during the disk read/write operation 
//                  |   (for write operation only may be the medium is write protected)      
//        2         |  RES_NOTRDY: the device has not been initialized
//        3         |  RES_PARERR: invalid parameter
//

{
    if (errCode != 0)
    {
        printf("\r\n\n* DISK ERROR *\r\n");
        printf("error code %u", errCode);
        switch (errCode)
        {
            case 1:     printf(" (RES_ERR): hardware or internal error"); break;
            case 2:     printf(" (RES_NOTRDY): the device has not been initialized"); break;
            case 3:     printf(" (RES_PARERR): invalid parameter"); break;
            default:    printf(": unknown error"); break;
        }
        printf("\n\r");
    }
}

unsigned char getKey()
// Read a char from input serial port 1 flushing all the previous chars already pending in the input 
//  buffer (if any), waiting for it if required.
{
    unsigned char c;

    do c = iosSer1Rx(); while (c != 0xff);          // Flush buffer if not empty
    do c = iosSer1Rx(); while (c == 0xff);          // Wait a single char
    return c;
}


unsigned char errDisk()
// Read the error code after a readSectLBA(), wrSectLBA() call (0 = no errors)
{
    iosStoreOpc(ERRLBA_OPC);
    return iosExecReadOpc();
}


// ---------------------------------------------------------------------------------------------------------------------
//
// Other functions
//
// ---------------------------------------------------------------------------------------------------------------------


unsigned long readNum()
// Read a decimal 1-10 digit unsigned number in the [0..4.294.967.295] range from the input stream ending with a CR, 
//  and echo it.
// The returned number is an unsigned long.
{
    unsigned char   i, j, inChar;
    unsigned long long   num;

    do
    {
        do inChar = getOneDigit();                  // Read first numeric char [0..9] + CR + BS
        while (inChar == 8);
        if (inChar == 13) return 0;                 // Read a CR, so return 0
        putchar(inChar);                            // Echo it
        num = inChar - 48;                          // Convert first num char into decimal
        for (i = 1; i <= 9; i++)
        // Read next 9 digits
        {
            inChar = getOneDigit();                 // Get a numeric char
            j = 0;
            if (inChar == 8)
            // Read a BACKSPACE, so erase all input
            {
                do
                {
                    putchar(8);
                    putchar(32);
                    putchar(8);
                    j++;
                }
                while (j < i);
                break;
            }
            else putchar(inChar);                   // Echo the current digit
            if (inChar == 13) 
            // Reached last digit
            {
                if (num > 0xffffffff) num = 0xffffffff;     // Avoid value overflow
                return (unsigned long) num;         // Return the number
            }
            num = num * 10;                         // Do a decimal shift
            num = num + (inChar - 48);              // Convert a numeric char
        }
        if (inChar != 8)
        // Wait a CR or BS after the 9th digit
        {
            do inChar = getOneDigit();
            while ((inChar != 13) && (inChar != 8));
            if ((inChar == 8) ) for (j = 0; j < 10; j++)
            // Is a BACKSPACE, so erase all previous input
            {
                putchar(8);
                putchar(32);
                putchar(8);
            }
        }
    }
    while (inChar == 8);
    putchar(inChar);
    if (num > 0xffffffff) num = 0xffffffff;         // Avoid value overflow
    return (unsigned long) num;                     // Return the number
}

char upperCase(unsigned char c)
// Change a character in upper case if it is in [a-z] range
{
    if ((c >96) && (c < 123)) c = c - 32;
    return c;
}

char getOneDigit()
// Read one numeric ASCII char [0..9] or a CR or a BACKSPACE from the input stream. Ignore others chars.
{
    unsigned char   inChar;
    
    do inChar = getKey();
    while (((inChar < 48) || (inChar > 57)) && (inChar != 13) && (inChar != 8));
    return inChar;
}

void hltCPU()
// Halt the CPU and turn on the HALT led.
// To do that we intentionally cause a double exception error loading an illegal address 
//  in the Address Error Exception vector ($0C) and in the Privilege Violation Exception 
//  vector ($20), and than generating a privilege violation exception with the instruction 
//  STOP $0700 that sets to 0 the protected mode bit inside the System Status Register (SSR)
{
    asm("move.l  #1,0x00020.l\n\t"                  // Store an illegal address (any odd number) in the Privilege Exception vector
        "move.l  #1,0x0000c.l\n\t"                  // Store an illegal address (any odd number) in the Address Error Exception vector
        "stop    #0x0700");                         // Halt the CPU with a double exception error)
}
