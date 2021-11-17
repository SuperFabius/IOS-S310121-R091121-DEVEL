;==================================================================================
;
; 68k-MBC - HW ref: A091020-R140221 (or following HW revisions unless stated otherwise)
;
; Demo using CPU interrupt on char available on serial 1 and 2 and the systick. Chars received from serial 1 
;  are echoed to serial 1 tx, chars received from serial 2 are echoed to serial 2 tx while USER led blinks 
;  using the systick IRQ.
;
;
; REQUIRED: IOS S310121-R091121 (or newer revisions until stated otherwise) 
;
; Assemble with the Easy68K editor/assembler.
;
;
;
;
; NOTE 1: When using the CPU interrupt the IOS opcode operations must be treated as atomic 
;         operations, so CPU interrupts must be disabled before and re-enabled after
;  
; NOTE 2: This demo can run only on the Full HW configuration option board, as the IRQ are required
;         (Lite HW configuration option doesn't support CPU IRQ)
;
; NOTE 3: Note how the IO-OP led behavior is changed when IRQ are enabled. Now the IO-OP led
;         is normally off as no polling is done, and blinks only when an IRQ occurs (and a 
;         consequent I/O is done).
;
; Easy68K can be downloaded here: 
; http://web.archive.org/web/20200113112413/http://www.easy68k.com/files/SetupEASy68K.exe
;
;==================================================================================

; IOS equates
IOBASE      EQU     $FFFFC              ; Address base for the I/O ports
EXCWR_PORT  EQU     IOBASE+0            ; Address of the EXECUTE WRITE OPCODE write port
EXCRD_PORT  EQU     IOBASE+0            ; Address of the EXECUTE READ OPCODE read port
STOPC_PORT  EQU     IOBASE+1            ; Address of the STORE OPCODE write port
SER1RX_PORT EQU     IOBASE+1            ; Address of the SERIAL 1 RX read port 
SYSFLG_PORT EQU     IOBASE+2            ; Address of the SYSFLAGS read port
SER2RX_PORT EQU     IOBASE+3            ; Address of the SERIAL 2 RX read port
USRLED_OPC  EQU     $00                 ; USER LED opcode
SER1TX_OPC  EQU     $01                 ; SERIAL 1 TX opcode
SETIRQ_OPC  EQU     $02                 ; SETIRQ opcode
SELDISK_OPC EQU     $09                 ; SELDISK opcode
SELTRCK_OPC EQU     $0A                 ; SELTRACK opcode
SELSECT_OPC EQU     $0B                 ; SELSECT opcode
WRTSECT_OPC EQU     $0C                 ; WRITESECT opcode
SER2TX_OPC  EQU     $10                 ; SERIAL 2 TX opcode
ERRDSK_OPC  EQU     $85                 ; ERRDISK opcode
RDSECT_OPC  EQU     $86                 ; READSECT opcode
SDMOUNT_OPC EQU     $87                 ; SDMOUNT opcode
FLUSHBF_OPC EQU     $88                 ; FLUSHBUFF opcode

; Common ASCII codes
cr          EQU     $0d                 ; Carriage return
lf          EQU     $0a                 ; Line feed
eos         EQU     0                   ; End of string


            ORG     $40*4               ; Vector 0x40 (Serial 1 Rx ISR vector)
            DC.L    Rx1ISR
            
            ORG     $41*4               ; Vector 0x41 (Serial 2 Rx ISR vector)
            DC.L    Rx2ISR
            
            ORG     $42*4               ; Vector 0x42 (Systick ISR vector)
            DC.L    tickISR

            ORG     $0400

    ;
    ; Serial 1 Rx ISR
    ;
Rx1ISR:     
    move.l  d0,-(sp)                    ; Save D0
    move.b  (SER1RX_PORT).l,Rx1Char     ; Yes, read it and save
    move.b  #1,Rx1Flag                  ; Set the valid char flag
Rx1ISRend:    
    move.l  (sp)+,d0                    ; Restore D0
    rte
    
    ;
    ; Serial 2 Rx ISR
    ;
Rx2ISR:     
    move.l  d0,-(sp)                    ; Save D0
    move.b  (SER2RX_PORT).l,Rx2Char     ; Yes, read it and save
    move.b  #1,Rx2Flag                  ; Set the valid char flag
Rx2ISRend:    
    move.l  (sp)+,d0                    ; Restore D0
    rte

    ;
    ; Systick ISR (default systick is 5ms)
    ; Blink the USER led with a 2*0.5s period
    ;
tickISR:
    sub.w   #1,tickCount
    beq     toggleLed                   ; Jump if time is elapsed
    rte
    
toggleLed:
    move.w  #100,tickCount              ; Reload the counter
    not.b   ledStatus                   ; Change the User led status
    move.b  #USRLED_OPC,(STOPC_PORT).l  ; Write USER LED opcode to IOS
    move.b  ledStatus,(EXCWR_PORT).l    ; Toggle User led
    rte

    ;
    ; Start of program
    ;        
start:
    ; NOTE: the SSP is already set at $1000 by the IOS load routine
    move.b  (SYSFLG_PORT).l,d0          ; D0.B = SYSFLAGS IOS register
    btst    #3,d0                       ; Lite HW configuration detected? (bit3 = 1?)
    bne     LiteHW                      ; Yes, jump
    lea     Msg2,a1                     ; No, Print a message
    bsr     puts1
    andi    #$a01f,sr                   ; Enable CPU IRQ (I2 = I1 = I0 = 0)
    lea     Msg4,a1                     ; Print a message
    bsr     puts1
    move.b  #SETIRQ_OPC,(STOPC_PORT).l  ; Write SETIRQ opcode to IOS
    move.b  #7,(EXCWR_PORT).l           ; Enable IPL1 activation from IOS on serial 1 and 2 Rx + systick
    lea     Msg3,a1                     ; Print a message on Serial 1
    bsr     puts1
    lea     CRLF,a1
    bsr     puts2
    lea     Msg3,a1                     ; Print a message on Serial 2
    bsr     puts2
    lea     Msg1,a1                     ; Print a message on Serial 1 and the prompt
    bsr     puts1
    lea     Msg1,a1                     ; Print a message on Serial 2 and the prompt
    bsr     puts2
loop:
    tst.b   Rx1Flag
    beq     ckcRx2                      ; Jump if no char is present
    bsr     getc1                       ; Read or a char from Rx 1
    bsr     putc1                       ; Print it on tx 1
ckcRx2:
    tst.b   Rx2Flag
    beq     loop                        ; Jump if no char is present
    bsr     getc2                       ; Read or a char from Rx 2
    bsr     putc2                       ; Print it on tx 2

    jmp     loop                        ; For ever...
    
LiteHW:
    lea     Msg5,a1                     ; Print a message...
    bsr     puts1
    bsr     abort                       ; ...and abort
    
; =========================================================================== ;
;
; Send a string to the serial port 1, A1 = pointer to the string.
; NOTE: Only D0.B and A1 are used
;
; =========================================================================== ;
puts1:
    move.b  (a1)+,d0                    ; D0.B = current char to print
    cmp.b   #eos,d0                     ; Is it an eos?
    beq     puts1_end                   ; Yes, jump
    bsr     putc1                       ; No, print it
    bra     puts1
puts1_end:
    rts
    
; =========================================================================== ;
;
; Send a string to the serial port 2, A1 = pointer to the string.
; NOTE: Only D0.B and A1 are used
;
; =========================================================================== ;
puts2:
    move.b  (a1)+,d0                    ; D0.B = current char to print
    cmp.b   #eos,d0                     ; Is it an eos?
    beq     puts2_end                   ; Yes, jump
    bsr     putc2                       ; No, print it
    bra     puts2
puts2_end:
    rts
    
; =========================================================================== ;
;
; Send a char to the serial port 1, D0.B = char to send.
;
; =========================================================================== ;
putc1:
    ori     #$0700,sr                   ; Disable CPU IRQ (I2 = I1 = I0 = 1)
    move.b  #SER1TX_OPC,(STOPC_PORT).l  ; Write SERIAL 1 TX opcode to IOS
    move.b  d0,(EXCWR_PORT).l           ; Print current char
    andi    #$a01f,sr                   ; Re-enable CPU IRQ (I2 = I1 = I0 = 0)
    rts
    
; =========================================================================== ;
;
; Read a char from the serial port 1 or wait for it.
; D0.B returns the char.
;
; =========================================================================== ;
getc1:
    tst.b   Rx1Flag
    beq     getc1                       ; Jump if no char is present
    ori     #$0700,sr                   ; Disable CPU IRQ (I2 = I1 = I0 = 1)
    move.b  Rx1Char,d0                  ; D0.B = char
    clr.b   Rx1Flag                     ; Clear the flag
    andi    #$a01f,sr                   ; Re-enable CPU IRQ (I2 = I1 = I0 = 0)
    rts

; =========================================================================== ;
;
; Send a char to the serial port 2, D0.B = char to send.
;
; =========================================================================== ;
putc2:
    ori     #$0700,sr                   ; Disable CPU IRQ (I2 = I1 = I0 = 1)
    move.b  #SER2TX_OPC,(STOPC_PORT).l  ; Write SERIAL 2 TX opcode to IOS
    move.b  d0,(EXCWR_PORT).l           ; Print current char
    andi    #$a01f,sr                   ; Re-enable CPU IRQ (I2 = I1 = I0 = 0)
    rts
    
; =========================================================================== ;
;
; Read a char from the serial port 2 or wait for it.
; D0.B returns the char.
;
; =========================================================================== ;
getc2:
    tst.b   Rx2Flag
    beq     getc2                       ; Jump if no char is present
    ori     #$0700,sr                   ; Disable CPU IRQ (I2 = I1 = I0 = 1)
    move.b  Rx2Char,d0                  ; D0.B = char
    clr.b   Rx2Flag                     ; Clear the flag
    andi    #$a01f,sr                   ; Re-enable CPU IRQ (I2 = I1 = I0 = 0)
    rts

; =========================================================================== ;
;
; Abort the demo.
; Prints a message and halt.
;
; =========================================================================== ;
abort:
    lea     Msg6,a1
    bsr     puts1
    ;
    ; Halt the CPU and set the HALT output pin active.
    ; To do that we intentionally cause a double exception error loading an illegal address 
    ; in the Address Error Exception vector ($0C) and in the Privilege Violation Exception 
    ; vector ($20), and than generating a privilege violation exception with the instruction 
    ; STOP $0700 that sets to 0 the protected mode bit inside the System Status Register (SSR)
    ;
    move.l  #1,$20.l                    ; Store an illegal address (any odd number) in the Privilege Exception vector
    move.l  #1,$c.l                     ; Store an illegal address (any odd number) in the Address Error Exception vector 
    stop    #$0700                      ; Halt the CPU with a double exception error


Msg1        DC.B    'Echo test (with IRQ) using the serial port 1 and 2', cr, lf, '>', eos
Rx1Char     DC.B    0
Rx2Char     DC.B    0
tickCount   DC.W    100                 ; Systick counter
ledStatus   DC.B    0                   ; User led status ( 0 = off, 1 = on)
CRLF        DC.B    cr, lf, eos
Msg2        DC.B    'Full HW option detected', cr, lf, eos
Msg3        DC.B    'IOS IRQ enabled', cr, lf, eos
Msg4        DC.B    'CPU IRQ enabled', cr, lf, eos
Msg5        DC.B    'Lite HW option detected - Cannot execute here!', cr, lf, eos
Msg6        DC.B    ' * CPU halted * ', cr, lf, eos
Rx1Flag     DC.B    0                   ; 0 = no char in Rx1Char 
Rx2Flag     DC.B    0                   ; 0 = no char in Rx2Char 

            END     start


