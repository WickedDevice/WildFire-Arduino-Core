#define FUNC_READ 1
#define FUNC_WRITE 1
/**********************************************************/
/* Optiboot bootloader for Arduino                        */
/*                                                        */
/* http://optiboot.googlecode.com                         */
/*                                                        */
/* Arduino-maintained version : See README.TXT            */
/* http://code.google.com/p/arduino/                      */
/*  It is the intent that changes not relevant to the     */
/*  Arduino production envionment get moved from the      */
/*  optiboot project to the arduino project in "lumps."   */
/*                                                        */
/* Heavily optimised bootloader that is faster and        */
/* smaller than the Arduino standard bootloader           */
/*                                                        */
/* Enhancements:                                          */
/*   Fits in 512 bytes, saving 1.5K of code space         */
/*   Higher baud rate speeds up programming               */
/*   Written almost entirely in C                         */
/*   Customisable timeout with accurate timeconstant      */
/*   Optional virtual UART. No hardware UART required.    */
/*   Optional virtual boot partition for devices without. */
/*                                                        */
/* What you lose:                                         */
/*   Implements a skeleton STK500 protocol which is       */
/*     missing several features including EEPROM          */
/*     programming and non-page-aligned writes            */
/*   High baud rate breaks compatibility with standard    */
/*     Arduino flash settings                             */
/*                                                        */
/* Fully supported:                                       */
/*   ATmega168 based devices  (Diecimila etc)             */
/*   ATmega328P based devices (Duemilanove etc)           */
/*                                                        */
/* Beta test (believed working.)                          */
/*   ATmega8 based devices (Arduino legacy)               */
/*   ATmega328 non-picopower devices                      */
/*   ATmega644P based devices (Sanguino)                  */
/*   ATmega1284P based devices                            */
/*   ATmega1280 based devices (Arduino Mega)              */
/*                                                        */
/* Alpha test                                             */
/*   ATmega32                                             */
/*                                                        */
/* Work in progress:                                      */
/*   ATtiny84 based devices (Luminet)                     */
/*                                                        */
/* Does not support:                                      */
/*   USB based devices (eg. Teensy, Leonardo)             */
/*                                                        */
/* Assumptions:                                           */
/*   The code makes several assumptions that reduce the   */
/*   code size. They are all true after a hardware reset, */
/*   but may not be true if the bootloader is called by   */
/*   other means or on other hardware.                    */
/*     No interrupts can occur                            */
/*     UART and Timer 1 are set to their reset state      */
/*     SP points to RAMEND                                */
/*                                                        */
/* Code builds on code, libraries and optimisations from: */
/*   stk500boot.c          by Jason P. Kyle               */
/*   Arduino bootloader    http://arduino.cc              */
/*   Spiff's 1K bootloader http://spiffie.org/know/arduino_1k_bootloader/bootloader.shtml */
/*   avr-libc project      http://nongnu.org/avr-libc     */
/*   Adaboot               http://www.ladyada.net/library/arduino/bootloader.html */
/*   AVR305                Atmel Application Note         */
/*                                                        */
/* This program is free software; you can redistribute it */
/* and/or modify it under the terms of the GNU General    */
/* Public License as published by the Free Software       */
/* Foundation; either version 2 of the License, or        */
/* (at your option) any later version.                    */
/*                                                        */
/* This program is distributed in the hope that it will   */
/* be useful, but WITHOUT ANY WARRANTY; without even the  */
/* implied warranty of MERCHANTABILITY or FITNESS FOR A   */
/* PARTICULAR PURPOSE.  See the GNU General Public        */
/* License for more details.                              */
/*                                                        */
/* You should have received a copy of the GNU General     */
/* Public License along with this program; if not, write  */
/* to the Free Software Foundation, Inc.,                 */
/* 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA */
/*                                                        */
/* Licence can be viewed at                               */
/* http://www.fsf.org/licenses/gpl.txt                    */
/*                                                        */
/**********************************************************/


/**********************************************************/
/*                                                        */
/* Optional defines:                                      */
/*                                                        */
/**********************************************************/
/*                                                        */
/* BIGBOOT:                                              */
/* Build a 1k bootloader, not 512 bytes. This turns on    */
/* extra functionality.                                   */
/*                                                        */
/* BAUD_RATE:                                             */
/* Set bootloader baud rate.                              */
/*                                                        */
/* SOFT_UART:                                             */
/* Use AVR305 soft-UART instead of hardware UART.         */
/*                                                        */
/* LED_START_FLASHES:                                     */
/* Number of LED flashes on bootup.                       */
/*                                                        */
/* LED_DATA_FLASH:                                        */
/* Flash LED when transferring data. For boards without   */
/* TX or RX LEDs, or for people who like blinky lights.   */
/*                                                        */
/* SUPPORT_EEPROM:                                        */
/* Support reading and writing from EEPROM. This is not   */
/* used by Arduino, so off by default.                    */
/*                                                        */
/* TIMEOUT_MS:                                            */
/* Bootloader timeout period, in milliseconds.            */
/* 500,1000,2000,4000,8000 supported.                     */
/*                                                        */
/* UART:                                                  */
/* UART number (0..n) for devices with more than          */
/* one hardware uart (644P, 1284P, etc)                   */
/*                                                        */
/**********************************************************/

/**********************************************************/
/* Version Numbers!                                       */
/*                                                        */
/* Arduino Optiboot now includes this Version number in   */
/* the source and object code.                            */
/*                                                        */
/* Version 3 was released as zip from the optiboot        */
/*  repository and was distributed with Arduino 0022.     */
/* Version 4 starts with the arduino repository commit    */
/*  that brought the arduino repository up-to-date with   */
/*  the optiboot source tree changes since v3.            */
/* Version 5 was created at the time of the new Makefile  */
/*  structure (Mar, 2013), even though no binaries changed*/
/* It would be good if versions implemented outside the   */
/*  official repository used an out-of-seqeunce version   */
/*  number (like 104.6 if based on based on 4.5) to       */
/*  prevent collisions.                                   */
/*                                                        */
/**********************************************************/

/**********************************************************/
/* Edit History:            */
/*                */
/* Aug 2014              */
/* 6.2 WestfW: make size of length variables dependent    */
/*              on the SPM_PAGESIZE.  This saves space    */
/*              on the chips where it's most important.   */
/* 6.1 WestfW: Fix OPTIBOOT_CUSTOMVER (send it!)    */
/*             Make no-wait mod less picky about    */
/*               skipping the bootloader.      */
/*             Remove some dead code        */
/* Jun 2014              */
/* 6.0 WestfW: Modularize memory read/write functions    */
/*             Remove serial/flash overlap      */
/*              (and all references to NRWWSTART/etc)    */
/*             Correctly handle pagesize > 255bytes       */
/*             Add EEPROM support in BIGBOOT (1284)       */
/*             EEPROM write on small chips now causes err */
/*             Split Makefile into smaller pieces         */
/*             Add Wicked devices Wildfire      */
/*         Move UART=n conditionals into pin_defs.h   */
/*         Remove LUDICOUS_SPEED option      */
/*         Replace inline assembler for .version      */
/*              and add OPTIBOOT_CUSTOMVER for user code  */
/*             Fix LED value for Bobuino (Makefile)       */
/*             Make all functions explicitly inline or    */
/*              noinline, so we fit when using gcc4.8     */
/*             Change optimization options for gcc4.8    */
/*             Make ENV=arduino work in 1.5.x trees.    */
/* May 2014                                               */
/* 5.0 WestfW: Add support for 1Mbps UART                 */
/* Mar 2013                                               */
/* 5.0 WestfW: Major Makefile restructuring.              */
/*             See Makefile and pin_defs.h                */
/*             (no binary changes)                        */
/*                                                        */
/* 4.6 WestfW/Pito: Add ATmega32 support                  */
/* 4.6 WestfW/radoni: Don't set LED_PIN as an output if   */
/*                    not used. (LED_START_FLASHES = 0)   */
/* Jan 2013              */
/* 4.6 WestfW/dkinzer: use autoincrement lpm for read     */
/* 4.6 WestfW/dkinzer: pass reset cause to app in R2      */
/* Mar 2012                                               */
/* 4.5 WestfW: add infrastructure for non-zero UARTS.     */
/* 4.5 WestfW: fix SIGNATURE_2 for m644 (bad in avr-libc) */
/* Jan 2012:                                              */
/* 4.5 WestfW: fix NRWW value for m1284.                  */
/* 4.4 WestfW: use attribute OS_main instead of naked for */
/*             main().  This allows optimizations that we */
/*             count on, which are prohibited in naked    */
/*             functions due to PR42240.  (keeps us less  */
/*             than 512 bytes when compiler is gcc4.5     */
/*             (code from 4.3.2 remains the same.)        */
/* 4.4 WestfW and Maniacbug:  Add m1284 support.  This    */
/*             does not change the 328 binary, so the     */
/*             version number didn't change either. (?)   */
/* June 2011:                                             */
/* 4.4 WestfW: remove automatic soft_uart detect (didn't  */
/*             know what it was doing or why.)  Added a   */
/*             check of the calculated BRG value instead. */
/*             Version stays 4.4; existing binaries are   */
/*             not changed.                               */
/* 4.4 WestfW: add initialization of address to keep      */
/*             the compiler happy.  Change SC'ed targets. */
/*             Return the SW version via READ PARAM       */
/* 4.3 WestfW: catch framing errors in getch(), so that   */
/*             AVRISP works without HW kludges.           */
/*  http://code.google.com/p/arduino/issues/detail?id=368n*/
/* 4.2 WestfW: reduce code size, fix timeouts, change     */
/*             verifySpace to use WDT instead of appstart */
/* 4.1 WestfW: put version number in binary.              */ 
/**********************************************************/
/* Changes made by Victor Aprea at Wicked Device LLC      */ 
/*                                                        */ 
/* Augmented Optiboot to burn from SPI Flash stored image */ 
/* if one is available and valid, and fall back to Serial */
/* bootloading otherwise.                                 */ 
/*                                                        */ 
/* Code relevant to reading/writing the SPIFlash          */ 
/* based heavily on work done by Felix Rusu at Low Power  */ 
/* Labs DualOptiboot project for Moteino.                 */ 
/* https://github.com/LowPowerLab/DualOptiboot            */ 
/*                                                        */ 
/* Assumes a 4Mbit SPI Flash is on SPI bus and CS         */ 
/* to PC2 (as it is on WildFire v3).                      */ 
/*                                                        */ 

#define OPTIBOOT_MAJVER 6
#define OPTIBOOT_MINVER 2

/*
 * OPTIBOOT_CUSTOMVER should be defined (by the makefile) for custom edits
 * of optiboot.  That way you don't wind up with very different code that
 * matches the version number of a "released" optiboot.
 */

#if !defined(OPTIBOOT_CUSTOMVER)
#define OPTIBOOT_CUSTOMVER 0
#endif

unsigned const int __attribute__((section(".version"))) 
optiboot_version = 256*(OPTIBOOT_MAJVER + OPTIBOOT_CUSTOMVER) + OPTIBOOT_MINVER;

#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>

/*
 * Note that we use our own version of "boot.h"
 * <avr/boot.h> uses sts instructions, but this version uses out instructions
 * This saves cycles and program memory.  Sorry for the name overlap.
 */
#include "boot.h"


// We don't use <avr/wdt.h> as those routines have interrupt overhead we don't need.

/*
 * pin_defs.h
 * This contains most of the rather ugly defines that implement our
 * ability to use UART=n and LED=D3, and some avr family bit name differences.
 */
#include "pin_defs.h"

/*
 * stk500.h contains the constant definitions for the stk500v1 comm protocol
 */
#include "stk500.h"

#ifndef LED_START_FLASHES
#define LED_START_FLASHES 0
#endif

/* set the UART baud rate defaults */
#ifndef BAUD_RATE
#if F_CPU >= 8000000L
#define BAUD_RATE   115200L // Highest rate Avrdude win32 will support
#elsif F_CPU >= 1000000L
#define BAUD_RATE   9600L   // 19200 also supported, but with significant error
#elsif F_CPU >= 128000L
#define BAUD_RATE   4800L   // Good for 128kHz internal RC
#else
#define BAUD_RATE 1200L     // Good even at 32768Hz
#endif
#endif

#ifndef UART
#define UART 0
#endif

#define BAUD_SETTING (( (F_CPU + BAUD_RATE * 4L) / ((BAUD_RATE * 8L))) - 1 )
#define BAUD_ACTUAL (F_CPU/(8 * ((BAUD_SETTING)+1)))
#define BAUD_ERROR (( 100*(BAUD_RATE - BAUD_ACTUAL) ) / BAUD_RATE)

#if BAUD_ERROR >= 5
#error BAUD_RATE error greater than 5%
#elif BAUD_ERROR <= -5
#error BAUD_RATE error greater than -5%
#elif BAUD_ERROR >= 2
#warning BAUD_RATE error greater than 2%
#elif BAUD_ERROR <= -2
#warning BAUD_RATE error greater than -2%
#endif

#if (F_CPU + BAUD_RATE * 4L) / (BAUD_RATE * 8L) - 1 > 250
#error Unachievable baud rate (too slow) BAUD_RATE 
#endif // baud rate slow check
#if (F_CPU + BAUD_RATE * 4L) / (BAUD_RATE * 8L) - 1 < 3
#if BAUD_ERROR != 0 // permit high bitrates (ie 1Mbps@16MHz) if error is zero
#error Unachievable baud rate (too fast) BAUD_RATE 
#endif
#endif // baud rate fastn check

/* Watchdog settings */
#define WATCHDOG_OFF    (0)
#define WATCHDOG_16MS   (_BV(WDE))
#define WATCHDOG_32MS   (_BV(WDP0) | _BV(WDE))
#define WATCHDOG_64MS   (_BV(WDP1) | _BV(WDE))
#define WATCHDOG_125MS  (_BV(WDP1) | _BV(WDP0) | _BV(WDE))
#define WATCHDOG_250MS  (_BV(WDP2) | _BV(WDE))
#define WATCHDOG_500MS  (_BV(WDP2) | _BV(WDP0) | _BV(WDE))
#define WATCHDOG_1S     (_BV(WDP2) | _BV(WDP1) | _BV(WDE))
#define WATCHDOG_2S     (_BV(WDP2) | _BV(WDP1) | _BV(WDP0) | _BV(WDE))
#ifndef __AVR_ATmega8__
#define WATCHDOG_4S     (_BV(WDP3) | _BV(WDE))
#define WATCHDOG_8S     (_BV(WDP3) | _BV(WDP0) | _BV(WDE))
#endif


/*
 * We can never load flash with more than 1 page at a time, so we can save
 * some code space on parts with smaller pagesize by using a smaller int.
 */
#if SPM_PAGESIZE > 255
typedef uint16_t pagelen_t ;
#define GETLENGTH(len) len = getch()<<8; len |= getch()
#else
typedef uint8_t pagelen_t;
#define GETLENGTH(len) (void) getch() /* skip high byte */; len = getch()
#endif


/* Function Prototypes
 * The main() function is in init9, which removes the interrupt vector table
 * we don't need. It is also 'OS_main', which means the compiler does not
 * generate any entry or exit code itself (but unlike 'naked', it doesn't
 * supress some compile-time options we want.)
 */

int main(void) __attribute__ ((OS_main)) __attribute__ ((section (".init9")));

void __attribute__((noinline)) putch(char);
uint8_t __attribute__((noinline)) getch(void);
void __attribute__((noinline)) verifySpace();
void __attribute__((noinline)) watchdogConfig(uint8_t x);

static inline void getNch(uint8_t);
static inline void flash_led(uint8_t);
static inline void watchdogReset();
static inline void writebuffer(int8_t memtype, uint8_t *mybuff,
             uint16_t address, pagelen_t len);
static inline void read_mem(uint8_t memtype,
          uint16_t address, pagelen_t len);

#ifdef SOFT_UART
void uartDelay() __attribute__ ((naked));
#endif
void appStart(uint8_t rstFlags) __attribute__ ((naked));

/*
 * RAMSTART should be self-explanatory.  It's bigger on parts with a
 * lot of peripheral registers.  Let 0x100 be the default
 * Note that RAMSTART (for optiboot) need not be exactly at the start of RAM.
 */
#if !defined(RAMSTART)  // newer versions of gcc avr-libc define RAMSTART
#define RAMSTART 0x100
#if defined (__AVR_ATmega644P__)
// correct for a bug in avr-libc
#undef SIGNATURE_2
#define SIGNATURE_2 0x0A
#elif defined(__AVR_ATmega1280__)
#undef RAMSTART
#define RAMSTART (0x200)
#endif
#endif

/* C zero initialises all global variables. However, that requires */
/* These definitions are NOT zero initialised, but that doesn't matter */
/* This allows us to drop the zero init code, saving us memory */
#define buff    ((uint8_t*)(RAMSTART))
#ifdef VIRTUAL_BOOT_PARTITION
#define rstVect (*(uint16_t*)(RAMSTART+SPM_PAGESIZE*2+4))
#define wdtVect (*(uint16_t*)(RAMSTART+SPM_PAGESIZE*2+6))
#endif

/******************* SPI FLASH Code **********************************/
#include <util/crc16.h> // for CRC calculations
#include <util/delay.h> // for led flashes

#define SPI_MODE0 0x00
#define SPI_MODE_MASK 0x0C     // CPOL = bit 3, CPHA = bit 2 on SPCR
#define SPI_CLOCK_MASK 0x03    // SPR1 = bit 1, SPR0 = bit 0 on SPCR
#define SPI_2XCLOCK_MASK 0x01  // SPI2X = bit 0 on SPSR
#define SPI_CLOCK_DIV2 0x04

// the FLASHSS settings here are specific to WildFire v3
#define FLASHSS_DDR     DDRC
#define FLASHSS_PORT    PORTC
#define FLASHSS         PINC2
  
#define FLASH_SELECT   { FLASHSS_PORT &= ~(_BV(FLASHSS)); }
#define FLASH_UNSELECT { FLASHSS_PORT |= _BV(FLASHSS); }

#define SPIFLASH_STATUSWRITE      0x01        // write status register
#define SPIFLASH_STATUSREAD       0x05        // read status register
#define SPIFLASH_WRITEENABLE      0x06        // write enable
#define SPIFLASH_ARRAYREADLOWFREQ 0x03        // read array (low frequency)
#define SPIFLASH_BLOCKERASE_4K    0x20        // erase one 4K block of flash memory
#define SPIFLASH_BLOCKERASE_32K   0x52        // erase one 32K block of flash memory
#define SPIFLASH_BLOCKERASE_64K   0xD8        // erase one 32K block of flash memory
#define SPIFLASH_JEDECID          0x9F        // read JEDEC ID
#define SPIFLASH_BYTEPAGEPROGRAM  0x02        // write (1 to 256bytes)

#define MAGIC_NUMBER              0x0ddba11   // this word at the end of SPI flash
                                              // is a signal to the bootloader to 
                                              // think about loading it
#define MAGIC_NUMBER_ADDRESS      0x7FFFC     // the last 4 bytes are the magic number
#define CRC16_CHECKSUM_ADDRESS    0x7FFFA     // the two bytes before the magic number
                                              // are the expected checksum of the file
#define FILESIZE_ADDRESS          0x7FFF6     // the four bytes before the checksum
                                              // are the stored file size
#define LAST_VALID_BYTE_ADDRESS   126975      // based on 4k reserved for bootloader   
                                              // (124 * 2^10) - 1                                            

#define WILDFIRE_LED_PORT         PORTD
#define WILDFIRE_LED_DDR          DDRD
#define WILDFIRE_LED              6

uint8_t SPI_transfer(uint8_t _data) {
  SPDR = _data;
  while (!(SPSR & _BV(SPIF)));
  return SPDR;
}

uint8_t FLASH_busy()
{
  FLASH_SELECT;
  SPI_transfer(SPIFLASH_STATUSREAD);
  uint8_t status = SPI_transfer(0);
  FLASH_UNSELECT;
  return status & 1;
}

void FLASH_command(uint8_t cmd, uint8_t isWrite){
  if (isWrite)
  {
    FLASH_command(SPIFLASH_WRITEENABLE, 0); // Write Enable
    FLASH_UNSELECT;
  }
  while(FLASH_busy()); //wait for chip to become available
  FLASH_SELECT;
  SPI_transfer(cmd);
}

uint8_t FLASH_readByte(uint32_t addr) {
  FLASH_command(SPIFLASH_ARRAYREADLOWFREQ, 0);
  SPI_transfer(addr >> 16);
  SPI_transfer(addr >> 8);
  SPI_transfer(addr);
  //SPI.transfer(0); //"dont care", needed with SPIFLASH_ARRAYREAD command only
  uint8_t result = SPI_transfer(0);
  FLASH_UNSELECT;
  return result;
}

void add_line_to_page_buffer(uint8_t * data_buffer, uint8_t num_bytes, uint16_t page_offset_address){
  uint8_t ii = 0; 
  uint16_t effective_page_address = page_offset_address;
  for(ii = 0; ii < num_bytes; ii++){
    if(effective_page_address < SPM_PAGESIZE){
      buff[effective_page_address++] = data_buffer[ii];
    }    
  }
}

void flash_wildfire_led(uint8_t num_blinks){
  uint8_t ii = 0;
  for(ii = 0; ii < num_blinks; ii++){
    WILDFIRE_LED_PORT ^= _BV(WILDFIRE_LED);
    _delay_ms(100);    
    WILDFIRE_LED_PORT ^= _BV(WILDFIRE_LED);
    _delay_ms(100);     
  }
}

/*  validates the following:
 *
 *  (1) The crc16 checksum of the first `hex_file_size` bytes in the SPI Flash
 *      match the `expected_crc16_checksum` value
 *  (2) Each line is self consistent with the Intel Hex format
 *      (a) starts with a ':' character
 *      (b) the bytes between ':' and "\r\n" are all valid hex characters
 *      (c) the length of each line is as expected based on the length field
 *      (d) the last byte address implied by the record is in-bounds
 *          for the application code space (aaaa + ll)
 *      (e) the record type is in: '00', '01', or '02'
 *      (f) the checksum for the record matches the calculated checksum
 *  (3) The number of data bytes is even (AVR instructions are 16-bits wide)
 *  
 *  if also_burn_flash != 0, then also burns the data progressively
 *
 */
uint8_t validate_image(uint32_t hex_file_size, uint16_t expected_crc16_checksum, uint8_t also_burn_flash){  
  uint16_t crc = 0;
  uint8_t line_buffer[32] = {0}; // these are the parsed values from this line
                                 // converted into integer equivalents in pairs
                                 // for every pair between the colon and '\r\n'
  uint8_t *pData = &(line_buffer[4]); // data always starts with 5th parsed byte
  uint8_t line_buffer_idx = 0;
  uint8_t even_char = 0;
  uint8_t odd_char = 0;        
  
  uint8_t num_data_bytes = 0;
  uint8_t line_length = 0; 
  uint8_t record_type = 0;
  uint16_t line_address = 0;
  uint8_t ii = 0;
  int32_t current_flash_page = -1;
  int32_t last_flash_page_written = -1;
  
  #define VALIDATE_STATE_EXPECT_COLON        0
  #define VALIDATE_STATE_EVEN_HEX_CHAR       1
  #define VALIDATE_STATE_ODD_HEX_CHAR        2
  #define VALIDATE_STATE_EXPECT_NEWLINE      3
  
  uint8_t state = VALIDATE_STATE_EXPECT_COLON;
  uint32_t extended_address = 0;
  uint32_t num_nonempty_data_records_processed = 0;
  uint32_t spiflash_address = 0;
  uint16_t flash_start_page_of_line = 0;
  uint16_t flash_end_page_of_line   = 0;
  uint32_t last_flash_address_on_start_page = 0;
  uint32_t first_flash_address_on_start_page = 0;
  uint32_t first_flash_address_on_current_page = 0;
  uint8_t num_data_bytes_on_first_page = 0;
  uint8_t num_data_bytes_on_second_page = 0;   
  uint32_t flash_start_address_of_line = 0;
  uint32_t flash_end_address_of_line = 0;     
  uint16_t flash_start_address_of_line_within_page = 0;   
  uint8_t calculated_checksum = 0;
  uint8_t burned_flash = 0; 
  char dataByte = 0;
  
  for(spiflash_address = 0; spiflash_address < hex_file_size; spiflash_address++){
    dataByte = FLASH_readByte(spiflash_address);
    crc = _crc16_update(crc, dataByte); // the crc is over the ascii file text
    
    // while validating , blink the LED on and off every so often to indicte progress
    if((spiflash_address & 0x1FFF) == 0) WILDFIRE_LED_PORT ^= _BV(WILDFIRE_LED);
    
    switch(state){
    case VALIDATE_STATE_EXPECT_COLON:
      if(dataByte != ':') return 0;
      line_buffer_idx = 0;
      line_length = 0;
      state = VALIDATE_STATE_EVEN_HEX_CHAR;
      break;
    case VALIDATE_STATE_EXPECT_NEWLINE:
      if(dataByte != '\n') return 0;
      state = VALIDATE_STATE_EXPECT_COLON;
      
      num_data_bytes = line_buffer[0];
      
      // analyze the line for semantic validation problems
      line_length = num_data_bytes + 5; // one line-length byte
                                        // two address bytes
                                        // one record type byte
                                        // one checksum byte
                                        // #data_bytes         
      
      // the number of stored bytes should equal line_length
      if(line_buffer_idx != line_length) return 0;
      
      line_address = line_buffer[1];
      line_address <<= 8;
      line_address |= line_buffer[2];        
      record_type  = line_buffer[3];
      
      calculated_checksum = 0;
      // calculate the checksum of the line bytes, they should sum to zero
      for(ii = 0; ii < line_buffer_idx; ii++){
        calculated_checksum += line_buffer[ii];
      }        
      
      if(calculated_checksum != 0) return 0; // by definition

      flash_start_address_of_line = 0;
      flash_end_address_of_line = 0;     
      flash_start_address_of_line_within_page = 0;
           
      switch(record_type){
      case 0:  // data record
        // flash_start_address_of_line drives all the rest of the calculations
        flash_start_address_of_line = extended_address + line_address;                
        flash_end_address_of_line = flash_start_address_of_line + line_buffer[0] - 1;
        flash_start_address_of_line_within_page = flash_start_address_of_line % SPM_PAGESIZE;      
        
        // flash_end_address_of_line must be in bounds
        if(flash_end_address_of_line > LAST_VALID_BYTE_ADDRESS) return 0;        
        
        if(also_burn_flash){              
          // this is the point at which we need to do more work *if* flash is to be burned
          // write bytes to the page buffer
          // handle page boundary conditions, if this line contains bytes that cross a page boundary
          flash_start_page_of_line = flash_start_address_of_line / SPM_PAGESIZE;
          flash_end_page_of_line   = flash_end_address_of_line   / SPM_PAGESIZE;                   
          last_flash_address_on_start_page = ((uint32_t) flash_start_page_of_line + 1) * SPM_PAGESIZE - 1;  
          first_flash_address_on_start_page = (uint32_t) flash_start_page_of_line * SPM_PAGESIZE;
          first_flash_address_on_current_page = current_flash_page * SPM_PAGESIZE;
          num_data_bytes_on_first_page = last_flash_address_on_start_page - flash_start_address_of_line + 1;
          num_data_bytes_on_second_page = num_data_bytes - num_data_bytes_on_first_page;          
          
          // handle the first record like this
          if(current_flash_page < 0) current_flash_page = flash_start_page_of_line;
          
          if(line_buffer[0] != 0) num_nonempty_data_records_processed++;        
        
          // four cases, 
          //    1. current_flash_page == flash_start_page_of_line == flash_end_page_of_line
          //    2. current_flash_page == flash_start_page_of_line != flash_end_page_of_line
          //    3. flash_start_page_of_line == flash_end_page_of_line != current_flash_page
          //    4. flash_start_page_of_line != flash_end_page_of_line != current_flash_page  
          //
          //    case 1: this line is contained within the current working page
          //            - add its contents to the page buffer
          //    case 2: part of this line is contained within the current working page
          //            - add that part of the line's contents to the page buffer
          //            - write the page buffer to flash
          //            - reset the page buffer 
          //            - and add the remainder of the line to the page buffer
          //    case 3: this line is part of a single totally new page
          //            - write the page buffer to flash
          //            - reset the page buffer
          //            - add the entire line to the page buffer
          //    case 4: this line spans two totally new pages (omg is this really possible!?)
          //            - write the page buffer to flash
          //            - reset the page buffer
          //            - add the first part of the line to the page buffer
          //            - write the page buffer to flash
          //            - reset the page buffer
          //            - add the second part of the line to the page buffer
          
          burned_flash = 0;                       
          if(flash_start_page_of_line == current_flash_page){
            if(flash_start_page_of_line == flash_end_page_of_line){
              // case 1: this line is contained within the current working page
              add_line_to_page_buffer(pData, num_data_bytes, flash_start_address_of_line_within_page);
            }
            else{ // flash_start_page_of_line != flash_end_page_of_line
              // case 2: part of this line is contained within the current working page
              burned_flash = 1;
              
              // add that part of the line's contents to the page buffer              
              add_line_to_page_buffer(pData, num_data_bytes_on_first_page, 
                flash_start_address_of_line_within_page);
              
              // write the page buffer to flash
              RAMPZ = (first_flash_address_on_start_page + SPM_PAGESIZE - 1 > 0xFFFF) ? 1 : 0;
              writebuffer(0, buff, first_flash_address_on_start_page, SPM_PAGESIZE);

              // and add the remainder of the line to the page buffer
              // since we crossed a page boundary, it *must* start at offset zero
              add_line_to_page_buffer(pData + num_data_bytes_on_first_page, 
                num_data_bytes_on_second_page, 0); 
            }
          }
          else{ // flash_start_page_of_line != current_flash_page
            if(flash_start_page_of_line == flash_end_page_of_line){
              // case 3: this line is part of a single totally new page
              burned_flash = 1;
              
              // write the page buffer to flash
              RAMPZ = (first_flash_address_on_current_page + SPM_PAGESIZE - 1 > 0xFFFF) ? 1 : 0;
              writebuffer(0, buff, first_flash_address_on_current_page, SPM_PAGESIZE);
 
              // add the entire line to the page buffer
              add_line_to_page_buffer(pData, num_data_bytes, flash_start_address_of_line_within_page);           
            }
            else{ // flash_start_page_of_line != flash_end_page_of_line
              // case 4: this line spans two totally new pages (omg is this really possible!?)
              burned_flash = 1;
              
              // write the page buffer to flash
              RAMPZ = (first_flash_address_on_current_page + SPM_PAGESIZE - 1 > 0xFFFF) ? 1 : 0;
              writebuffer(0, buff, first_flash_address_on_current_page, SPM_PAGESIZE);

              // add the first part of the line to the page buffer
              add_line_to_page_buffer(pData, num_data_bytes_on_first_page, 
                flash_start_address_of_line_within_page);
              
              // write the page buffer to flash
              RAMPZ = (first_flash_address_on_start_page + SPM_PAGESIZE - 1 > 0xFFFF) ? 1 : 0;
              writebuffer(0, buff, first_flash_address_on_start_page, SPM_PAGESIZE);

              // add the second part of the line to the page buffer
              // since we crossed a page boundary, it *must* start at offset zero
              add_line_to_page_buffer(pData + num_data_bytes_on_first_page, 
                num_data_bytes_on_second_page, 0); 
            }
          }
          
          if(burned_flash){
            last_flash_page_written = current_flash_page; 
            current_flash_page = flash_end_page_of_line;            
            WILDFIRE_LED_PORT ^= _BV(WILDFIRE_LED);
          }
        }            
                   
        break;
      case 1:  // end of file record
          // at this point we still have to write the page buffer to flash
          // if the page buffer has never yet been written to flash and at least
          //   one valid data record was processed,
          // or if the current flash page differs from the last one written
          // then we need to write the page buffer to flash
          if(((last_flash_page_written == -1) && (num_nonempty_data_records_processed > 0)) || 
             (last_flash_page_written != current_flash_page)){
             
             first_flash_address_on_current_page = current_flash_page * SPM_PAGESIZE;
             RAMPZ = (first_flash_address_on_current_page + SPM_PAGESIZE - 1 > 0xFFFF) ? 1 : 0;
             writebuffer(0, buff, first_flash_address_on_current_page, SPM_PAGESIZE);             
          }
        break;
      case 2:  // extended segment address record 
        if(line_address != 0) return 0; // by definition
        
        extended_address = line_buffer[4];
        extended_address <<= 8;
        extended_address |= line_buffer[5];
        extended_address <<= 4; // extended address is bits 4..19
        break;      
      case 4: // extended linear address record
        if(line_address != 0) return 0; // by definition
        
        extended_address = line_buffer[4];
        extended_address <<= 8;
        extended_address |= line_buffer[5];
        extended_address <<= 16; // extended address is bits 16..31
        break;    
      default: // we should never see anything else 
        return 0;
        break;
      }
      
      break;        
    case VALIDATE_STATE_EVEN_HEX_CHAR: // i.e. d in 'd3'        
      if(dataByte <= '9' && dataByte >='0'){
        even_char = dataByte - '0';
        state = VALIDATE_STATE_ODD_HEX_CHAR;
      }
      else if(dataByte <= 'F' && dataByte >='A'){
        even_char = dataByte - 'A' + 10;
        state = VALIDATE_STATE_ODD_HEX_CHAR;
      }
      else if(dataByte == '\r'){
        state = VALIDATE_STATE_EXPECT_NEWLINE;
      }
      else{
        return 0;
      }
      break;
    case VALIDATE_STATE_ODD_HEX_CHAR:  // i.e. 3 in 'd3'
      if(dataByte <= '9' && dataByte >='0'){
        odd_char = dataByte - '0';
        state = VALIDATE_STATE_EVEN_HEX_CHAR;
      }
      else if(dataByte <= 'F' && dataByte >='A'){
        odd_char = dataByte - 'A' + 10;
        state = VALIDATE_STATE_EVEN_HEX_CHAR;
      }
      else{
        return 0;
      }    
      
      // interpret the newly acquired pair as a byte value 
      // and store it in line_buffer for the sake of local checksum-ing later
      if(line_buffer_idx < 32){ // guard against buffer overflow
        line_buffer[line_buffer_idx++] = (even_char << 4) + odd_char;          
      }
      else{
        return 0;
      }
      
      break;
    default:
      return 0; // this should never be reached or the state machine is broken
      break;
    }    
  }
  
  // finally verify that the calculated checksum matches the expected checksum
  // for the entire flash image
  if(crc != expected_crc16_checksum) return 0;
  
  return 1;
}

void CheckFlashImage() {
  watchdogConfig(WATCHDOG_OFF);

  // set up the SPI hardware - ATmega1284P specific code here
  PORTB |= _BV(PB4);                       // set SS HIGH
  DDRB  |= _BV(PB4) | _BV(PB5) | _BV(PB7); // OUTPUTS for SS, MOSI, SCK

  DDRC  |= _BV(FLASHSS);                   // OUTPUT for FLASH_SS
  FLASH_UNSELECT;                          // unselect FLASH chip

  WILDFIRE_LED_DDR |= _BV(WILDFIRE_LED);   // make the LED pin for WildFire an output
  WILDFIRE_LED_PORT &= ~_BV(WILDFIRE_LED); // turn the WildFire LED Off

  SPCR |= _BV(MSTR) | _BV(SPE); //enable SPI and set SPI to MASTER mode

  // check if the SPI Flash is present at all
  //    read first byte of JEDECID
  //    if chip is present it should return a non-0 and non-FF value
  FLASH_SELECT;
  SPI_transfer(SPIFLASH_JEDECID);
  uint8_t deviceId = SPI_transfer(0);
  FLASH_UNSELECT;
  if ((deviceId == 0) || (deviceId == 0xFF)) return;

  // global unprotect  
  FLASH_command(SPIFLASH_STATUSWRITE, 1);
  SPI_transfer(0);
  FLASH_UNSELECT;  

  // first check if the magic number is located in the last byte of Flash
  uint32_t magic_number = FLASH_readByte(MAGIC_NUMBER_ADDRESS);
  magic_number <<= 8;
  magic_number |= FLASH_readByte(MAGIC_NUMBER_ADDRESS + 1);
  magic_number <<= 8;
  magic_number |= FLASH_readByte(MAGIC_NUMBER_ADDRESS + 2);
  magic_number <<= 8;
  magic_number |= FLASH_readByte(MAGIC_NUMBER_ADDRESS + 3);  
  if(magic_number != MAGIC_NUMBER) return;

  flash_wildfire_led(3);
  
  WILDFIRE_LED_PORT ^= _BV(WILDFIRE_LED);
  _delay_ms(100);    
  
  // if it is... grab the file size and the checksum out of the Flash
  uint32_t hex_file_size = FLASH_readByte(FILESIZE_ADDRESS);
  hex_file_size <<= 8;
  hex_file_size |= FLASH_readByte(FILESIZE_ADDRESS + 1);
  hex_file_size <<= 8;
  hex_file_size |= FLASH_readByte(FILESIZE_ADDRESS + 2);
  hex_file_size <<= 8;
  hex_file_size |= FLASH_readByte(FILESIZE_ADDRESS + 3);    
  
  uint16_t expected_crc16_checksum = FLASH_readByte(CRC16_CHECKSUM_ADDRESS);
  expected_crc16_checksum <<= 8;
  expected_crc16_checksum |= FLASH_readByte(CRC16_CHECKSUM_ADDRESS + 1);
  
  // use the filesize and checksum to validate the file contained in Flash
  // also validate that none of the addresses in the hex records are out of
  // range for the bootloader size while we're at it, but don't write flash yet
  if(validate_image(hex_file_size, expected_crc16_checksum, 0)){
    // if the file is deemed valid 
    // then burn the file contained in SPI flash to the MCU flash
    if(validate_image(hex_file_size, expected_crc16_checksum, 1)){
      WILDFIRE_LED_PORT &= ~_BV(WILDFIRE_LED); // turn off the WildFire LED
      
      // Wipe out the Magic Number so that next reset we don't reburn the flash
      // Don't just erase the sector, because it's useful for the application
      // to know the signature of the last loaded file by reading out of Flash 
      FLASH_command(SPIFLASH_BYTEPAGEPROGRAM, 1);  // Byte/Page Program
      SPI_transfer((MAGIC_NUMBER_ADDRESS + 1) >> 16);
      SPI_transfer((MAGIC_NUMBER_ADDRESS + 1) >> 8);
      SPI_transfer((MAGIC_NUMBER_ADDRESS + 1));
      SPI_transfer(0x00);   // it's enough to just corrupt the second byte
      FLASH_UNSELECT;          
      
      watchdogConfig(WATCHDOG_16MS);    // shorten WD timeout
      while (1)        // and busy-loop so that WD causes
        ;              // a reset and app start.    
    }    
  } 
}
/******************* END SPI FLASH Code ****************************/

/* main program starts here */
int main(void) {
  uint8_t ch;

  /*
   * Making these local and in registers prevents the need for initializing
   * them, and also saves space because code no longer stores to memory.
   * (initializing address keeps the compiler happy, but isn't really
   *  necessary, and uses 4 bytes of flash.)
   */
  register uint16_t address = 0;
  register pagelen_t  length;

  // After the zero init loop, this is the first code to run.
  //
  // This code makes the following assumptions:
  //  No interrupts will execute
  //  SP points to RAMEND
  //  r1 contains zero
  //
  // If not, uncomment the following instructions:
  // cli();
  asm volatile ("clr __zero_reg__");
#if defined(__AVR_ATmega8__) || defined (__AVR_ATmega32__)
  SP=RAMEND;  // This is done by hardware reset
#endif

  /*
   * modified Adaboot no-wait mod.
   * Pass the reset reason to app.  Also, it appears that an Uno poweron
   * can leave multiple reset flags set; we only want the bootloader to
   * run on an 'external reset only' status
   */
  ch = MCUSR;
  MCUSR = 0;
  if (ch & (_BV(WDRF) | _BV(BORF) | _BV(PORF)))
      appStart(ch);

  CheckFlashImage();
  WILDFIRE_LED_PORT &= ~_BV(WILDFIRE_LED); // turn off the WildFire LED

#if LED_START_FLASHES > 0
  // Set up Timer 1 for timeout counter
  TCCR1B = _BV(CS12) | _BV(CS10); // div 1024
#endif

#ifndef SOFT_UART
#if defined(__AVR_ATmega8__) || defined (__AVR_ATmega32__)
  UCSRA = _BV(U2X); //Double speed mode USART
  UCSRB = _BV(RXEN) | _BV(TXEN);  // enable Rx & Tx
  UCSRC = _BV(URSEL) | _BV(UCSZ1) | _BV(UCSZ0);  // config USART; 8N1
  UBRRL = (uint8_t)( (F_CPU + BAUD_RATE * 4L) / (BAUD_RATE * 8L) - 1 );
#else
  UART_SRA = _BV(U2X0); //Double speed mode USART0
  UART_SRB = _BV(RXEN0) | _BV(TXEN0);
  UART_SRC = _BV(UCSZ00) | _BV(UCSZ01);
  UART_SRL = (uint8_t)( (F_CPU + BAUD_RATE * 4L) / (BAUD_RATE * 8L) - 1 );
#endif
#endif

  // Set up watchdog to trigger after 500ms
  watchdogConfig(WATCHDOG_1S);

#if (LED_START_FLASHES > 0) || defined(LED_DATA_FLASH)
  /* Set LED pin as output */
  LED_DDR |= _BV(LED);
#endif

#ifdef SOFT_UART
  /* Set TX pin as output */
  UART_DDR |= _BV(UART_TX_BIT);
#endif

#if LED_START_FLASHES > 0
  /* Flash onboard LED to signal entering of bootloader */
  flash_led(LED_START_FLASHES * 2);
#endif

  /* Forever loop: exits by causing WDT reset */
  for (;;) {
    /* get character from UART */
    ch = getch();

    if(ch == STK_GET_PARAMETER) {
      unsigned char which = getch();
      verifySpace();
      /*
       * Send optiboot version as "SW version"
       * Note that the references to memory are optimized away.
       */
      if (which == 0x82) {
    putch(optiboot_version & 0xFF);
      } else if (which == 0x81) {
    putch(optiboot_version >> 8);
      } else {
  /*
   * GET PARAMETER returns a generic 0x03 reply for
         * other parameters - enough to keep Avrdude happy
   */
  putch(0x03);
      }
    }
    else if(ch == STK_SET_DEVICE) {
      // SET DEVICE is ignored
      getNch(20);
    }
    else if(ch == STK_SET_DEVICE_EXT) {
      // SET DEVICE EXT is ignored
      getNch(5);
    }
    else if(ch == STK_LOAD_ADDRESS) {
      // LOAD ADDRESS
      uint16_t newAddress;
      newAddress = getch();
      newAddress = (newAddress & 0xff) | (getch() << 8);
#ifdef RAMPZ
      // Transfer top bit to RAMPZ
      RAMPZ = (newAddress & 0x8000) ? 1 : 0;
#endif
      newAddress += newAddress; // Convert from word address to byte address
      address = newAddress;
      verifySpace();
    }
    else if(ch == STK_UNIVERSAL) {
      // UNIVERSAL command is ignored
      getNch(4);
      putch(0x00);
    }
    /* Write memory, length is big endian and is in bytes */
    else if(ch == STK_PROG_PAGE) {
      // PROGRAM PAGE - we support flash programming only, not EEPROM
      uint8_t desttype;
      uint8_t *bufPtr;
      pagelen_t savelength;

      GETLENGTH(length);
      savelength = length;
      desttype = getch();

      // read a page worth of contents
      bufPtr = buff;
      do *bufPtr++ = getch();
      while (--length);

      // Read command terminator, start reply
      verifySpace();

#ifdef VIRTUAL_BOOT_PARTITION
      if ((uint16_t)(void*)address == 0) {
        // This is the reset vector page. We need to live-patch the code so the
        // bootloader runs.
        //
        // Move RESET vector to WDT vector
        uint16_t vect = buff[0] | (buff[1]<<8);
        rstVect = vect;
        wdtVect = buff[8] | (buff[9]<<8);
        vect -= 4; // Instruction is a relative jump (rjmp), so recalculate.
        buff[8] = vect & 0xff;
        buff[9] = vect >> 8;

        // Add jump to bootloader at RESET vector
        buff[0] = 0x7f;
        buff[1] = 0xce; // rjmp 0x1d00 instruction
      }
#endif

      writebuffer(desttype, buff, address, savelength);


    }
    /* Read memory block mode, length is big endian.  */
    else if(ch == STK_READ_PAGE) {
      uint8_t desttype;
      GETLENGTH(length);

      desttype = getch();

      verifySpace();
    
      read_mem(desttype, address, length);
    }

    /* Get device signature bytes  */
    else if(ch == STK_READ_SIGN) {
      // READ SIGN - return what Avrdude wants to hear
      verifySpace();
      putch(SIGNATURE_0);
      putch(SIGNATURE_1);
      putch(SIGNATURE_2);
    }
    else if (ch == STK_LEAVE_PROGMODE) { /* 'Q' */
      // Adaboot no-wait mod
      watchdogConfig(WATCHDOG_16MS);
      verifySpace();
    }
    else {
      // This covers the response to commands like STK_ENTER_PROGMODE
      verifySpace();
    }
    putch(STK_OK);
  }
}

void putch(char ch) {
#ifndef SOFT_UART
  while (!(UART_SRA & _BV(UDRE0)));
  UART_UDR = ch;
#else
  __asm__ __volatile__ (
    "   com %[ch]\n" // ones complement, carry set
    "   sec\n"
    "1: brcc 2f\n"
    "   cbi %[uartPort],%[uartBit]\n"
    "   rjmp 3f\n"
    "2: sbi %[uartPort],%[uartBit]\n"
    "   nop\n"
    "3: rcall uartDelay\n"
    "   rcall uartDelay\n"
    "   lsr %[ch]\n"
    "   dec %[bitcnt]\n"
    "   brne 1b\n"
    :
    :
      [bitcnt] "d" (10),
      [ch] "r" (ch),
      [uartPort] "I" (_SFR_IO_ADDR(UART_PORT)),
      [uartBit] "I" (UART_TX_BIT)
    :
      "r25"
  );
#endif
}

uint8_t getch(void) {
  uint8_t ch;

#ifdef LED_DATA_FLASH
#if defined(__AVR_ATmega8__) || defined (__AVR_ATmega32__)
  LED_PORT ^= _BV(LED);
#else
  LED_PIN |= _BV(LED);
#endif
#endif

#ifdef SOFT_UART
  __asm__ __volatile__ (
    "1: sbic  %[uartPin],%[uartBit]\n"  // Wait for start edge
    "   rjmp  1b\n"
    "   rcall uartDelay\n"          // Get to middle of start bit
    "2: rcall uartDelay\n"              // Wait 1 bit period
    "   rcall uartDelay\n"              // Wait 1 bit period
    "   clc\n"
    "   sbic  %[uartPin],%[uartBit]\n"
    "   sec\n"
    "   dec   %[bitCnt]\n"
    "   breq  3f\n"
    "   ror   %[ch]\n"
    "   rjmp  2b\n"
    "3:\n"
    :
      [ch] "=r" (ch)
    :
      [bitCnt] "d" (9),
      [uartPin] "I" (_SFR_IO_ADDR(UART_PIN)),
      [uartBit] "I" (UART_RX_BIT)
    :
      "r25"
);
#else
  while(!(UART_SRA & _BV(RXC0)))
    ;
  if (!(UART_SRA & _BV(FE0))) {
      /*
       * A Framing Error indicates (probably) that something is talking
       * to us at the wrong bit rate.  Assume that this is because it
       * expects to be talking to the application, and DON'T reset the
       * watchdog.  This should cause the bootloader to abort and run
       * the application "soon", if it keeps happening.  (Note that we
       * don't care that an invalid char is returned...)
       */
    watchdogReset();
  }
  
  ch = UART_UDR;
#endif

#ifdef LED_DATA_FLASH
#if defined(__AVR_ATmega8__) || defined (__AVR_ATmega32__)
  LED_PORT ^= _BV(LED);
#else
  LED_PIN |= _BV(LED);
#endif
#endif

  return ch;
}

#ifdef SOFT_UART
// AVR305 equation: #define UART_B_VALUE (((F_CPU/BAUD_RATE)-23)/6)
// Adding 3 to numerator simulates nearest rounding for more accurate baud rates
#define UART_B_VALUE (((F_CPU/BAUD_RATE)-20)/6)
#if UART_B_VALUE > 255
#error Baud rate too slow for soft UART
#endif

void uartDelay() {
  __asm__ __volatile__ (
    "ldi r25,%[count]\n"
    "1:dec r25\n"
    "brne 1b\n"
    "ret\n"
    ::[count] "M" (UART_B_VALUE)
  );
}
#endif

void getNch(uint8_t count) {
  do getch(); while (--count);
  verifySpace();
}

void verifySpace() {
  if (getch() != CRC_EOP) {
    watchdogConfig(WATCHDOG_16MS);    // shorten WD timeout
    while (1)            // and busy-loop so that WD causes
      ;              //  a reset and app start.
  }
  putch(STK_INSYNC);
}

#if LED_START_FLASHES > 0
void flash_led(uint8_t count) {
  do {
    TCNT1 = -(F_CPU/(1024*16));
    TIFR1 = _BV(TOV1);
    while(!(TIFR1 & _BV(TOV1)));
#if defined(__AVR_ATmega8__)  || defined (__AVR_ATmega32__)
    LED_PORT ^= _BV(LED);
#else
    LED_PIN |= _BV(LED);
#endif
    watchdogReset();
  } while (--count);
}
#endif

// Watchdog functions. These are only safe with interrupts turned off.
void watchdogReset() {
  __asm__ __volatile__ (
    "wdr\n"
  );
}

void watchdogConfig(uint8_t x) {
  WDTCSR = _BV(WDCE) | _BV(WDE);
  WDTCSR = x;
}

void appStart(uint8_t rstFlags) {
  // save the reset flags in the designated register
  //  This can be saved in a main program by putting code in .init0 (which
  //  executes before normal c init code) to save R2 to a global variable.
  __asm__ __volatile__ ("mov r2, %0\n" :: "r" (rstFlags));

  watchdogConfig(WATCHDOG_OFF);
  __asm__ __volatile__ (
#ifdef VIRTUAL_BOOT_PARTITION
    // Jump to WDT vector
    "ldi r30,4\n"
    "clr r31\n"
#else
    // Jump to RST vector
    "clr r30\n"
    "clr r31\n"
#endif
    "ijmp\n"
  );
}

/*
 * void writebuffer(memtype, buffer, address, length)
 */
static inline void writebuffer(int8_t memtype, uint8_t *mybuff,
             uint16_t address, pagelen_t len)
{
    switch (memtype) {
    case 'E': // EEPROM
#if defined(SUPPORT_EEPROM) || defined(BIGBOOT)
        while(len--) {
          eeprom_write_byte((uint8_t *)(address++), *mybuff++);
        }
#else
  /*
   * On systems where EEPROM write is not supported, just busy-loop
   * until the WDT expires, which will eventually cause an error on
   * host system (which is what it should do.)
   */
  while (1)
      ; // Error: wait for WDT
#endif
  break;
    default:  // FLASH
  /*
   * Default to writing to Flash program memory.  By making this
   * the default rather than checking for the correct code, we save
   * space on chips that don't support any other memory types.
   */
    {
        // Copy buffer into programming buffer
        uint8_t *bufPtr = mybuff;
        uint16_t addrPtr = (uint16_t)(void*)address;

        /*
         * Start the page erase and wait for it to finish.  There
         * used to be code to do this while receiving the data over
         * the serial link, but the performance improvement was slight,
         * and we needed the space back.
         */
        __boot_page_erase_short((uint16_t)(void*)address);
        boot_spm_busy_wait();

        /*
         * Copy data from the buffer into the flash write buffer.
         */
        do {
          uint16_t a;
          a = *bufPtr++;
          a |= (*bufPtr++) << 8;
          __boot_page_fill_short((uint16_t)(void*)addrPtr,a);
          addrPtr += 2;
        } while (len -= 2);

        /*
         * Actually Write the buffer to flash (and wait for it to finish.)
         */
        __boot_page_write_short((uint16_t)(void*)address);
        boot_spm_busy_wait();
  #if defined(RWWSRE)
        // Reenable read access to flash
        boot_rww_enable();
  #endif
    } // default block
    break;
  } // switch
}

static inline void read_mem(uint8_t memtype, uint16_t address, pagelen_t length)
{
    uint8_t ch;

    switch (memtype) {

#if defined(SUPPORT_EEPROM) || defined(BIGBOOT)
    case 'E': // EEPROM
      do {
          putch(eeprom_read_byte((uint8_t *)(address++)));
      } while (--length);
      break;
#endif
    default:
      do {
#ifdef VIRTUAL_BOOT_PARTITION
        // Undo vector patch in bottom page so verify passes
        if (address == 0)       ch=rstVect & 0xff;
        else if (address == 1)  ch=rstVect >> 8;
        else if (address == 8)  ch=wdtVect & 0xff;
        else if (address == 9) ch=wdtVect >> 8;
        else ch = pgm_read_byte_near(address);
        address++;
#elif defined(RAMPZ)
        // Since RAMPZ should already be set, we need to use EPLM directly.
        // Also, we can use the autoincrement version of lpm to update "address"
        //      do putch(pgm_read_byte_near(address++));
        //      while (--length);
        // read a Flash and increment the address (may increment RAMPZ)
        __asm__ ("elpm %0,Z+\n" : "=r" (ch), "=z" (address): "1" (address));
#else
        // read a Flash byte and increment the address
        __asm__ ("lpm %0,Z+\n" : "=r" (ch), "=z" (address): "1" (address));
#endif
        putch(ch);
      } while (--length);
      break;
    } // switch
}
