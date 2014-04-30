//
//
// Pins definitions for Wildfire 1284P
// Original file from Maniacbug's Mighty-1284 core
// This version by Victor Aprea (22nd September 2013)
//
//

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>

//                    ATMEL ATMEGA1284P on Wildfire
//                    Actual implementation with TQFP
//
//                               +---\/---+
//            PCINT8 (D14) PB0  1|        |40  PA0 (AI 0 / D24) PCINT0
//            PCINT9 (D15) PB1  2|        |39  PA1 (AI 1 / D25) PCINT1
//      PCINT10/INT2 (D 8) PB2  3|        |38  PA2 (AI 2 / D26) PCINT2
//      PCINT11/OC0A*(D 9) PB3  4|        |37  PA3 (AI 3 / D27) PCINT3
//   PCINT12/0C0B/SS*(D10) PB4  5|        |36  PA4 (AI 4 / D28) PCINT4
//      PCINT13/MOSI (D11) PB5  6|        |35  PA5 (AI 5 / D29) PCINT5
// PCINT14/OC3A/MISO*(D12) PB6  7|        |34  PA6 (AI 6 / D30) PCINT6
//  PCINT15/OC3B/SCK*(D13) PB7  8|        |33  PA7 (AI 7 / D31) PCINT7
//                         RST  9|        |32  AREF
//                         VCC 10|        |31  GND 
//                         GND 11|        |30  AVCC
//                       XTAL2 12|        |29  PC7 (D23) TOSC2/PCINT23
//                       XTAL1 13|        |28  PC6 (D22) TOSC1/PCINT22
//       PCINT24/RX0 (D 0) PD0 14|        |27  PC5 (D21) TDI/PCINT21
//       PCINT25/TX0 (D 1) PD1 15|        |26  PC4 (D20) TDO/PCINT20
//  PCINT26/INT0/RX1 (D 2) PD2 16|        |25  PC3 (D19) TMS/PCINT19
//  PCINT27/INT1/TX1 (D 3) PD3 17|        |24  PC2 (D18) TCK/PCINT18
//      PCINT28/OC1B*(D 4) PD4 18|        |23  PC1 (D17) SDA/PCINT17
//      PCINT29/OC1A*(D 5) PD5 19|        |22  PC0 (D16) SCL/PCINT16
//      PCINT30/OC2B*(D 6) PD6 20|        |21  PD7 (D 7)*OC2A/PCINT31
//                               +--------+
//
// D 3 dedicated to CC3000 Enable
// D 4 dedicated to SD Card Socket
// D 9 dedicated to CC3000 IRQ
// D10 dedicated to CC3000 SPI Slave Select
//
// * = PWM capable pin
// TOSCn = RTC Crystal pinout
// TCK/TMS/TDO/TDI = JTAG pinout

/*                       
                         PCICR
   PCINT7-0:   D31-24  : bit 0   
   PCINT15-8:  D15- 8  : bit 1
   PCINT23-16: D23-16  : bit 2
   PCINT31-24: D 7- 0  : bit 3
*/

#define NUM_DIGITAL_PINS            32
#define NUM_ANALOG_INPUTS           8

#define analogInputToDigitalPin(p)  ((p < 8) ? (p) + 24 : -1)
#define digitalPinHasPWM(p)         (((p) > 3) && ((p) < 14) && ((p) != 7) && ((p) != 11))

static const uint8_t SS   = 9;
static const uint8_t MOSI = 11;
static const uint8_t MISO = 12;
static const uint8_t SCK  = 13;

static const uint8_t SDA = 19;
static const uint8_t SCL = 20;
static const uint8_t LED_BUILTIN = 6;

static const uint8_t A0 = 24;
static const uint8_t A1 = 25;
static const uint8_t A2 = 26;
static const uint8_t A3 = 27;
static const uint8_t A4 = 28;
static const uint8_t A5 = 29;
static const uint8_t A6 = 30;
static const uint8_t A7 = 31;

#define digitalPinToPCICR(p)    (((p) >= 0 && (p) < NUM_DIGITAL_PINS) ? (&PCICR) : ((uint8_t *)0))
#define digitalPinToPCICRbit(p) ((p) > 23 ? 0 : (((p) > 13 && (p) < 21) || ((p) == 7) ? 2 : (((p) > 10) || ((p) == 9) || ((p) == 4) ? 1 : 3)))
#define digitalPinToPCMSK(p)    ((p) < 32 ? ((p) > 23 ? (&PCMSK0) : (((p) > 13 && (p) < 21) || ((p) == 7) ? (&PCMSK2) : (((p) > 10) || ((p) == 9) || ((p) == 4) ? (&PCMSK1) : (&PCMSK3)))) : ((uint8_t *)0))

#ifndef ARDUINO_MAIN
extern const uint8_t PROGMEM digital_pin_to_PCMSK_bit_PGM;
#endif

#define digitalPinToPCMSKbit(p) (pgm_read_byte(digital_pin_to_PCMSK_bit_PGM + p))

#ifdef ARDUINO_MAIN
#define PA 1
#define PB 2
#define PC 3
#define PD 4

// this was just too complicated to express as a formula, and we have plenty of flash memory 
const uint8_t PROGMEM digital_pin_to_PCMSK_bit_PGM[] = {
  0,
  1,
  2,
  3,
  3,
  5,
  6,
  3,
  4,
  4,
  7,
  5,
  6,
  7,
  4,
  2,
  5,
  6,
  7,
  1,
  0,
  0,
  2,
  1,
  0,
  1,
  2,
  3,
  4,
  5,
  6,
  7
};

// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)
const uint16_t PROGMEM port_to_mode_PGM[] =
{
	NOT_A_PORT,
	(uint16_t) &DDRA,
	(uint16_t) &DDRB,
	(uint16_t) &DDRC,
	(uint16_t) &DDRD,
};

const uint16_t PROGMEM port_to_output_PGM[] =
{
	NOT_A_PORT,
	(uint16_t) &PORTA,
	(uint16_t) &PORTB,
	(uint16_t) &PORTC,
	(uint16_t) &PORTD,
};

const uint16_t PROGMEM port_to_input_PGM[] =
{
	NOT_A_PORT,
	(uint16_t) &PINA,
	(uint16_t) &PINB,
	(uint16_t) &PINC,
	(uint16_t) &PIND,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] =
{
  PD, 
  PD,
  PD,
  PD,
  PB,
  PD,
  PD,
  PC,
  
  PD, 
  PB,
  PD,
  PB,
  PB,
  PB,
  PC,
  PC,
  
  PC,  
  PC,
  PC,
  PC,
  PC,
  PB,
  PB,
  PB,
  
  PA, 
  PA,
  PA,
  PA,
  PA,
  PA,
  PA,
  PA
};


const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] =
{
  _BV(0), // PD
  _BV(1), // PD
  _BV(2), // PD
  _BV(3), // PD
  _BV(3), // PB
  _BV(5), // PD
  _BV(6), // PD
  _BV(3), // PC
  _BV(4), // PD
  _BV(4), // PB
  _BV(7), // PD
  _BV(5), // PB
  _BV(6), // PB
  _BV(7), // PB
  _BV(4), // PC
  _BV(2), // PC
  _BV(5), // PC
  _BV(6), // PC
  _BV(7), // PC
  _BV(1), // PC
  _BV(0), // PC
  _BV(0), // PB
  _BV(2), // PB
  _BV(1), // PB
  _BV(0), // PA
  _BV(1), // PA
  _BV(2), // PA
  _BV(3), // PA
  _BV(4), // PA
  _BV(5), // PA
  _BV(6), // PA
  _BV(7)  // PA
};

const uint8_t PROGMEM digital_pin_to_timer_PGM[] =
{
  NOT_ON_TIMER, // PD0
  NOT_ON_TIMER, // PD1
  NOT_ON_TIMER, // PD2
  NOT_ON_TIMER, // PD3
  TIMER0A,      // PB3
  TIMER1A,      // PD5
  TIMER2B,      // PD6
  NOT_ON_TIMER, // PC3

  TIMER1B,      // PD4
  TIMER0B,      // PB4
  TIMER2A,      // PD7
  NOT_ON_TIMER, // PB5
  TIMER3A,      // PB6
  TIMER3B,      // PB7
  NOT_ON_TIMER, // PC4
  NOT_ON_TIMER, // PC2

  NOT_ON_TIMER, // PC5
  NOT_ON_TIMER, // PC6
  NOT_ON_TIMER, // PC7
  NOT_ON_TIMER, // PC1
  NOT_ON_TIMER, // PC0
  NOT_ON_TIMER, // PB0
  NOT_ON_TIMER, // PB2
  NOT_ON_TIMER, // PB1

  NOT_ON_TIMER, // PA0
  NOT_ON_TIMER, // PA1
  NOT_ON_TIMER, // PA2
  NOT_ON_TIMER, // PA3
  NOT_ON_TIMER, // PA4
  NOT_ON_TIMER, // PA5
  NOT_ON_TIMER, // PA6
  NOT_ON_TIMER, // PA7
};

#endif // ARDUINO_MAIN

#endif // Pins_Arduino_h
// vim:ai:cin:sts=2 sw=2 ft=cpp
