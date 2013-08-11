//*****************************************************************************
//
//   Arduino Core
//   Primary Include File for Arduino Development                     Arduino.h
//
//*****************************************************************************

//*****************************************************************************
//                                                              Copyleft Claims
//*****************************************************************************
/*
 *   This software is made available under GNU General Public License.
 */

//*****************************************************************************
//                                                                  Development
//*****************************************************************************
/*
 *   11-07-13  Improved source code format. Programify.
 *             Installed CoreLib datatypes and some useful defines.
 */

#ifndef Arduino_h
#define Arduino_h

//-----------------------------------------------------------------------------
//                                                          Compiler Directives
//-----------------------------------------------------------------------------
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "binary.h"

//#############################################################################
//                                                            Compile in C Mode
//#############################################################################
#ifdef __cplusplus
extern "C" {
#endif

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - -  Manifest Constants
// Function declaration
#define   IN
#define   OUT

// Pin voltage level
#define   HIGH           0x1
#define   LOW            0x0

// Pin operating mode
#define   INPUT          0x0
#define   OUTPUT         0x1
#define   INPUT_PULLUP   0x2

// Boolean
#define   false          0x0
#define   true           0x1

#define   FALSE          0
#define   TRUE           1
#define   INVALID        -1

// Math constants
#define   PI             3.1415926535897932384626433832795
#define   HALF_PI        1.5707963267948966192313216916398
#define   TWO_PI         6.283185307179586476925286766559
#define   DEG_TO_RAD     0.017453292519943295769236907684886
#define   RAD_TO_DEG     57.295779513082320876798154814105

// ?
#define   SERIAL         0x0
#define   DISPLAY        0x1

// Byte order keywords
#define   LSBFIRST       0
#define   MSBFIRST       1

// Bit transition keywords
#define   CHANGE         1
#define   FALLING        2
#define   RISING         3

// ?
#if defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
#    define    DEFAULT        0
#    define    EXTERNAL       1
#    define    INTERNAL       2
#else  
#    if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644P__)
#         define    INTERNAL1V1    2
#         define    INTERNAL2V56   3
#    else
#         define    INTERNAL       3
#    endif
#    define    DEFAULT        1
#    define    EXTERNAL       0
#endif

// ?
#define   NOT_A_PIN      0
#define   NOT_A_PORT     0

// User Pins ?
#ifdef ARDUINO_MAIN
#    define    PA   1
#    define    PB   2
#    define    PC   3
#    define    PD   4
#    define    PE   5
#    define    PF   6
#    define    PG   7
#    define    PH   8
#    define    PJ   10
#    define    PK   11
#    define    PL   12
#endif

#define   NOT_ON_TIMER   0
#define   TIMER0A        1
#define   TIMER0B        2
#define   TIMER1A        3
#define   TIMER1B        4
#define   TIMER2         5
#define   TIMER2A        6
#define   TIMER2B        7
#define   TIMER3A        8
#define   TIMER3B        9
#define   TIMER3C        10
#define   TIMER4A        11
#define   TIMER4B        12
#define   TIMER4C        13
#define   TIMER4D        14	
#define   TIMER5A        15
#define   TIMER5B        16
#define   TIMER5C        17

// Check if stdlib's abs already encountered
#ifdef abs
#    undef abs
#endif

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - Inline Macros
#define   abs(x)                   ((x)>0?(x):-(x))
#define   bit(b)                   (1UL<<(b))
#define   bitClear(value, bit)     ((value)&=~(1UL<<(bit)))
#define   bitRead(value, bit)      (((value)>>(bit))&0x01)
#define   bitSet(value, bit)       ((value)|=(1UL<<(bit)))
#define   bitWrite(value, bit, bitvalue)  (bitvalue?bitSet(value,bit):bitClear(value,bit))
#define   constrain(amt,low,high)  ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define   degrees(rad)             ((rad)*RAD_TO_DEG)
#define   highByte(w)              ((BYTE)((w)>>8))
#define   lowByte(w)               ((BYTE)((w)&0xff))
#define   max(a,b)                 ((a)>(b)?(a):(b))
#define   min(a,b)                 ((a)<(b)?(a):(b))
#define   radians(deg)             ((deg)*DEG_TO_RAD)
#define   round(x)                 ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define   sq(x)                    ((x)*(x))

/*
 *   Get the bit location within the hardware port of the given virtual pin.
 *   This comes from the pins_*.c file for the active board configuration.
 *
 *   These perform slightly better as macros compared to inline functions.
 */
#define   analogInPinToBit(P)      (P)
#define   digitalPinToPort(P)      (pgm_read_byte(digital_pin_to_port_PGM+(P)))
#define   digitalPinToBitMask(P)   (pgm_read_byte(digital_pin_to_bit_mask_PGM+(P)))
#define   digitalPinToTimer(P)     (pgm_read_byte(digital_pin_to_timer_PGM+(P)))
#define   portOutputRegister(P)    ((volatile BYTE *)(pgm_read_word(port_to_output_PGM+(P))))
#define   portInputRegister(P)     ((volatile BYTE *)(pgm_read_word(port_to_input_PGM+(P))))
#define   portModeRegister(P)      ((volatile BYTE *)(pgm_read_word(port_to_mode_PGM+(P))))

// Conversions
#define   clockCyclesPerMicrosecond()   (F_CPU/1000000L)
#define   clockCyclesToMicroseconds(a)  ((a)/clockCyclesPerMicrosecond())
#define   microsecondsToClockCycles(a)  ((a)*clockCyclesPerMicrosecond())

/*
 *   Get the bit location within the hardware port of the given virtual pin.
 *   This comes from the pins_*.c file for the active board configuration.
 */
#define   analogInPinToBit(P)      (P)

// Synonyms
#define   interrupts()             sei()
#define   noInterrupts()           cli()

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  Data Types
// Corelib standard data types
typedef   int16_t             BOOL ;
typedef   uint8_t             BYTE ;
typedef   uint32_t            DWORD ;
typedef   uint64_t            QWORD ;
typedef   int32_t             SDWORD ;
typedef   int64_t             SQWORD ;
typedef   int16_t             SWORD ;
typedef   uint16_t            WORD ;

// Arduino library data types
typedef   uint8_t             boolean ;
typedef   uint8_t             byte ;
///typedef   unsigned int        word ;

//-----------------------------------------------------------------------------
//                                                                    Functions
//-----------------------------------------------------------------------------
int       analogRead          (BYTE) ;
void      analogReference     (BYTE mode) ;
void      analogWrite         (BYTE, int) ;
void      attachInterrupt     (BYTE, void (*)(void), int mode) ;
void      delay               (DWORD) ;
void      delayMicroseconds   (WORD us) ;
void      detachInterrupt     (BYTE) ;
int       digitalRead         (BYTE) ;
void      digitalWrite        (BYTE, BYTE) ;
void      init                (void) ;
void      loop                (void) ;
DWORD     micros              (void) ;
DWORD     millis              (void) ;
void      pinMode             (BYTE, BYTE) ;
DWORD     pulseIn             (BYTE pin, BYTE state, DWORD timeout) ;
void      setup               (void) ;
BYTE      shiftIn             (BYTE dataPin, BYTE clockPin, BYTE bitOrder) ;
void      shiftOut            (BYTE dataPin, BYTE clockPin, BYTE bitOrder, BYTE val) ;

//-----------------------------------------------------------------------------
//                                                                  Global Data
//-----------------------------------------------------------------------------

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -  Composite Data

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - String Literals

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - Externals
extern const BYTE PROGMEM digital_pin_to_port_PGM [] ;
//extern const BYTE PROGMEM digital_pin_to_bit_PGM [] ;
extern const BYTE PROGMEM digital_pin_to_bit_mask_PGM [] ;
extern const BYTE PROGMEM digital_pin_to_timer_PGM [] ;
/*
 *   On the ATmega1280, the addresses of some of the port registers are
 *   greater than 255, so we can't store them in BYTE's.
 */
extern const WORD PROGMEM port_to_mode_PGM [] ;
extern const WORD PROGMEM port_to_input_PGM [] ;
extern const WORD PROGMEM port_to_output_PGM [] ;

#ifdef __cplusplus
} // End of extern "C" block


//#############################################################################
//                                                          Compile in C++ Mode
//#############################################################################

//-----------------------------------------------------------------------------
//                                                          Compiler Directives
//-----------------------------------------------------------------------------
#include "WCharacter.h"
#include "WString.h"
#include "HardwareSerial.h"

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - C++ Inline Macros
#ifdef __cplusplus
#    define    word(...)      makeWord(__VA_ARGS__)
#endif

WORD      makeWord       (WORD w) ;
WORD      makeWord       (byte h, byte l) ;
void      noTone         (BYTE _pin) ;
DWORD     pulseIn        (BYTE pin, BYTE state, DWORD timeout = 1000000L) ;
long      map            (long, long, long, long, long) ;
long      random         (long) ;
long      random         (long, long) ;
void      randomSeed     (WORD) ;
void      tone           (BYTE _pin, WORD frequency, DWORD duration = 0) ;

#endif

#include "pins_arduino.h"


#endif
