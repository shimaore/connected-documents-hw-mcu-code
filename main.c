/* Functionalities:
*
* - PWM control of motor via L293D;
* - control SPI serial Flash;
* - mini Forth interpreter;
* - patterns are played realtime from serial flash to motor.
*
* - patterns are downloaded from BT to serial flash (via Forth code)
* - interface with Bluetooth module, e.g. HM-5 or HM-10 (in Forth code)
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

typedef uint8_t byte;
typedef byte* pointer;

typedef uint8_t bool;
enum { false = 0, true = 1 };

/* With a 512Ko flash we need at least 3 octets per address */
typedef uint32_t word;
typedef word  external_pointer;

/* Flash designations in different platform versions
* v0.0.1 (test): Spansion S25FL204K0TMFI011
*/
#define FLASH_SIZE 0x00080000   /* 524/288 bytes */

/* RAM designations in different platform versions
* v0.0.1 (test): MicroChip 23K256-I/SN
*/
#define RAM_SIZE   0x00008000   /* 32768 bytes */

/* Plarform version. */
#define VERSION 0x00000100

/* This is for a 16MHz clock. */
#define F_CPU 16000000

void usart_9600() {
#undef BAUD
#define BAUD 9600
#include <util/setbaud.h>
UBRRH = UBRRH_VALUE;
UBRRL = UBRRL_VALUE;
#if USE_2X
UCSRA |= (1 << U2X);
#else
UCSRA &= ~(1 << U2X);
#endif
}

void usart_38400() {
#undef BAUD  // avoid compiler warning
#define BAUD 38400
#include <util/setbaud.h>
UBRRH = UBRRH_VALUE;
UBRRL = UBRRL_VALUE;
#if USE_2X
UCSRA |= (1 << U2X);
#else
UCSRA &= ~(1 << U2X);
#endif
}

/* Part of the address is used as target select. */

enum {
  ADDR_FLASH = 0x10000000,
  ADDR_RAM   = 0x20000000
};

/* Forth Interrupt Block layout */

enum {
  interrupt_reset = 0 | ADDR_FLASH,
  interrupt_usart = 4 | ADDR_FLASH,
};

/* Run Forth reset when AVR resets. */

word interrupt = interrupt_reset;

/* Bluetooth interface uses the USART in RS-232 mode. */

/* Defaults:
 * - BC-04 uses 9600/1 stop/no parity
 * - HC-05 uses 9600/1 stop/no parity
 * - HM-10 uses 
 */

void init_usart() {
  UCSRB = (1<<RXCIE)|(1<<RXEN)|(1<<TXEN);
  UCSRC = (3<<UCSZ0); // 8 bits
  usart_9600();
}

/* USART "buffer" (single char for now) */
char received = 0;

ISR( USART_RX_vect, ISR_BLOCK ) {
  /* Get the (inbound) character from the USART. */
  received = UDR;

  /* Set Forth interrupt */
  interrupt = interrupt_usart;
}

bool usart_send( byte t ) {
  /* Sent the character out through the USART. */
  if( !( UCSRA & (1 << UDRE))) {
    return false;
  }
  UDR = t;
  return true;
}

/* PWM control */

/* Patterns may be read from Flash or from RAM over SPI. Patterns must provide a value for base frequency (OCR1A) and duty cycle (OCR1B). */

/*
 * "If the base PWM frequency is actively changed, using the OCR1A as TOP is clearly a better choice."
 * Mode: Fast PWM; OC1A is not connected (used as top) therefor COM1A1=0, COM1A0=0; OC1B is used as PWM output (with output cleared at Compare Match and set at top, therefor COM1B1=1, COM1B0=0); WGM13=1; WGM(12,11,10)=1
 *
 * Also TOV1 must be set so that the interrupt is called to update the new OCR1A and OCR1B at each cycle.
 *
 * See also App Note http://www.atmel.com/Images/doc2542.pdf for filtering (R in series, C between output of R and ground, R=10k, C=100nF for crossover frequency of 1kHz (low-pass filter)).
 */

#define CS_RAM_PORT PORTD
#define CS_RAM_PIN  PORTD2
#define CS_FLASH_PORT PORTD
#define CS_FLASH_PIN  PORTD3

inline void clear_bit_CS_RAM() {
  CS_RAM_PORT &= ~_BV(CS_RAM_PIN);
}

inline void clear_bit_CS_FLASH() {
  CS_FLASH_PORT &= ~_BV(CS_FLASH_PIN);
}

inline void set_bit_CS_RAM() {
  CS_RAM_PORT |= _BV(CS_RAM_PIN);
}

inline void set_bit_CS_FLASH() {
  CS_FLASH_PORT |= _BV(CS_FLASH_PIN);
}


/* Flash/RAM SPI interface */

#define spi_mode(addr) ((byte)(addr >> 24))

void spi_start( byte mode ) {
  switch( mode & 0xf0 ) {
    case 0x10: // ADDR_FLASH >> 24
      clear_bit_CS_FLASH();
      break;
    case 0x20: // ADDR_RAM >> 24
      clear_bit_CS_RAM();
      break;
  }
  cli();
}

void spi_stop() {
  sei();
  set_bit_CS_RAM();
  set_bit_CS_FLASH();
}

uint8_t byte_SPI( uint8_t value ) {
  USIDR = value;

  uint8_t clock_off = _BV(USIWM0) | _BV(USITC);
  uint8_t clock_on  = _BV(USIWM0) | _BV(USITC) | _BV(USICLK);

  /* Toggle the external clock (USITC) while progressing the internal clock (USICLK) */
  for( byte i = 0; i < 8; i++ ) {
    USICR = clock_off;
    USICR = clock_on;
  }
  /* See ATtiny2313A doc, page 158 bottom */

  return USIDR;
}

void spi_read(pointer target, external_pointer start, byte length ) {
  spi_start(spi_mode(start));
  // TODO: send command & send address
  while(length--) {
    *(target++) = byte_SPI(0);
  }
  spi_stop();
}

void spi_write(pointer source, external_pointer start, byte length ) {
  spi_start(spi_mode(start));
  // TODO: send command & send address
  while(length--) {
    byte_SPI(*(source++));
  }
  spi_stop();
}

/* Higher-level access methods */
word read_word( external_pointer p ) {
  word w;
  spi_read((pointer)&w,p,sizeof(w));
  return w;
}

byte read_byte( external_pointer p ) {
  byte b;
  spi_read(&b,p,sizeof(b));
  return b;
}

void write_word( external_pointer p, word w ) {
  spi_write((pointer)&w,p,sizeof(w));
}

void write_byte( external_pointer p, byte b ) {
  spi_write(&b,p,sizeof(b));
}

/* Forth operations */

/*
 * The stacks are stored in (external) RAM.
 * For now the code does not attempt to do anything clever like caching.
 */

enum { rp0 = 0x1000, sp0 = 0x2000, pad = 0x2000 };

typedef struct {
  external_pointer p;
} stack;

word top(stack* s) {
  return read_word(s->p);
}

void set_top(stack* s, word r) {
  write_word(s->p,r);
}

word top2(stack* s) {
  return read_word(s->p+sizeof(word));
}

void set_top2(stack* s, word r) {
  write_word(s->p+sizeof(word),r);
}

void push(stack* s, word r) {
  s->p -= sizeof(word);
  write_word(s->p,r);
}

word pop(stack* s) {
  word w = read_word(s->p);
  s->p += sizeof(word);
  return w;
}

const word FORTH_FALSE = 0;
const word FORTH_TRUE = ~0;

int main() {
  init_usart();

  stack R = { p: (external_pointer)(rp0 | ADDR_RAM) };
  stack S = { p: (external_pointer)(sp0 | ADDR_RAM) };

  external_pointer ip = 0;
  stack* s = &S;
  stack* r = &R;

  while(1) {
    /* On interrupt we save the current ip pointer and proceed to the interrupt vector. */
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
      if(interrupt) {
        push(r,ip);
        ip = interrupt;
      }
    }
    word op = read_word(ip);
    ip += sizeof(word);

    /* If the value is an address, call to that address. */
    if( (byte)(op >> 24) & 0xf0 ) {
      push(r,ip);
      ip = op;
    } else {
      /* Otherwise the value is an opcode. */

      word w;
      byte b;
      switch(op) {

        case 0: // opcode("exit")
          ip = pop(r);
          break;

        case 1: // opcode(">r")
          push(r, pop(s));
          break;

        case 2: // opcode("r@")
          push(s, top(r));
          break;

        case 3: // opcode("r>")
          push(s, pop(r));
          break;

        case 4: // opcode("rdrop")
          r->p += sizeof(word);
          break;

        case 5: // opcode("dup")
          push(s, top(s));
          break;

        case 6: // opcode("drop")
          s->p += sizeof(word);
          break;

        case 7: // opcode("nip")
          w = pop(s);
          set_top(s,w);
          break;

        case 8: // opcode("swap")
          w = top(s);
          set_top(s,top2(s));
          set_top2(s,w);
          break;

        case 9: // opcode("over")
          w = top2(s);
          push(s,w);
          break;

        case 10: // opcode("u<")
          w = pop(s);
          set_top(s, top(s) < w ? FORTH_TRUE : FORTH_FALSE);
          break;

        case 11: // opcode("+")
          w = pop(s);
          set_top(s, top(s) + w);
          break;

        case 12: // opcode("and")
          w = pop(s);
          set_top(s, top(s) & w);
          break;

        case 13: // opcode("or")
          w = pop(s);
          set_top(s, top(s) | w);
          break;

        case 14: // opcode("xor") // optionally
          w = pop(s);
          set_top(s, top(s) ^ w);
          break;

        case 15: // opcode("1+")
          set_top(s, top(s) + 1);
          break;

        case 16: // opcode("0=")
          set_top(s, top(s) == 0 ? FORTH_TRUE : FORTH_FALSE);
          break;

        case 17: // opcode("negate") // optionally
          set_top(s, -top(s));
          break;

        case 18: // opcode("invert") optionally
          set_top(s, ~top(s));
          break;

        case 19: // opcode("2/")
          set_top(s, top(s) >> 1);
          break;

        case 20: // opcode("c@")
          set_top(s, read_byte(top(s)));
          break;

        case 21: // opcode("@")
          set_top(s, read_word(top(s)));
          break;

        case 22: // opcode("c!")
          w = pop(s);
          write_byte(w,pop(s) & 0xff);
          break;

        case 23: // opcode("!")
          w = pop(s);
          write_word(w,pop(s));
          break;

        case 24: // opcode("lit")
          push(s, read_word(ip));
          break;

        case 25: // opcode("0branch")
          ip = pop(s) == 0 ? read_word(ip) : ip+sizeof(word);
          break;

        case 26: // opcode("branch")
          ip = read_word(ip);
          break;

        /* Normally called from within `interrupt_usart`. */
        case 27: // opcode("get-key")
          push(s,received);
          break;

        /* Send one character out through USART; returns TRUE iff the character was sent. */
        case 28: // opcode("emit")
          b = top(s) & 0xff;
          set_top(s, usart_send(b) ? FORTH_TRUE : FORTH_FALSE);
          break;
      }
    }
  }
}

void exit(int __status) {}
