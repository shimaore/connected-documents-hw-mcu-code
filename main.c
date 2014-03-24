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

enum {
  MODE_FLASH = 0x10,
  MODE_RAM   = 0x20,

  MODE_MASK  = 0x0f
};

/* Forth Interrupt Block layout */

enum {
  forth_interrupt_reset = 0 | ADDR_FLASH,
  forth_interrupt_usart = 4 | ADDR_FLASH,
};

/* Run Forth reset when AVR resets. */

word forth_interrupt = forth_interrupt_reset;

/************************ USART *******************************/

/* Bluetooth interface uses the USART in RS-232 mode. */

/* Defaults:
 * - BC-04 uses 9600/1 stop/no parity
 * - HC-05 uses 9600/1 stop/no parity
 * - HM-10 uses 
 */

void usart_init() {
  UCSRB = (1<<RXCIE)|(1<<RXEN)|(1<<TXEN);
  UCSRC = (3<<UCSZ0); // 8 bits
  usart_9600();
}

/* USART "buffer" (single char for now) */
uint8_t received = 0;

ISR( USART_RX_vect, ISR_BLOCK ) {
  /* Get the (inbound) character from the USART. */
  received = UDR;

  /* Set Forth interrupt */
  forth_interrupt = forth_interrupt_usart;
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

void pwm_init() {
}



/*********************** Flash/RAM SPI interface ********************************/

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

/* Instructions for 23K256-IS/N */
#define RAM_READ            0x03
#define RAM_WRITE           0x02
#define RAM_RDSR            0x05
#define RAM_WRSR            0x01

#define RAM_SEQUENTIAL_MODE 0x40
#define RAM_ENABLE_HOLD     0x00
#define RAM_DISABLE_HOLD    0x01

void spi_init() {
  // Set the SRAM to Sequential Mode (bit 7/6 = 01); also enable HOLD (bit 0 = 0).
  clear_bit_CS_RAM(); // Chip Select RAM
  cli();
  byte_SPI(RAM_WRSR); // Write Status Register
  byte_SPI(RAM_SEQUENTIAL_MODE | RAM_ENABLE_HOLD);
  set_bit_CS_RAM(); // Chip unSelect RAM

  // TODO: Configure the Flash
  clear_bit_CS_FLASH(); // Chip Select Flash

  set_bit_CS_FLASH(); // Chip unSelect Flash

  // Re-enable interrupts
  sei();
}

void spi_start( byte mode ) {
  switch( mode & MODE_MASK ) {
    case MODE_FLASH:
      clear_bit_CS_FLASH();
      break;
    case MODE_RAM:
      clear_bit_CS_RAM();
      break;
  }
  cli();
}

void spi_address( byte mode, word target ) {
  switch( mode & MODE_MASK ) {
    case MODE_FLASH:
      byte_SPI((target>>16)&0xff);
      byte_SPI((target>>8)&0xff);
      byte_SPI((target)&0xff);
      break;
    case MODE_RAM:
      byte_SPI((target>>8)&0xff);
      byte_SPI((target)&0xff);
      break;
  }
}

void spi_stop() {
  sei();
  set_bit_CS_RAM();
  set_bit_CS_FLASH();
}

#define spi_mode(addr) ((byte)(addr >> 24))

void spi_read(pointer target, external_pointer start, byte length ) {
  spi_start(spi_mode(start));
  // send command & send address
  byte_SPI(0x03);
  spi_address(spi_mode(start),start);
  while(length--) {
    *(target++) = byte_SPI(0);
  }
  spi_stop();
}

void spi_write(pointer source, external_pointer start, byte length ) {
  spi_start(spi_mode(start));
  // send command & send address
  byte_SPI(0x04);
  spi_address(spi_mode(start),start);
  while(length--) {
    byte_SPI(*(source++));
  }
  spi_stop();
}

/* Higher-level SPI access methods */
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

typedef external_pointer stack;

const word FORTH_FALSE = 0;
const word FORTH_TRUE = ~0;

int main() {
  usart_init();
  spi_init();

  stack s = (stack)(sp0 | ADDR_RAM);
  stack r = (stack)(rp0 | ADDR_RAM);

  inline word top() {
    return read_word(s);
  }

  inline void set_top(word w) {
    write_word(s,w);
  }

  word top2() {
    return read_word(s+sizeof(word));
  }

  void set_top2(word w) {
    write_word(s+sizeof(word),w);
  }

  void push(word w) {
    s -= sizeof(word);
    write_word(s,w);
  }

  word pop() {
    word w = read_word(s);
    s += sizeof(word);
    return w;
  }

  inline word r_top() {
    return read_word(r);
  }

  void r_push(word w) {
    r -= sizeof(word);
    write_word(r,w);
  }

  word r_pop() {
    word w = read_word(r);
    r += sizeof(word);
    return w;
  }

  void set_top_truthness(bool b) {
    set_top(b ? FORTH_TRUE : FORTH_FALSE);
  }


  external_pointer ip = 0;

  while(1) {
    /* On interrupt we save the current ip pointer and proceed to the interrupt vector. */
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
      if(forth_interrupt) {
        r_push(ip);
        ip = forth_interrupt;
      }
    }
    word op = read_word(ip);
    ip += sizeof(word);

    /* If the value is an address, call to that address. */
    if( spi_mode(op) & MODE_MASK ) {
      r_push(ip);
      ip = op;
    } else {
      /* Otherwise the value is an opcode. */

      word w;
      byte b;
      switch(op) {

        case 0: // opcode("exit")
          ip = r_pop();
          break;

        case 1: // opcode(">r")
          r_push(pop());
          break;

        case 2: // opcode("r@")
          push(r_top());
          break;

        case 3: // opcode("r>")
          push(r_pop());
          break;

        case 4: // opcode("rdrop")
          r += sizeof(word);
          break;

        case 5: // opcode("dup")
          push(top());
          break;

        case 6: // opcode("drop")
          s += sizeof(word);
          break;

        case 7: // opcode("nip")
          w = pop();
          set_top(w);
          break;

        case 8: // opcode("swap")
          w = top();
          set_top(top2());
          set_top2(w);
          break;

        case 9: // opcode("over")
          w = top2();
          push(w);
          break;

        case 10: // opcode("u<")
          w = pop();
          set_top_truthness(top() < w);
          break;

        case 11: // opcode("+")
          w = pop();
          set_top(top() + w);
          break;

        case 12: // opcode("and")
          w = pop();
          set_top(top() & w);
          break;

        case 13: // opcode("or")
          w = pop();
          set_top(top() | w);
          break;

        case 14: // opcode("xor") // optionally
          w = pop();
          set_top(top() ^ w);
          break;

        case 15: // opcode("1+")
          set_top(top() + 1);
          break;

        case 16: // opcode("0=")
          set_top_truthness(top() == 0);
          break;

        case 17: // opcode("negate") // optionally
          set_top(-top());
          break;

        case 18: // opcode("invert") optionally
          set_top(~top());
          break;

        case 19: // opcode("2/")
          set_top(top() >> 1);
          break;

        case 20: // opcode("c@")
          set_top(read_byte(top()));
          break;

        case 21: // opcode("@")
          set_top(read_word(top()));
          break;

        case 22: // opcode("c!")
          w = pop();
          write_byte(w,pop() & 0xff);
          break;

        case 23: // opcode("!")
          w = pop();
          write_word(w,pop());
          break;

        case 24: // opcode("lit")
          push(read_word(ip));
          break;

        case 25: // opcode("0branch")
          ip = pop() == 0 ? read_word(ip) : ip+sizeof(word);
          break;

        case 26: // opcode("branch")
          ip = read_word(ip);
          break;

        /* Normally called from within `forth_interrupt_usart`. */
        case 27: // opcode("get-key")
          push((word)received);
          break;

        /* Send one character out through USART; returns TRUE iff the character was sent. */
        case 28: // opcode("emit")
          b = top() & 0xff;
          set_top_truthness(usart_send(b));
          break;
      }
    }
  }
}

void exit(int __status) {}
