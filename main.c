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

#include <stdlib.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

typedef uint8_t byte;
typedef byte* pointer;

typedef uint8_t bool;
enum { false = 0, true = 1 };

/* With a 512Ko flash we need at least 3 octets per address */
typedef uint32_t cell;
typedef cell  external_pointer;

/* We used 16bits integers for PWM. */
typedef uint32_t word;

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

cell forth_interrupt = forth_interrupt_reset;

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
enum {
  RAM_READ    = 0x03,
  RAM_WRITE   = 0x02,
  RAM_RDSR    = 0x05,
  RAM_WRSR    = 0x01,

  RAM_SEQUENTIAL_MODE = 0x40,
  RAM_ENABLE_HOLD     = 0x00,
  RAM_DISABLE_HOLD    = 0x01
};

void spi_init() {
  cli();

  // Set the SRAM to Sequential Mode (bit 7/6 = 01); also enable HOLD (bit 0 = 0).
  clear_bit_CS_RAM(); // Chip Select RAM
  byte_SPI(RAM_WRSR); // Write Status Register
  byte_SPI(RAM_SEQUENTIAL_MODE | RAM_DISABLE_HOLD);
  set_bit_CS_RAM(); // Chip unSelect RAM

  /* The Flash is not configured, the forth code must take care of it using
   * the `spi_flash` instruction.
   */

  // Re-enable interrupts
  sei();
}

void spi_start( byte mode ) {
  cli();
  switch( mode & MODE_MASK ) {
    case MODE_FLASH:
      clear_bit_CS_FLASH();
      break;
    case MODE_RAM:
      clear_bit_CS_RAM();
      break;
  }
}

void spi_address( byte mode, cell target ) {
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
  set_bit_CS_RAM();
  set_bit_CS_FLASH();
  sei();
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
cell read_cell( external_pointer p ) {
  cell c;
  spi_read((pointer)&c,p,sizeof(c));
  return c;
}

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

void write_cell( external_pointer p, cell c ) {
  spi_write((pointer)&c,p,sizeof(c));
}

void write_word( external_pointer p, word w) {
  spi_write((pointer)&w,p,sizeof(w));
}

void write_byte( external_pointer p, byte b ) {
  spi_write(&b,p,sizeof(b));
}

/********************* PWM control ********************/

/* Patterns may be read from Flash or from RAM over SPI. Patterns must provide a value for base frequency (OCR1A) and duty cycle (OCR1B). */

/*
 * "If the base PWM frequency is actively changed, using the OCR1A as TOP is clearly a better choice."
 * Mode: Fast PWM; OC1A is not connected (used as top) therefor COM1A1=0, COM1A0=0; OC1B is used as PWM output (with output cleared at Compare Match and set at top, therefor COM1B1=1, COM1B0=0); WGM13=1; WGM(12,11,10)=1
 *
 * Also TOV1 must be set so that the interrupt is called to update the new OCR1A and OCR1B at each cycle.
 *
 * See also App Note http://www.atmel.com/Images/doc2542.pdf for filtering (R in series, C between output of R and ground, R=10k, C=100nF for crossover frequency of 1kHz (low-pass filter)).
 */

/* Address in main-memory/flash where the loop description starts. */
external_pointer loop_start = 0;
word loop_length = 0; // in number of uint16_t words
word loop_pos = 0; // in number of uint16_t words

/* Internal buffer for PWM buffer */
enum { pwm_buffer_size = 16 };
uint8_t pwm_buffer_len = 0;
uint8_t pwm_buffer_pos = 0;
word pwm_duty_cycles[pwm_buffer_size];

void pwm_init() {
  // TODO: initialize COM1A1, COM1A0, COM1B1, COM1B0, WGM, TOV1...
}

/* Copy a full block worth of data, then increments the position. */
void read_pwm_duty_cycles() {
  /* Simplify in case loop_length is zero */
  if(loop_length == 0) {
    loop_pos = 0;
    pwm_buffer_len = 0;
    pwm_buffer_pos = 0;
    return;
  }
  /* With a positive loop_length, check whether we are reading the last block,
   * which might be incomplete. */
  if( loop_pos + pwm_buffer_size >= loop_length ) {
    pwm_buffer_len = loop_length - loop_pos;
  } else {
    pwm_buffer_len = pwm_buffer_size;
  }
  spi_read((pointer)pwm_duty_cycles,loop_start+loop_pos*sizeof(word)+4,pwm_buffer_len*sizeof(word));
  loop_pos += pwm_buffer_len;
  if(loop_pos >= loop_length) {
    loop_pos = 0;
  }
  pwm_buffer_pos = 0;
}

void pwm_change_loop( external_pointer start ) {
  loop_start = start;

  /* The loop description starts with the base frequency of the loop. */
  OCR1A = read_word(loop_start);

  /* OCR1B is never modified if the loop_length is zero */
  OCR1B = 0;

  /* Then comes the length in number of 16bits elements, itself a 16bits element. */
  loop_length = read_word(loop_start+2);
  loop_pos = 0;

  read_pwm_duty_cycles();
}

ISR( TIMER1_OVF_vect, ISR_BLOCK ) {
  if(pwm_buffer_len > 0) {
    OCR1B = pwm_duty_cycles[pwm_buffer_pos];
    pwm_buffer_pos++;
    if(pwm_buffer_pos >= pwm_buffer_len) {
      read_pwm_duty_cycles();
    }
  }
}

/* Examples of scenarios to test:
 * - base frequency defined but length = 0
 * - base frequency with length = 1 (set duty cycle)
 * - base frequency with length < pwm_buffer_size
 * - base frequency with length = pwm_buffer_size
 * - base frequency with length > pwm_buffer_size (2 blocks)
 * - base frequency with length > pwm_buffer_size (>2 blocks)
 */


/****************** Forth operations *******************/

/*
 * The stacks are stored in (external) RAM.
 * For now the code does not attempt to do anything clever like caching.
 */

enum { rp0 = 0x1000, sp0 = 0x2000, pad = 0x2000 };

typedef external_pointer stack;

const cell FORTH_FALSE = 0;
const cell FORTH_TRUE = ~0;

int main() {
  usart_init();
  spi_init();
  pwm_init();

  stack s = (stack)(sp0 | ADDR_RAM);
  stack r = (stack)(rp0 | ADDR_RAM);

  inline cell top() {
    return read_cell(s);
  }

  inline void set_top(cell c) {
    write_cell(s,c);
  }

  cell top2() {
    return read_cell(s+sizeof(cell));
  }

  void set_top2(cell c) {
    write_cell(s+sizeof(cell),c);
  }

  void push(cell c) {
    s -= sizeof(cell);
    write_cell(s,c);
  }

  cell pop() {
    cell c = read_cell(s);
    s += sizeof(cell);
    return c;
  }

  inline cell r_top() {
    return read_cell(r);
  }

  void r_push(cell c) {
    r -= sizeof(cell);
    write_cell(r,c);
  }

  cell r_pop() {
    cell c = read_cell(r);
    r += sizeof(cell);
    return c;
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
    cell op = read_cell(ip);
    ip += sizeof(cell);

    /* If the value is an address, call to that address. */
    if( spi_mode(op) & MODE_MASK ) {
      r_push(ip);
      ip = op;
    } else {
      /* Otherwise the value is an opcode. */

      cell c;
      byte b;
      uint8_t buffer[0x20];
      switch(op) {

        /**** IP manipulation ****/
        case 0: // opcode("exit")
          ip = r_pop();
          break;

        case 1: // opcode("branch") // optionally:  0 0branch or: lit N >R exit
          ip = read_cell(ip);
          break;

        case 2: // opcode("lit")
          push(read_cell(ip));
          break;

        case 3: // opcode("0branch")
          ip = pop() == 0 ? read_cell(ip) : ip+sizeof(cell);
          break;

        /***** Return stack *****/
        case 4: // opcode(">r")
          r_push(pop());
          break;

        case 5: // opcode("r@") // optionally: r> dup >r
          push(r_top());
          break;

        case 6: // opcode("rdrop") // optionally: r> drop
          r += sizeof(cell);
          break;

        /***** Data Stack *****/
        case 7: // opcode("dup")
          push(top());
          break;

        case 8: // opcode("drop")
          s += sizeof(cell);
          break;

        case 9: // opcode("swap")
          c = top();
          set_top(top2());
          set_top2(c);
          break;

        case 10: // opcode("over")
          c = top2();
          push(c);
          break;

        /****** ALU ********/
        case 11: // opcode("u<")
          c = pop();
          set_top_truthness(top() < c);
          break;

        case 12: // opcode("0=")
          set_top_truthness(top() == 0);
          break;

        case 13: // opcode("+")
          c = pop();
          set_top(top() + c);
          break;

        case 14: // opcode("2/")
          set_top(top() >> 1);
          break;

        case 15: // opcode("and")
          c = pop();
          set_top(top() & c);
          break;

        case 16: // opcode("or")
          c = pop();
          set_top(top() | c);
          break;

        case 17: // opcode("xor")
          c = pop();
          set_top(top() ^ c);
          break;

        /***** Memory *******/
        case 18: // opcode("c@")
          set_top(read_byte(top()));
          break;

        case 19: // opcode("c!")
          c = pop();
          write_byte(c,pop() & 0xff);
          break;

        case 20: // opcode("w!")
          c = pop();
          write_word(c,pop() & 0xffff);

        case 21: // opcode("w@")
          set_top(read_word(top()));
          break;

        case 22: // opcode("@")
          set_top(read_cell(top()));
          break;

        case 23: // opcode("!")
          c = pop();
          write_cell(c,pop());
          break;

        /* Normally called from within `forth_interrupt_usart`. */
        case 24: // opcode("get-key")
          push((cell)received);
          break;

        /* Send one character out through USART; returns TRUE iff the character was sent. */
        case 25: // opcode("emit")
          b = top() & 0xff;
          set_top_truthness(usart_send(b));
          break;

        /* SPI commands */
        /* These commands are read from memory into an internal buffer, then sent out as a unit.
         * The response from the SPI bus is written back in the same internal buffer, which is
         * copied back into memory.
         * They allow the forth code to take over management of the flash, since it requires
         * to do sector erase, etc. before page program.
         * There's no similar instruction for RAM since RAM can be read & written without further
         * configuration.
         */
        /* spi_flash: addr length(<32) -- */
        case 26: // opcode("spi_flash")
          {
            b = pop() & 0x1f;
            c = pop();
            pointer source = buffer;
            spi_read(buffer,c,b);
            cli();
            clear_bit_CS_FLASH();
            while(c--) {
              *source = byte_SPI(*source);
              source++;
            }
            set_bit_CS_FLASH();
            sei();
            spi_write(buffer,c,b);
          }
          break;

        case 27: // opcode("pwm")
          pwm_change_loop(pop());
          break;

      }
    }
  }
}

void exit(int __status) {}
