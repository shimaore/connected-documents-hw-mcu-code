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

/* This is for a 8MHz clock. */
#define F_CPU 8000000

#ifdef __EMULATE
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

#define ATOMIC_FORCEON
#define ATOMIC_BLOCK(x)
#define set_sleep_mode(x)

#define opcode(x) printf("  opcode %s\n",x);

enum { sizeof_cell = 3 };

#else

#include <stdlib.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <avr/sleep.h>

#define opcode(x)

enum { sizeof_cell = sizeof(cell) };

#endif // __EMULATE

typedef uint8_t byte;
typedef byte* pointer;

typedef uint8_t bool;
enum { false = 0, true = 1 };

/* We use 16bits integers for PWM. */
typedef uint16_t word;

#ifdef __EMULATE
typedef uint32_t __uint24;
#endif

/* With a 512Ko flash we need at least 3 octets per address */
typedef __uint24 cell;
typedef cell  external_pointer;

/* Part of the address is used as target select. */

/* Note: there is only Flash or RAM, and we only test
 * for Flash; we use the highest bit as Flash indicator
 * so that the compiler can optimize the test as a
 * "is number negative" test.
 */

enum {
  ADDR_FLASH = 0x800000,
  ADDR_RAM   = 0x400000,

  ADDR_MASK  = 0xc00000
};

/* Forth Interrupt Block layout */

enum {
  forth_interrupt_reset = 0 | ADDR_FLASH,
  forth_interrupt_usart = sizeof_cell | ADDR_FLASH,
};

/* Run Forth reset when AVR resets. */

volatile cell forth_interrupt = forth_interrupt_reset;

/************************ USART *******************************/

#ifdef __EMULATE
inline void usart_9600() {}
inline void usart_38400() {}
#else

inline void usart_9600() {
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

inline void usart_38400() {
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

#endif // __EMULATE

/* Bluetooth interface uses the USART in RS-232 mode. */

/* Defaults:
 * - BC-04 uses 9600/1 stop/no parity
 * - HC-05 uses 9600/1 stop/no parity
 * - HM-10 uses 
 */


#ifdef __EMULATE
inline void usart_init() {
  printf("usart_init()\n");
}
#else
inline void usart_init() {
  UCSRB = _BV(RXCIE)|_BV(RXEN)|_BV(TXEN);
  UCSRC = (3<<UCSZ0); // 8 bits
  usart_9600();
}
#endif

/* USART "buffer" (single char for now) */
volatile uint8_t received = 0;

#ifdef __EMULATE

void sleep_mode() {
  /* Get the (inbound) character from stdin. */
  received = getchar();

  /* Set Forth interrupt */
  forth_interrupt = forth_interrupt_usart;
}

#else

ISR( USART_RX_vect, ISR_BLOCK ) {
  /* Get the (inbound) character from the USART. */
  received = UDR;

  /* Set Forth interrupt */
  forth_interrupt = forth_interrupt_usart;
}

#endif /* __EMULATE */

#ifdef __EMULATE

inline bool usart_send( byte t ) {
  putchar(t);
}

#else

inline bool usart_send( byte t ) {
  /* Sent the character out through the USART. */
  if( !( UCSRA & (1 << UDRE))) {
    return false;
  }
  UDR = t;
  return true;
}

#endif /* __EMULATE */

/*********************** Flash/RAM SPI interface ********************************/

#ifdef __EMULATE
// No pins in emulation

byte flash_memory[FLASH_SIZE];
byte ram_memory[RAM_SIZE];

byte flash_init[] = {
#include "forth.bin.h"
  0xff
};

inline void spi_init() {
  cell i;
  printf("spi_init()\n");
  for( i = 0; i < FLASH_SIZE; i++ ) {
    flash_memory[i] = 0xff;
  }
  for( i = 0; i < sizeof(flash_init); i++ ) {
    flash_memory[i] = flash_init[i];
  }
  for( i = 0; i < RAM_SIZE; i++ ) {
    ram_memory[i] = 0xff;
  }

}

inline byte read_byte(external_pointer p) {
  if(p & ADDR_FLASH) {
    p -= ADDR_FLASH;
    if(p > FLASH_SIZE) {
      printf("Invalid read in Flash at %0x",p+ADDR_FLASH);
      exit(1);
    }
    return flash_memory[p];
  }
  if(p & ADDR_RAM) {
    p -= ADDR_RAM;
    if( p > RAM_SIZE ) {
      printf("Invalid read in RAM at %0x",p+ADDR_RAM);
      exit(1);
    }
    return ram_memory[p];
  }
  printf("Invalid read at %0x",p);
}

inline void write_byte(external_pointer p, byte b) {
  if(p & ADDR_FLASH) {
    p -= ADDR_FLASH;
    if(p > FLASH_SIZE) {
      printf("Invalid write in Flash at %0x",p+ADDR_FLASH);
      exit(1);
    }
    flash_memory[p] = b;
    return;
  }
  if(p & ADDR_RAM) {
    p -= ADDR_RAM;
    if( p > RAM_SIZE ) {
      printf("Invalid write in RAM at %0x",p+ADDR_RAM);
      exit(1);
    }
    ram_memory[p] = b;
    return;
  }
  printf("Invalid write at %0x",p);
  exit(1);
}
inline word read_word(external_pointer p) {
  return ((word)read_byte(p) << 8) | (word)read_byte(p+1);
}
inline void write_word(external_pointer p, word w) {
  write_byte(p, (w >> 8) & 0xff);
  write_byte(p+1, w & 0xff);
}

inline cell read_cell(external_pointer p) {
  return ((cell)read_byte(p) << 16) | ((cell)read_byte(p+1) << 8) | ((cell)read_byte(p+2));
}

inline void write_cell(external_pointer p, cell c) {
  write_byte(p+0, (c >> 16) & 0xff);
  write_byte(p+1, (c >> 8) & 0xff);
  write_byte(p+2, c & 0xff);
}

#else

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

inline void spi_init() {
  SCK_DDR |= _BV(SCK_BIT);
  DO_DDR |= _BV(DO_BIT);

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
  // Set the SRAM to Sequential Mode (bit 7/6 = 01); also enable HOLD (bit 0 = 0).
  clear_bit_CS_RAM(); // Chip Select RAM
  byte_SPI(RAM_WRSR); // Write Status Register
  byte_SPI(RAM_SEQUENTIAL_MODE | RAM_DISABLE_HOLD);
  set_bit_CS_RAM(); // Chip unSelect RAM

  /* The Flash is not configured, the forth code must take care of it using
   * the `spi_flash` instruction.
   */
  }
}

inline void spi_start( cell target ) {
  if( target & ADDR_FLASH ) {
    clear_bit_CS_FLASH();
  } else {
    clear_bit_CS_RAM();
  }
}

void spi_address( cell target ) {
  if( target & ADDR_FLASH ) {
      byte_SPI((target>>16)&0xff);
  }
  byte_SPI((target>>8)&0xff);
  byte_SPI((target)&0xff);
}

inline void spi_stop() {
  set_bit_CS_RAM();
  set_bit_CS_FLASH();
}

typedef enum {
  SPI_COMMAND_READ  = 0x03,
  SPI_COMMAND_WRITE = 0x04
} spi_command;

void spi_cmd(pointer target, external_pointer start, byte length, spi_command cmd ) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
  spi_start(start);
  // send command & send address
  byte_SPI(cmd);
  spi_address(start);
  while(length--) {
    *(target) = byte_SPI(*target);
    target++;
  }
  spi_stop();
  }
}

inline void spi_read(pointer t, external_pointer p, byte l) { spi_cmd(t,p,l,SPI_COMMAND_READ); }
inline void spi_write(pointer t, external_pointer p, byte l) { spi_cmd(t,p,l,SPI_COMMAND_WRITE); }

/* Higher-level SPI access methods */
cell spi_cell( external_pointer p, cell c, byte cmd ) {
  spi_cmd((pointer)&c,p,sizeof(c),cmd);
  return c;
}

word spi_word( external_pointer p, word w, byte cmd ) {
  spi_cmd((pointer)&w,p,sizeof(w),cmd);
  return w;
}

byte spi_byte( external_pointer p, byte b, byte cmd ) {
  spi_cmd((pointer)&b,p,sizeof(b),cmd);
  return b;
}

inline byte read_byte(external_pointer p) { return spi_byte(p,0,SPI_COMMAND_READ); }
inline void write_byte(external_pointer p, byte b) { spi_byte(p,b,SPI_COMMAND_WRITE); }
inline word read_word(external_pointer p) { return spi_word(p,0,SPI_COMMAND_READ); }
inline void write_word(external_pointer p, word w) { spi_word(p,w,SPI_COMMAND_WRITE); }
inline cell read_cell(external_pointer p) { return spi_cell(p,0,SPI_COMMAND_READ); }
inline void write_cell(external_pointer p, cell c) { spi_cell(p,c,SPI_COMMAND_WRITE); }

#endif

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

#ifdef __EMULATE

inline void pwm_init() {
  printf("pwm_init\n");
}

#else

/* Address in main-memory/flash where the loop description starts. */
external_pointer loop_start = 0;
volatile word loop_length = 0; // in number of uint16_t words
volatile word loop_pos = 0; // in number of uint16_t words

inline void pwm_init() {
  TCCR1A = _BV(COM1B1) | _BV(WGM10) | _BV(WGM11);
  TCCR1B = _BV(WGM12) | _BV(WGM13);
  TIFR |= _BV(TOV1);
  TIMSK |= _BV(TOIE1);
  // Set OC1A / PB3 as output.
  OC1B_DDR |= _BV(OC1B_BIT);
}

inline void pwm_change_loop( external_pointer start ) {
  loop_start = start;

  /* The loop description starts with the base frequency of the loop. */
  OCR1A = read_word(loop_start);

  /* OCR1B is never modified if the loop_length is zero */
  OCR1B = 0;

  /* Then comes the length in number of 16bits elements, itself a 16bits element. */
  loop_length = read_word(loop_start+2);
  loop_pos = 0;
}

ISR( TIMER1_OVF_vect, ISR_BLOCK ) {
  if(loop_pos >= loop_length) {
    loop_pos = 0;
  }
  OCR1B = read_word(loop_pos);
  loop_pos++;
}

/* Examples of scenarios to test:
 * - base frequency defined but length = 0
 * - base frequency with length = 1 (set duty cycle)
 * - base frequency with length < pwm_buffer_size
 * - base frequency with length = pwm_buffer_size
 * - base frequency with length > pwm_buffer_size (2 blocks)
 * - base frequency with length > pwm_buffer_size (>2 blocks)
 */

#endif /* __EMULATE */

/****************** Forth operations *******************/

/*
 * The stacks are stored in (external) RAM.
 * For now the code does not attempt to do anything clever like caching.
 */

enum {
  rp0 = 0x1000,
  sp0 = 0x2000
};

typedef external_pointer stack;

const cell FORTH_FALSE = (cell)0;
const cell FORTH_TRUE = (cell)~0;

int main() {
  usart_init();
  spi_init();
  pwm_init();

  register stack s = (stack)(sp0 | ADDR_RAM);
  register stack r = (stack)(rp0 | ADDR_RAM);

  inline cell top() {
    return read_cell(s);
  }

  inline void set_top(cell c) {
    write_cell(s,c);
  }

  cell top2() {
    return read_cell(s+sizeof_cell);
  }

  void set_top2(cell c) {
    write_cell(s+sizeof_cell,c);
  }

  void push(cell c) {
#ifdef __EMULATE
    printf("  push %06x\n",c);
#endif
    s -= sizeof_cell;
    write_cell(s,c);
  }

  cell pop() {
    cell c = read_cell(s);
    s += sizeof_cell;
    return c;
  }

  inline cell r_top() {
    return read_cell(r);
  }

  void r_push(cell c) {
#ifdef __EMULATE
    printf("  rpush %06x\n",c);
#endif
    r -= sizeof_cell;
    write_cell(r,c);
  }

  cell r_pop() {
    cell c = read_cell(r);
    r += sizeof_cell;
    return c;
  }

  void set_top_truthness(bool b) {
    set_top(b ? FORTH_TRUE : FORTH_FALSE);
  }

  /* IP is set to 0, but since forth_interrupt is set at startup, it will prevail. */
  external_pointer ip = 0;

  while(1) {
    /* On interrupt we save the current ip pointer and proceed to the interrupt vector. */
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
      if(forth_interrupt) {
        r_push(ip);
        ip = forth_interrupt;
        forth_interrupt = 0;
      }
    }
    cell op = read_cell(ip);
#ifdef __EMULATE
    printf("Read op=%06x from ip=%06x [rp=%06x,sp=%06x]\n",op,ip,r,s);
#endif
    ip += sizeof_cell;

    /* If the value is an address, call to that address. */
    if( op & ADDR_MASK ) {
#ifdef __EMULATE
      printf("Push ip=%06x\n",ip);
#endif
      r_push(ip);
      ip = op;
#ifdef __EMULATE
      printf("Going to ip=%06x\n",ip);
#endif
    } else {
      /* Otherwise the value is an opcode. */

      cell c;
      byte b;
      enum { buffer_length = 0x20 };
      uint8_t buffer[buffer_length];
      switch(op) {

        /**** IP manipulation ****/
        case 0: opcode("EXIT")
          ip = r_pop();
          break;

        case 1: opcode("BRANCH") // optionally:  0 0branch or: lit N >R exit
          ip = read_cell(ip);
          break;

        case 2: opcode("LIT")
          push(read_cell(ip));
          ip += sizeof_cell;
          break;

        case 3: opcode("0BRANCH")
          ip = pop() == 0 ? read_cell(ip) : ip+sizeof_cell;
          break;

        /***** Return stack *****/
        case 4: opcode(">R")
          r_push(pop());
          break;

        case 5: opcode("R@") // optionally: r> dup >r
          push(r_top());
          break;

        case 6: opcode("RDROP") // optionally: r> drop
          r += sizeof_cell;
          break;

        /***** Data Stack *****/
        case 7: opcode("DUP")
          push(top());
          break;

        case 8: opcode("DROP")
          s += sizeof_cell;
          break;

        case 9: opcode("SWAP")
          c = top();
          set_top(top2());
          set_top2(c);
          break;

        case 10: opcode("OVER")
          c = top2();
          push(c);
          break;

        /****** ALU ********/
        case 11: opcode("U<")
          c = pop();
          set_top_truthness(top() < c);
          break;

        case 12: opcode("0=")
          set_top_truthness(top() == 0);
          break;

        case 13: opcode("+")
          c = pop();
          set_top(top() + c);
          break;

        case 14: opcode("2/")
          set_top(top() >> 1);
          break;

        case 15: opcode("AND")
          c = pop();
          set_top(top() & c);
          break;

        case 16: opcode("OR")
          c = pop();
          set_top(top() | c);
          break;

        case 17: opcode("XOR")
          c = pop();
          set_top(top() ^ c);
          break;

        /***** Memory *******/
        case 18: opcode("C@")
          set_top(read_byte(top()));
          break;

        case 19: opcode("C!")
          c = pop();
          write_byte(c,pop() & 0xff);
          break;

        case 20: opcode("W!")
          c = pop();
          write_word(c,pop() & 0xffff);

        case 21: opcode("W@")
          set_top(read_word(top()));
          break;

        case 22: opcode("@")
          set_top(read_cell(top()));
          break;

        case 23: opcode("!")
          c = pop();
          write_cell(c,pop());
          break;

        /* Normally called from within `forth_interrupt_usart`. */
        case 24: opcode("usart_received")
          push((cell)received);
          break;

        /* Send one character out through USART; returns TRUE iff the character was sent. */
        case 25: opcode("usart_send")
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
        case 26: opcode("spi-flash")
#ifdef __EMULATE
          printf("Invalid opcode spi-flash in emulation mode.\n");
          exit(1);
          break;
#else
          {
            b = pop() & (buffer_length-1);
            c = pop();
            pointer source = buffer;
            /* Read the command */
            spi_read(buffer,c,b);
            /* Execute the command */
            ATOMIC_BLOCK(ATOMIC_FORCEON) {
              clear_bit_CS_FLASH();
              while(c--) {
                *source = byte_SPI(*source);
                source++;
              }
              set_bit_CS_FLASH();
            }
            /* Write the output back */
            spi_write(buffer,c,b);
          }
          break;
#endif

        case 27: opcode("pwm")
#ifdef __EMULATE
          printf("Invalid opcode pwm in emulation mode.\n");
          exit(1);
          break;
#else
          pwm_change_loop(pop());
          break;
#endif

        case 28: opcode("usart-9600")
          usart_9600();
          break;

        case 29: opcode("usart-38400")
          usart_38400();
          break;

        case 30: opcode("sleep")
          set_sleep_mode(SLEEP_MODE_IDLE);
          sleep_mode();
          break;

#ifdef __EMULATE
        default:
          printf("Invalid opcode in emulation mode.\n");
          exit(1);
          break;
#endif

      }
    }
  }
}

#ifdef __EMULATE
// Use standard exit()
#else
void exit(int __status) {}
#endif
