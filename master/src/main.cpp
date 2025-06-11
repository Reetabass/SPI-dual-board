#include <avr/io.h>
#include <avr/interrupt.h>
#include "util/delay.h"
#include <stdlib.h>
#include <math.h>

#define bitSet(reg, n) (reg |= 1 << n)
#define bitRead(reg, n) (reg >> n & 1)
#define bitClear(reg, n) (reg &= ~(1 << n))
#define bitCheck(reg, n) (reg & 1 << n)

//TWI definitions
#define OWN_ADDR 0x11
#define SLA_ADDR 0x22
#define SLA_W (SLA_ADDR << 1)
#define SLA_R (SLA_ADDR << 1 | 1)



//USART Declerations
void usart_init(float baud);
void usart_send_byte(unsigned char data);
void usart_send_string(char *pstr);
void usart_send_num(float num, char num_int, char num_decimal);
void usart_init_v2(float baud);
void usart_flush(void);
void adc_init();


//declerations

void twi_init_master();
void twi_write(uint8_t x);

//GLOBAL VARIABLES

volatile bool flag_slave_done = 0;
volatile bool flag_usart_done = 0;
unsigned char data = 0;

char twi_buffer[50];
volatile uint8_t twi_buffer_index = 0;

char usart_buffer[50];
char *pstr = usart_buffer;



char remote_name[]= "Connor: ";
char local_name[] = "Reed: ";


ISR(USART_RX_vect) {
  char tmp = UDR0;

  if (tmp == '\n') {
      *pstr++ = '\n';
      *pstr = '\0';           // Null-terminate the string
      pstr = usart_buffer;    // Reset pointer for next message

      bitClear(UCSR0B, RXCIE0);  // Disable RX interrupt until main re-enables

      usart_send_string(local_name);   // Optional echo back
      usart_send_string(usart_buffer);

      flag_usart_done = 1;
  } else {
      *pstr++ = tmp;
      if ((pstr - usart_buffer) >= sizeof(usart_buffer)) {
          pstr = usart_buffer; // Prevent overflow
      }
  }
}

ISR(TWI_vect)
{
    uint8_t status = TWSR & 0xF8;
    //usart_tx_reg(status); // Optional debug
    switch (status)
    {
        case 0x08:
        case 0x10:
            TWDR = SLA_R;
            twi_buffer_index = 0;
            TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWIE) | (1 << TWINT);
            break;

        case 0x40: // Data received, ACK returned
            TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWIE) | (1 << TWINT);
            break;

        case 0x50: // Data received, ACK returned
            twi_buffer[twi_buffer_index++] = TWDR;
            if (twi_buffer[twi_buffer_index - 1] == '\n')
            {
                TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT);
            }
            else
            {
                TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWIE) | (1 << TWINT);
            }
            break;

        case 0x48:
            TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT) | (1 << TWIE);
            break;

        case 0x58: // STOP or repeated START received
            usart_send_string("Stop or repeated start");
            twi_buffer[twi_buffer_index] = '\0';
            flag_slave_done = 1;
            TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA) | (1 << TWSTO);
            break;

        default:
            TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWINT);
            break;
    }
}



int main(void) {

  bitClear( DDRC, PC4);
  bitSet(PORTC, PC4); //SDA
  bitClear(DDRC, PC5);
  bitSet(PORTC, PC5); // SCL

  usart_init_v2(9600);
  sei();

  while(1) {

    flag_usart_done = 0;
    
    bitSet(UCSR0B, RXCIE0);
    
    while(!flag_usart_done)
        ;
      
    twi_init_master();
    TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);
    
    while(!(TWCR & (1 << TWINT))) usart_send_string("while 1");

    twi_write(SLA_W);

    while(*pstr != '\0'){
      twi_write(*pstr++);
     usart_send_string("while 2");
    } 
    pstr = usart_buffer;


    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
    while(TWCR & (1 << TWSTO)) usart_send_string("while 3");

    _delay_ms(100);
    

    flag_slave_done = 0;
    TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT) | (1 << TWIE);

    while(!flag_slave_done) //usart_send_string(" slave not done ");

    usart_send_string(remote_name);
    usart_send_string(twi_buffer);
    _delay_ms(1000);
    // }

  }
  

}


//write
void twi_write(uint8_t x) {
  TWDR = x;
  TWCR = (1 << TWINT) | (1 << TWEN);
  while (!(TWCR & (1 << TWINT)));

}
//init

void twi_init_master() {
  TWBR = 0x48;
  TWSR = 0;
}


/*USART FUNCTIONS
*
*
*
*/

void usart_init(float baud) {

  float ubrr0 = 1.0e6 / baud;
  int ubrr0a = (int)ubrr0;

  if(ubrr0 - ubrr0a >= 0.5) {
    ubrr0a = ubrr0a + 1;
  }

  UBRR0 = ubrr0a;
  bitSet(UCSR0B, TXEN0);
  UCSR0C |= 3 << UCSZ00;
}

void usart_send_byte(unsigned char data) {
  while(!bitCheck(UCSR0A, UDRE0));
  UDR0 = data;
}

void usart_send_string(char *pstr) {
  while(*pstr != '\0') {
    usart_send_byte(*pstr);
    pstr++;
  }
}

void usart_send_num(float num, char num_int, char num_decimal) {
  char str[20];
  if(num_decimal == 0) {
    dtostrf(num, num_int, num_decimal, str);
  }
  else {
    dtostrf(num, num_int + num_decimal + 1, num_decimal, str);
  }
  str[num_int + num_decimal + 1] = '\0';
  usart_send_string(str);
}

//USART initialization with RX enabled
void usart_init_v2(float baud) {
  usart_init(baud);
  bitSet(UCSR0B, RXCIE0);
  bitSet(UCSR0B, RXEN0);
}

void usart_flush(void) {
  char dummy;
  while(bitCheck(UCSR0A, RXC0)) {
    dummy = UDR0;
  }
}