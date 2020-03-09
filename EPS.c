// EPS_Software.c

//Libraries and Global Initialization
#define F_CPU 16000000UL

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <util/twi.h>
#include <avr/interrupt.h>
#include <util/crc16.h>

#define CLK1 PC4
#define CS1 PC5
#define DIN1 PC6
#define DOUT1 PC7

#define CLK2 PC0
#define CS2 PC1
#define DIN2 PC2
#define DOUT2 PC3

enum
{
  X,
  Y,
  Z
};

//******------------SSP/I2C------------******//

uint8_t data_send = 0x00, data_receive = 0x00, data_receive_main = 0x00, data_send_main = 0x00;
uint8_t I2C_ADDR = 0x20;

// Function to initialize slave
void TWI_init_slave(void)
{
  // Fill slave address to TWAR

  TWBR = 1;
  //Setting up the SCL frequency by writing a value in TWBR
  TWSR |= (0 << TWPS1) | (0 << TWPS0);
  //Fscl=     Fcpu/(16+2*(TWBR)*(4^TWPS))
  TWAR = 0x20;
  //The first seven bits indicate the slave address
  TWCR |= (1 << TWINT);
  TWCR |= (1 << TWEN) | (1 << TWEA) | (0 << TWSTA) | (0 << TWSTO);
  //Enabling Acknowledge function
  while (!(TWCR & (1 << TWINT)))
    ;
  //Wait for the interrupt to be cleared as it will indicate the successful reception
  while (TWCR & (0xF8) != (0x60))
    ;
  //Checking if the self-address+W has been received and ACK has been sent
}

void TWI_write_slave(void) // Function to write data
{
  // Fill TWDR register with the data to be sent
  TWDR = data_send;
  // Enable TWI, Clear TWI interrupt flag
  TWCR = (1 << TWEN) | (1 << TWINT);
  // Wait for the acknowledgement
  while ((TWSR & 0xF8) != 0xC0)
    ;
}

// Function to match the slave address and slave
void TWI_match_write_slave(void)
{
  // Loop till correct acknowledgement have been received
  while ((TWSR & 0xF8) != 0xA8)
  {
    // Get acknowledgment, Enable TWI, Clear TWI interrupt flag
    TWCR = (1 << TWEA) | (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)))
      ; // Wait for TWINT flag
  }
}

void TWI_read_slave(void)
{
  // Clear TWI interrupt flag,Get acknowledgement, Enable TWI
  TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN);
  // Wait for TWINT flag
  while (!(TWCR & (1 << TWINT)))
    ;
  // Wait for acknowledgement
  while ((TWSR & 0xF8) != 0x80)
    ;
  // Get value from TWDR
  data_receive = TWDR;
}

//Function to match the slave address and slave direction bit(read)
void TWI_match_read_slave(void) //Function to match the slave address and slave dirction bit(read)
{
  while ((TWSR & 0xF8) != 0x60) // Loop till correct acknoledgement have been received
  {
    // Get acknowlegement, Enable TWI, Clear TWI interrupt flag
    TWCR = (1 << TWEA) | (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)))
      ; // Wait for TWINT flag
  }
}

int setup(void)
{
  TWI_init_slave(); // Function to initialize slave
  while (1)
  {
    //Function to match the slave address and slave direction bit(read)
    TWI_match_read_slave();
    // Function to read data
    TWI_read_slave();

    //Function to match the slave address and slave direction bit(write)
    TWI_match_write_slave();
    // Function to write data
    TWI_write_slave();
  }
}

//1 FEND 0xc0
//2 Destination 8 bits
//3 Source 8 bits
//4 Type 2 bits
//5 Type 6 bits
//6 Data 8 bits
//7 CRC  16 bits
//8 CRC  16 bits
//9 FEND 0xc0

//FEND in raw data then
//FESC 0xDB
//TFEND 0xDC

//FESC in raw data then
//FESC 0xDB
//TFESC 0xDD

void SSP_I2C_receive()
{
  uint16_t CRC = 0;
  uint8_t CRCH = 0;
  uint8_t CRCL = 0;
  uint8_t temp_data_crc1 = 0;
  uint16_t temp_data_crc2 = 0;
  uint8_t i = 0;
  uint8_t FLAG_FEND = 0;
  uint8_t FLAG_DEST = 0;
  uint8_t FLAG_SRC = 0;
  uint8_t FLAG_TYPE = 0;
  uint8_t FLAG_DATA = 0;
  uint8_t FLAG_CRC1 = 0;
  uint8_t FLAG_CRC2 = 0;
  for (i = 0; i < 8; i++)
  {
    setup();
    switch (i)
    {
    case 0:
      if (data_receive == 0xc0)
      {
        FLAG_FEND = 1; //else error;
      }
      break;
    case 1:
      if (data_receive == I2C_ADDR)
      {
        FLAG_DEST = 1; //EPS is DEST //else error;
      }
      break;
    case 2:
      if (data_receive == 0x03)
      {
        FLAG_SRC = 1; //OBC is SRC //else error;
      }
      break;
    case 3:
      if (data_receive == 0x70)
      {
        FLAG_TYPE = 1; //TYPE Command is Correct //else error;
      }
    case 4:
      if (data_receive == 0xf1)
      {
        temp_data_crc1 = data_receive;
        data_receive_main = data_receive;
        FLAG_DATA = 1; //DATA Command is Correct //else error;
      }
      else if (data_receive = 0xdd)
      {
        data_receive = 0xdb;
        temp_data_crc1 = data_receive;
        FLAG_DATA = 1; //else error;
      }
      else if (data_receive = 0xdc)
      {
        data_receive = 0xc0;
        temp_data_crc1 = data_receive;
        FLAG_DATA = 1; //else error;
      }
      break;
    case 5:
      CRC = CRCL | data_receive;
      CRC = 8 << CRC;
      setup();
      CRC = CRCH | data_receive;
      CRC = _crc_ccitt_update(CRC, temp_data_crc1);
      if (CRC == 0)
        FLAG_CRC1 = 1; //else error;
      CRC = temp_data_crc2;
      break;
    case 6:
      CRC = CRCL | data_receive;
      CRC = 8 << CRC;
      setup();
      CRC = CRCH | data_receive;
      CRC = _crc_ccitt_update(CRC, temp_data_crc2); //CRC FROM CRC itself need work
      if (CRC == 0)
        FLAG_CRC2 = 1; //else error;
      break;
    case 7:
      if (data_receive == 0xc0 && FLAG_FEND == 1)
      {
        FLAG_FEND = 0; //else error; //sentence is arbitrary doesn't effect
      }
      break;
    }
  }
}

void SSP_I2C_send()
{
  uint16_t CRC = 0;
  uint8_t CRCH = 0;
  uint8_t CRCL = 0;
  uint8_t temp_data_crc1 = 0;
  uint16_t temp_data_crc2 = 0;
  uint8_t i = 0;
  for (i = 0; i < 8; i++)
  {
    setup();
    switch (i)
    {
    case 0:
      data_send = 0xc0;
      break;
    case 1:
      data_send = 0x03; //OBC is DEST
      break;
    case 2:
      data_send = I2C_ADDR; //EPS is SRC
      break;
    case 3:
      data_send = 0x01; //TYPE Command
      break;
    case 4:
      data_send = 0x02; //Telemetry Data according to type
      data_send_main = data_send;
      if (data_send = 0xc0)
      {
        data_send = 0xdb;
        setup();
        data_send = 0xdc;
      }
      else if (data_send = 0xdb)
      {
        data_send = 0xdb;
        setup();
        data_send = 0xdd;
      }
      temp_data_crc1 = data_send;
      break;
    case 5:
      CRC = _crc_ccitt_update(0xffff, temp_data_crc1);
      CRCL = CRC & 0xff;
      CRCH = CRC >> 8;
      data_send = CRCL;
      setup();
      data_send = CRCH;
      CRC = temp_data_crc2;
      break;
    case 6:
      CRC = _crc_ccitt_update(0xffff, temp_data_crc2); //CRC FROM CRC itself need work
      CRCL = CRC & 0xff;
      CRCH = CRC >> 8;
      data_send = CRCL;
      setup();
      data_send = CRCH;
      break;
    case 7:
      data_send = 0xc0;
      break;
    }
  }
}
//******-------------- -END------------------******//

//******-----------ADC_INTERNAL------------******//
void ADC_init()
{
  ADMUX |= (1 << REFS0); // AVcc with external capacitor at AREF
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

//******************************************************//
uint8_t ADC_read(uint8_t ch)
{
  if (ch < 8)
  {
    ch = ch & 0b00000111;
    ADMUX |= ch;
    ADCSRA |= (1 << ADSC); // start conversion
    ADCSRB &= 0b11110111;
    while (!(ADCSRA & (1 << ADIF)))
      ;                    // waiting for ADIF, conversion complete
    ADCSRA |= (1 << ADIF); // clearing of ADIF, it is done by writing 1 to it
    uint8_t x = ADCL;
    ADMUX &= 0b11111000;

    return x;
  }
  else
  {
    ch = ch - 8;
    ch = ch & 0b00000111;
    ADMUX |= ch;
    ADCSRB |= 0b00001000;
    ADCSRA |= (1 << ADSC);
    // start conversion
    while (!(ADCSRA & (1 << ADIF)))
      ;                    // waiting for ADIF, conversion complete
    ADCSRA |= (1 << ADIF); // clearing of ADIF, it is done by writing 1 to it
    uint8_t x = ADCL;
    ADMUX &= 0b11111000;
    return x;
  }
}
//******----------------END------------------******//

//******------------EXTERNAL_ADC2------------******//
int readADC_2(uint8_t adcnum)
{

  if ((adcnum > 7) || (adcnum < 0))
    return -1; // Wrong adc address return -1

  PORTC |= (1 << CS2);
  PORTC &= ~(1 << CLK2); // low
  PORTC &= ~(1 << CS2);

  uint8_t commanDOUT1 = adcnum;
  commanDOUT1 |= 0x18; //  # start bit + single-ended bit
  commanDOUT1 <<= 3;   //    # we only need to send 5 bits here
  uint8_t i;
  for (i = 0; i < 5; i++)
  {
    if (commanDOUT1 & 0x80)
      PORTC |= (1 << DIN2);
    else
      PORTC &= ~(1 << DIN2);

    commanDOUT1 <<= 1;
    PORTC |= (1 << CLK2);
    PORTC &= ~(1 << CLK2);
  }

  uint8_t adcout = 0;
  // read in one empty bit, one null bit and 10 ADC bits
  i = 0;
  for (i = 0; i < 12; i++)
  {
    PORTC |= (1 << CLK2);
    PORTC &= ~(1 << CLK2);
    adcout <<= 1;
    if (PINC & 0b00001000) //CHECKING DOUT11111111
      adcout |= 0x1;
  }
  PORTC |= (1 << CS2);

  adcout >>= 1; //      # first bit is 'null' so drop it

  return adcout;
}
//******-------------- -END------------------******//

//******------------EXTERNAL_ADC1------------******//
int readADC_1(uint8_t adcnum)
{

  if ((adcnum > 7) || (adcnum < 0))
    return -1; // Wrong adc address return -1

  PORTC |= (1 << CS1);
  PORTC &= ~(1 << CLK1); // low
  PORTC &= ~(1 << CS1);

  int commanDOUT1 = adcnum;
  commanDOUT1 |= 0x18; //  # start bit + single-ended bit
  commanDOUT1 <<= 3;   //    # we only need to send 5 bits here
  uint8_t i;
  for (i = 0; i < 5; i++)
  {
    if (commanDOUT1 & 0x80)
      PORTC |= (1 << DIN1);
    else
      PORTC &= ~(1 << DIN1);

    commanDOUT1 <<= 1;
    PORTC |= (1 << CLK1);
    PORTC &= ~(1 << CLK1);
  }

  uint8_t adcout = 0;
  // read in one empty bit, one null bit and 10 ADC bits
  for (uint8_t i = 0; i < 12; i++)
  {
    PORTC |= (1 << CLK1);
    PORTC &= ~(1 << CLK1);
    adcout <<= 1;
    if (PINC & 0b10000000) //CHECKING DOUT11111111
      adcout |= 0x1;
  }
  PORTC |= (1 << CS1);

  adcout >>= 1; //      # first bit is 'null' so drop it

  return adcout;
}
//******-------------- -END------------------******//

//16-bit Timer/Counter (Timer/Counter 1, 3, 4, and 5)
//******------------PWM Frequency------------******//
void setPwmFrequency(uint16_t divisor)
{

  uint8_t mode;
  {
    switch (divisor)
    {
    case 1:
      mode = 0x01;
      break;
    case 8:
      mode = 0x02;
      break;
    case 32:
      mode = 0x03;
      break;
    case 64:
      mode = 0x04;
      break;
    case 128:
      mode = 0x05;
      break;
    case 256:
      mode = 0x06;
      break;
    case 1024:
      mode = 0x07;
      break;
    default:
      return;
      TCCR3B = TCCR3B & 0b11111000 | mode;
      // set frequency PWM
      TCCR3A |= (1 << WGM33) | (1 << WGM32) | (1 << WGM31) | (1 << WGM30);
      // set fast PWM
    }
  }
}

//******------------PWM------------******//
void PWM_(uint8_t d, uint8_t l)
{
  if (l == X)
  {
    DDRE |= (1 << PE3);
    // PB3 is now an output
    TCCR3A |= (1 << COM3A1);
    // set none-inverting mode
    OCR3A = d;
    // set PWM duty cycle
  }
  else if (l == Y)
  {
    DDRE |= (1 << PE4);
    // PB4 is now an output
    TCCR3A |= (1 << COM3B1);
    // set none-inverting mode
    OCR3B = d;
    // set PWM duty cycle
  }
  else if (l == Z)
  {
    DDRE |= (1 << PE5);
    // PB4 is now an output
    TCCR3A |= (1 << COM3C1);
    // set none-inverting mode
    OCR3C = d;
    // set PWM duty cycle
  }
}
//******-------------- -END------------------******//

//*******************MAIN CORE***********************//
int main(void)
{
  //*******************LOADS CONTROL INITALIZATION***********************//
  TWAR = 0b10010000;
  SSP_I2C_receive();
  ADC_init();
  DDRA = 0xff;
  DDRC = 0b01110111; // SETTING APPROPRIATE INPUTS AND OUTPUTS

  //*******************MPPT INITALIZATION********************************//
  /*
Pin ADC internal numbers:
IIN_X is 8  input
IIN_Y is 9  input 
IIN_Z is 10 input

Pin ADC1 external numbers:
M_SA_X is 7 output

Pin ADC2 external numbers:
M_SA_Y is 0 output
M_SA_Z is 1 output
ISA_X is 2 output
ISA_Y is 3 output
ISA_Z is 4 output
VDC_X is 5 input
VDC_Y is 6 input
VDC_Z is 7 input

*/
  float Iout;
  float Vout;
  float nextpower;
  float nextVin;
  float nextIin;
  uint8_t duty;
  int stepsize;
  int valueread_vout;
  int valueread_iout;
  int sample;
  uint8_t pinread_vout;
  uint8_t pinread_iout;
  int interval;
  float stepsize_factor;

  uint8_t LOAD;

  const uint8_t pinread_vin_x = 5;  //VDC_X is 5 input
  const uint8_t pinread_vout_x = 7; //M_SA_X is 7 output only ADC1 external
  const uint8_t pinread_iin_x = 8;  //IIN_X is 8  input
  const uint8_t pinread_iout_x = 2; //ISA_X is 2 output

  const uint8_t pinread_vin_y = 6;  //VDC_Y is 6 input
  const uint8_t pinread_vout_y = 0; //M_SA_Y is 0 output
  const uint8_t pinread_iin_y = 9;  //IIN_Y is 9  input
  const uint8_t pinread_iout_y = 3; //ISA_Y is 3 output

  const uint8_t pinread_vin_z = 7;  //VDC_Z is 7 input
  const uint8_t pinread_vout_z = 1; //M_SA_Z is 1 output
  const uint8_t pinread_iin_z = 10; //IIN_Z is 10  input
  const uint8_t pinread_iout_z = 4; //ISA_Z is 4 output

  uint8_t duty_x = 100;
  uint16_t divisor_x = 1;

  uint8_t duty_y = 100;
  uint16_t divisor_y = 1;

  uint8_t duty_z = 100;
  uint16_t divisor_z = 1;

  int sample_x = 1;
  int interval_x = 1000;

  int sample_y = 1;
  int interval_y = 1000;

  int sample_z = 1;
  int interval_z = 1000;

  int valueread_vin_x = 200;
  int valueread_vout_x = 200;
  int valueread_iin_x = 200;
  int valueread_iout_x = 200;

  int valueread_vin_y = 200;
  int valueread_vout_y = 200;
  int valueread_iin_y = 200;
  int valueread_iout_y = 200;

  int valueread_vin_z = 200;
  int valueread_vout_z = 200;
  int valueread_iin_z = 200;
  int valueread_iout_z = 200;

  float Iin_x = 1.0;
  float nextIin_x = 1.0;
  float prevIin_x = 2.0;

  float Iin_y = 1.0;
  float nextIin_y = 1.0;
  float prevIin_y = 2.0;

  float Iin_z = 1.0;
  float nextIin_z = 1.0;
  float prevIin_z = 2.0;

  float Vin_x = 2.0;
  float prevVin_x = 3.0;
  float nextVin_x = 1.0;

  float Vin_y = 2.0;
  float prevVin_y = 3.0;
  float nextVin_y = 1.0;

  float Vin_z = 2.0;
  float prevVin_z = 3.0;
  float nextVin_z = 1.0;

  float Vout_x = 1.0;
  float Iout_x = 1.0;

  float Vout_y = 1.0;
  float Iout_y = 1.0;

  float Vout_z = 1.0;
  float Iout_z = 1.0;

  float Pin_x = 2.0;
  float prevpower_x = 3.0;
  float nextpower_x = 1.0;

  float Pin_y = 2.0;
  float prevpower_y = 3.0;
  float nextpower_y = 1.0;

  float Pin_z = 2.0;
  float prevpower_z = 3.0;
  float nextpower_z = 1.0;

  float Pout_x = 1.0;
  float Pout_y = 1.0;
  float Pout_z = 1.0;

  float offsetvoltage_i = 2.5;
  float scale_i = 1.75;
  float scale_v = 0.5;

  float stepsize_factor_x = 1.5;
  int stepsize_x = 1;

  float stepsize_factor_y = 1.5;
  int stepsize_y = 1;

  float stepsize_factor_z = 1.5;
  int stepsize_z = 1;

  setPwmFrequency(1);
  //****************************END**************************************//

  //*********************MAIN LOOP*************************//
  while (1)
  {

    //*********************CORE LOAD CONTROL*************************//

    SSP_I2C_receive();
    if (TWDR == 'J')
    {
      uint8_t i = 0;
      for (i = 0; i < 8; i++)
      {
        int val = 0;
        uint8_t ADC_ext = 0;

        SSP_I2C_receive();

        val = readADC_1(i);
        ADC_ext = val / 4;
        data_send_main = ADC_ext;
      }
    }

    if (TWDR == 'B')
    {
      uint8_t i = 0;
      for (i = 0; i < 16; i++)
      {

        SSP_I2C_receive();

        ADC_init();
        ADC_read(i);

        data_send_main = ADCL;
      }
    }

    else if (TWDR == 'F')
    {

      uint8_t i = 0;
      for (i = 0; i < 24; i++)
      {

        SSP_I2C_receive();

        ADC_init();
        ADC_read(i);

        data_send_main = ADCL;
      }

      i = 0;
      for (i = 0; i < 8; i++)
      {
        int val = 0;
        uint8_t ADC_ext = 0;

        SSP_I2C_receive();

        val = readADC_1(i);
        ADC_ext = val / 4;
        data_send_main = ADC_ext;
      }
    }

    else if (TWDR == 'G')
    {
      uint8_t i = 0;
      for (i = 0; i < 16; i++)
      {
        SSP_I2C_receive();
        ADC_init();
        ADC_read(i);
        data_send_main = ADCH;
      }

      i = 0;
      for (i = 0; i < 8; i++)
      {
        int val = 0;
        uint8_t ADC_ext = 0;

        SSP_I2C_receive();

        val = readADC_1(i);
        ADC_ext = val / 4;
        data_send_main = ADC_ext;
      }

      i = 0;
      for (i = 0; i < 8; i++)
      {
        int val = 0;
        uint8_t ADC_ext = 0;

        val = readADC_2(i);
        ADC_ext = val / 4;
        data_send_main = ADC_ext;
      }
    }
    //////***********Digital pins**********//////

    else if (TWDR == 'R') //*******PA0*******//ON
    {
      PORTA |= (1 << PA0);
    }

    else if (TWDR == 'r') //*******PA0*******//Off
    {
      PORTA &= ~(1 << PA0);
    }
    //***********************************//

    else if (TWDR == 'L')
    {
      PORTA |= (1 << PA1);
    }
    else if (TWDR == 'l')
    {
      PORTA &= ~(1 << PA1);
    }
    //***********************************//

    else if (TWDR == 'K')
    {
      PORTA |= (1 << PA2);
    }
    else if (TWDR == 'k')
    {
      PORTA &= ~(1 << PA2);
    }
    //***********************************//
    else if (TWDR == 'Q')
    {
      PORTA |= (1 << PA3);
    }
    else if (TWDR == 'q')
    {
      PORTA &= ~(1 << PA3);
    }
    //***********************************//

    else if (TWDR == 'W')
    {
      PORTA |= (1 << PA4);
    }
    else if (TWDR == 'w')
    {
      PORTA &= ~(1 << PA4);
    }
    //***********************************//

    else if (TWDR == 'M')
    {
      PORTA |= (1 << PA5);
    }
    else if (TWDR == 'm')
    {
      PORTA &= ~(1 << PA5);
    }
    //***********************************//

    else if (TWDR == 'N')
    {
      PORTA |= (1 << PA6);
    }
    else if (TWDR == 'n')
    {
      PORTA &= ~(1 << PA6);
    }
    //***********************************//

    else if (TWDR == 'T')
    {
      PORTA |= (1 << PA7);
    }
    else if (TWDR == 't')
    {
      PORTA &= ~(1 << PA7);
    }
    //***********************************//

    else if (TWDR == 'H')
    {
      PORTA = 0xff;
    }
    else if (TWDR == 'h')
    {
      PORTA = 0x00;
    }
    //***********************************//

    //****************************END**************************************//
    //*********************CORE MPPT CONTROL*************************//

    ADC_init();
    //Line_X
    valueread_vin_x = 0;
    valueread_vout_x = 0;
    valueread_iin_x = 0;
    valueread_iout_x = 0;
    //Line_Y
    valueread_vin_y = 0;
    valueread_vout_y = 0;
    valueread_iin_y = 0;
    valueread_iout_y = 0;
    //Line_Z
    valueread_vin_z = 0;
    valueread_vout_z = 0;
    valueread_iin_z = 0;
    valueread_iout_z = 0;

    for (int i = 0; i < sample_x; i++)
    {
      //Line_X
      valueread_vin_x += readADC_2(pinread_vin_x); // read and accumulate the solar panel voltage
      _delay_us(interval_x);
      valueread_vout_x += readADC_1(pinread_vout_x); // read and accumulate the load voltage
      _delay_us(interval_x);
      ADC_init();
      valueread_iin_x += ADC_read(pinread_iin_x); // read and accumulate the solar panel current
      _delay_us(interval_x);
      valueread_iout_x += readADC_2(pinread_iout_x); // read and accumulate the load voltage
      _delay_us(interval_x);
    }
    for (int i = 0; i < sample_y; i++)
    {
      //Line_Y
      valueread_vin_y += readADC_2(pinread_vin_y); // read and accumulate the solar panel voltage
      _delay_us(interval_y);
      valueread_vout_y += readADC_2(pinread_vout_y); // read and accumulate the load voltage
      _delay_us(interval_y);
      ADC_init();
      valueread_iin_y += ADC_read(pinread_iin_y); // read and accumulate the solar panel current
      _delay_us(interval_y);
      valueread_iout_y += readADC_2(pinread_iout_y); // read and accumulate the load voltage
      _delay_us(interval_y);
    }
    for (int i = 0; i < sample_z; i++)
    {
      //Line_Z
      valueread_vin_z += readADC_2(pinread_vin_z); // read and accumulate the solar panel voltage
      _delay_us(interval_z);
      valueread_vout_z += readADC_2(pinread_vout_z); // read and accumulate the load voltage
      _delay_us(interval_z);
      ADC_init();
      valueread_iin_z += ADC_read(pinread_iin_z); // read and accumulate the solar panel current
      _delay_us(interval_z);
      valueread_iout_z += readADC_2(pinread_iout_z); // read and accumulate the load voltage
      _delay_us(interval_z);
    }
    ///////////////////////

    //Line_X
    Vin_x = ((valueread_vin_x * 5) / (1024.0 * sample_x)) / scale_v;
    Vout_x = ((valueread_vout_x * 5) / (1024.0 * sample_x)) / scale_v;
    Iin_x = (valueread_iin_x / (1024.0 * sample_x)) * (5 / scale_i);
    Iout_x = (valueread_iout_x / (1024.0 * sample_x)) * (5 / scale_i);

    //Line_Y
    Vin_y = ((valueread_vin_y * 5) / (1024.0 * sample_y)) / scale_v;
    Vout_y = ((valueread_vout_y * 5) / (1024.0 * sample_y)) / scale_v;
    Iin_y = (valueread_iin_y / (1024.0 * sample_y)) * (5 / scale_i);
    Iout_y = (valueread_iout_y / (1024.0 * sample_y)) * (5 / scale_i);

    //Line_Z
    Vin_z = ((valueread_vin_z * 5) / (1024.0 * sample_z)) / scale_v;
    Vout_z = ((valueread_vout_z * 5) / (1024.0 * sample_z)) / scale_v;
    Iin_z = (valueread_iin_z / (1024.0 * sample_z)) * (5 / scale_i);
    Iout_z = (valueread_iout_z / (1024.0 * sample_z)) * (5 / scale_i);

    stepsize_x = ((abs(Vout - 7.5) * stepsize_factor_x)) + 1;
    stepsize_y = ((abs(Vout - 7.5) * stepsize_factor_y)) + 1;
    stepsize_z = ((abs(Vout - 7.5) * stepsize_factor_z)) + 1;
    //Line_X
    Pout_x = Vout_x * Iout_x;
    //Line_Y
    Pout_y = Vout_y * Iout_y;
    //Line_Z
    Pout_z = Vout_z * Iout_z;

    //Line_X
    Pin_x = Vin_x * Iin_x;

    nextpower_x = Pin_x - prevpower_x;
    prevpower_x = Pin_x;

    nextIin_x = Iin_x - prevIin_x;
    prevIin_x = Iin_x;

    nextVin_x = Vin_x - prevVin_x;
    prevVin_x = Vin_x;

    //Line_Y
    Pin_y = Vin_y * Iin_y;

    nextpower_y = Pin_y - prevpower_y;
    prevpower_y = Pin_y;

    nextIin_y = Iin_y - prevIin_y;
    prevIin_y = Iin_y;

    nextVin_y = Vin_y - prevVin_y;
    prevVin_y = Vin_y;

    //Line_Z
    Pin_z = Vin_z * Iin_z;

    nextpower_z = Pin_z - prevpower_z;
    prevpower_z = Pin_z;

    nextIin_z = Iin_z - prevIin_z;
    prevIin_z = Iin_z;

    nextVin_z = Vin_z - prevVin_z;
    prevVin_z = Vin_z;

    //////////////////////
    for (int LINES = 1; LINES < 4; LINES++)
    {
      switch (LINES)
      {
      case 1: //Line_X
        Iout = Iout_x;
        Vout = Vout_x;
        nextpower = nextpower_x;
        nextVin = nextVin_x;
        nextIin = nextIin_x;
        duty = duty_x;
        stepsize = stepsize_x;
        valueread_vout = valueread_vout_x;
        valueread_iout = valueread_iout_x;
        sample = sample_x;
        pinread_vout = pinread_vout_x;
        pinread_iout = pinread_iout_x;
        interval = interval_x;
        stepsize_factor = stepsize_factor_x;
        LOAD = X;
        break;
      case 2: //Line_Y
        Iout = Iout_y;
        Vout = Vout_y;
        nextpower = nextpower_y;
        nextVin = nextVin_y;
        nextIin = nextIin_y;
        duty = duty_y;
        stepsize = stepsize_y;
        valueread_vout = valueread_vout_y;
        valueread_iout = valueread_iout_y;
        sample = sample_y;
        pinread_vout = pinread_vout_y;
        pinread_iout = pinread_iout_y;
        interval = interval_y;
        stepsize_factor = stepsize_factor_y;
        LOAD = Y;
        break;
      case 3: //Line_Z
        Iout = Iout_z;
        Vout = Vout_z;
        nextpower = nextpower_z;
        nextVin = nextVin_z;
        nextIin = nextIin_z;
        duty = duty_z;
        stepsize = stepsize_z;
        valueread_vout = valueread_vout_z;
        valueread_iout = valueread_iout_z;
        sample = sample_z;
        pinread_vout = pinread_vout_z;
        pinread_iout = pinread_iout_z;
        interval = interval_z;
        stepsize_factor = stepsize_factor_z;
        LOAD = Z;
        break;
      }
      if (Iout < 2)
      {
        if (Vout < 7.5)
        {
          if (nextpower != 0.000)
          {
            if (nextpower > 0.000)
            {
              if (nextVin > 0.000)
              {
                if (nextIin > 0.000)
                {
                  duty = duty + stepsize;
                  if (duty > 255)
                  {
                    duty = 254;
                  }
                }
                else
                {
                  duty = duty - stepsize;
                  if (duty < 1)
                  {
                    duty = 1;
                  }
                }
              }
              else
              {
                duty = duty + stepsize;
                if (duty > 255)
                {
                  duty = 254;
                }
              }
            }
            else
            {
              if (nextVin > 0.000)
              {
                duty = duty + stepsize;
                if (duty > 255)
                {
                  duty = 254;
                }
              }
              else
              {
                duty = duty - stepsize;
                if (duty < 1)
                {
                  duty = 1;
                }
              }
            }
          }
          else
          {
            duty = duty;
          }
        }
        else if (Vout > 7.5)
        {
          while (Vout > 7.5)
          {
            valueread_vout = 0;
            for (int i = 0; i < sample; i++)
            {
              ADC_init();
              valueread_vout += ADC_read(pinread_vout); // read and accumulate the load voltage
              _delay_us(interval);
            }
            Vout = ((valueread_vout * 5) / (1024.0 * sample)) / scale_v;
            stepsize = ((abs(Vout - 7.5) * stepsize_factor)) + 1;
            duty = duty + stepsize;
            if (duty > 255)
            {
              duty = 254;
            }
            PWM_(duty, LOAD);
          }
        }
        else
        {
          duty = duty;
        }
      }
      else if (Iout > 2)
      {
        while (Iout > 2)
        {
          valueread_iout = 0;
          for (int i = 0; i < sample; i++)
          {
            ADC_init();
            valueread_iout += ADC_read(pinread_iout); // read and accumulate the load voltage
            _delay_us(interval);
          }
          Iout = (valueread_iout / (1024.0 * sample)) * 5;
          stepsize = ((abs(Vout - 7.5) * stepsize_factor)) + 1;
          duty = duty + stepsize;
          if (duty > 255)
          {
            duty = 254;
          }
          PWM_(duty, LOAD);
        }
      }
      else
      {
        duty = duty;
      }

      PWM_(duty, LOAD);
    }

    SSP_I2C_receive();
    SSP_I2C_send();

    // KILL SWITCH COMMAND WILL BE COMPLEMENTED LOGIC INSIDE TYPE COMMAND OF CASE 3 IN SSP_I2C_RECEIVE FUNCTION
  }
}
//****************************END**************************************//
