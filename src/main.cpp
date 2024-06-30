#include <Arduino.h>
#include "lcd.c"

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include <util/twi.h>

#include "adc.c"

#define ADC_PORT PORTC
#define ADC_DDR DDRC

#define ADC_PIN_0  0
#define ADC_PIN_1  1

volatile uint16_t ADC_Wert0 = 0;
volatile uint16_t ADC_Wert1 = 0;

#define LOOPLEDPORT		PORTD
#define LOOPLEDDDR		DDRD
#define LOOPSTEP			0x08FF



// Define fuer Slave:
#define LOOPLED			4

#define SPI_CONTROL_DDR			DDRB
#define SPI_CONTROL_PORT		PORTB
#define SPI_CONTROL_CS			PORTB2	//CS fuer HomeCentral Master
#define SPI_CONTROL_MOSI		PORTB3
#define SPI_CONTROL_MISO		PORTB4
#define SPI_CONTROL_SCK			PORTB5

#define SPI_CS_HI     SPI_CONTROL_PORT |= (1<<SPI_CONTROL_CS)
#define SPI_CS_LO     SPI_CONTROL_PORT &= ~(1<<SPI_CONTROL_CS)

#define BUFSIZE 8

volatile unsigned char incoming[BUFSIZE];
volatile short int received=0;
volatile uint8_t spistatus = 0;
#define RECEIVED	0

#define SPI_WAIT 0xFF
#define BUFSIZE 8
uint16_t loopcount0=0;
uint16_t loopcount1=0;
uint8_t loopcount2=0;

uint16_t spicounter=0;
uint8_t spicheck = 0; // laufvariable
volatile uint8_t transferindex = 0; // pos von data auf SPI
volatile uint8_t out_data[BUFSIZE];
volatile uint8_t in_data[BUFSIZE];


unsigned char spi_tranceiver (unsigned char data)
{

    // Load data into the buffer
    SPDR = data;
    spicounter = 0;
    //Wait until transmission complete
    while(!(SPSR & (1<<SPIF)))
    {
      ;
      /*
      spicounter++;
      if (spicounter >= SPI_WAIT)
      return 0;   // Return received data
      */
    }

  return(SPDR);
}

uint8_t received_from_spi(uint8_t data)
{
  SPDR = data;
  return SPDR;
}
#include <avr/io.h>

void SPI_MasterInit(void) {
    // Set MOSI, SCK and SS as output, MISO as input
    DDRB = (1<<DDB3)|(1<<DDB5)|(1<<DDB2);
    
    // Enable SPI, set as master, and set clock rate fck/4
    SPCR = (1<<SPE)|(1<<MSTR);
}

void SPI_MasterTransmit(char cData) 
{
    // Start transmission by loading data into the SPI data register
    SPDR = cData;
    // Wait for transmission to complete (SPIF flag set)
    while(!(SPSR & (1<<SPIF)))
    {
      ;
      /*
      spicounter++;
      if (spicounter >= SPI_WAIT)
      return ;   // Return received data
      */
    }
}



void setup() 
{
  LOOPLEDDDR |= (1<<LOOPLED);		//Pin 5 von PORT D als Ausgang fuer LED Loop
	LOOPLEDPORT |= (1<<LOOPLED);		//Pin 5 von PORT D als Ausgang fuer LED Loop
  SPI_MasterInit();
  
  // ADC
ADC_DDR &= ~(1<<ADC_PIN_0);
ADC_DDR &= ~(1<<ADC_PIN_1);

initADC(ADC_PIN_0);

#define COMP_ADC_DDR DDRC

#define COMP_ADC_PIN_A  4
#define COMP_ADC_PIN_B  5

  
  //LCD 
	LCD_DDR |= (1<<LCD_RSDS_PIN);	//Pin 4 von PORT B als Ausgang fuer LCD
 	LCD_DDR |= (1<<LCD_ENABLE_PIN);	//Pin 5 von PORT B als Ausgang fuer LCD
	LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin 6 von PORT B als Ausgang fuer LCD

	  // initialize the LCD 
	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
	
	lcd_gotoxy(0,0);
	lcd_puts("SPI LCD Master");
 	
	  _delay_ms(1000);

}
void parse_message()
{

 switch(incoming[0]) 
 {
 case 2:
   //flash_led(incoming[1])
	;break;
 default:
   PORTD ^=(1<<1);//LED 1 toggeln
	;
 }
 out_data[2] = 111;
 out_data[3] = 112;
  out_data[4] = 221;
 out_data[5] = 222;
}


void loop() 
{
  loopcount0++;
	if (loopcount0>LOOPSTEP)
	{
			//PORTB ^= (1<<6);
			//PORTB ^= (1<<7);

			loopcount0 = 0;
			 

      loopcount1++;
      if(loopcount1 > 0x2F)
        {
          
          ADC_Wert0 = readKanal(ADC_PIN_0) >> 2;
          out_data[0] = ADC_Wert0;
          ADC_Wert1 = readKanal(ADC_PIN_1) >> 2;
          out_data[1] = ADC_Wert1;

          transferindex &= 0x07;
          out_data[6] = transferindex;
          out_data[7] = 0xFF;
          uint8_t paketnummer = transferindex%4;
          
            
              SPI_CS_LO;
              SPI_MasterTransmit(out_data[2*paketnummer]); 
              SPI_CS_HI;
              _delay_us(2);
              SPI_CS_LO;
              SPI_MasterTransmit(out_data[2*paketnummer+1]); 
              SPI_CS_HI;
            
            transferindex++;
          

          /*
          switch (transferindex)
          {
            case 0:
            {
              SPI_CS_LO;
              SPI_MasterTransmit(ADC_Wert0); 
              SPI_CS_HI;
            }break;
            case 1:
            {
              SPI_CS_LO;
              SPI_MasterTransmit(ADC_Wert1); 
              SPI_CS_HI;
            }break;
            case 2:
            {
              SPI_CS_LO;
              SPI_MasterTransmit(11); 
              SPI_CS_HI;

            }break;

            case 3:
            {
              SPI_CS_LO;
              SPI_MasterTransmit(22); 
              SPI_CS_HI;
            }break;
            case 4:
            {
              SPI_CS_LO;
              SPI_MasterTransmit(33); 
              SPI_CS_HI;
            }break;
            case 5:
            {
              SPI_CS_LO;
              SPI_MasterTransmit(44); 
              //in_data[transferindex] =  spi_tranceiver[44];
              SPI_CS_HI;
            }break;
            case 6:
            {
              SPI_CS_LO;
              SPI_MasterTransmit(spicheck++); 
              SPI_CS_HI;
            }break;
            case 7:
            {
              
              SPI_CS_LO;
              SPI_MasterTransmit(0xFF); 
              SPI_CS_HI;
              //transferindex = 0;
            }break;

          }// switch
          */
          
            LOOPLEDPORT ^=(1<<LOOPLED);
            loopcount1 = 0;

            lcd_gotoxy(16,0);
            lcd_putint(transferindex);
            lcd_gotoxy(16,1);
            lcd_putint(in_data[5]);
            loopcount2++;

            transferindex++;
 
        }
  }// loopcount

}

