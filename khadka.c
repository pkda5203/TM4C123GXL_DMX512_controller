/*
 * Name  : Prabesh Khadka
 * UTA ID: 1001201007
 *
 *
 * CSE4342 Embedded Systems II
 * Spring 2019 Project
 *
 *
 * Hardware initialization code adopted from example code provided by Dr. Losh
 */

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red LED connected to PF1
// Green LED connected to PF3
// UART Interface:
//  U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//  Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"


#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define PIN_U1TX     (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 5*4)))
#define PIN_DEN    (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4)))

#define GREEN_LED_MASK 8
#define RED_LED_MASK 2
#define MAX_CHAR 80        //define max character allowed for the commands
#define delay4Cycles() __asm(" NOP\n NOP\n NOP\n NOP")


//https://stackoverflow.com/questions/9146395/reset-c-int-array-to-zero-the-fastest-way
#define ZERO_ANY(T, a, n) do{\
T *a_ = (a);\
size_t n_ = (n);\
for (; n_ > 0; --n_, ++a_)\
*a_ = (T) { 0 };\
} while (0)

#define ARRAY_SIZE(a) (sizeof (a) / sizeof *(a))
#define ZERO_ANY_A(T, a) ZERO_ANY(T, (a), ARRAY_SIZE(a))

//-----------------------------------------------------------------------------
//Global Variables
//-----------------------------------------------------------------------------
uint16_t maxM = 512;
uint8_t dmxData[512];
uint8_t fieldCount;
uint8_t pos[MAX_CHAR+1];
uint16_t deviceAddress =1; //have to write to eeprom 1 by default
uint8_t ON =0;      //DMX stream initialized to off //have to write to EEPROM
uint8_t MODE= 0;              //define a global variable for controller mode (0: Controller and 1: Device) //have to write to eeprom
uint16_t phase;
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

//code adopted from Dr. losh's provided example
// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A and F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOC;

    // Configure LED pins
    GPIO_PORTF_DIR_R = GREEN_LED_MASK | RED_LED_MASK;  // bits 1 and 3 are outputs
    GPIO_PORTF_DR2R_R = GREEN_LED_MASK | RED_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = GREEN_LED_MASK | RED_LED_MASK;  // enable LEDs

    // Configure UART0 pins
    GPIO_PORTA_DIR_R |= 2;                           // enable output on UART0 TX pin: default, added for clarity
    GPIO_PORTA_DEN_R |= 3;                           // enable digital on UART0 pins: default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // use peripheral to drive PA0, PA1: default, added for clarity
    GPIO_PORTA_PCTL_R &= 0xFFFFFF00;                 // set fields for PA0 and PA1 to zero
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;
    // select UART0 to drive pins PA0 and PA1: default, added for clarity

    // Configure UART0 to 115200 baud, 8N1 format
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other UARTs in same status
    delay4Cycles();                                  // wait 4 clock cycles
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
    // enable TX, RX, and module

    //configure UART1 pins
    //set tx pin into normal gpio mode and drive zero
    //need to implement
    GPIO_PORTC_DIR_R |= 0x60;                        //set the bit 5 and 6 as output and rest as input
    GPIO_PORTC_DEN_R |= 0x60;                        //set digital enable on bits 5 and 6 //drives zero by default



    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;         // turn-on UART1, leave other UARTs in same status
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC5_U1TX;

    // Configure UART1 to 250000 baud, 8N2 format (must be 3 clocks from clock enable and config writes)
    UART1_CTL_R = 0;                       // turn-off UART0 to allow safe programming
    delay4Cycles();
    UART1_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART1_IBRD_R = 10;                               // r = 40 MHz / (Nx250kHz), set floor(r)=21, where N=16
    UART1_FBRD_R = 0;                               // round(fract(r)*64)=45
    UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN |UART_LCRH_STP2 ; // configure for 8N1 w/o FIFO
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer
  //  GPIO_PORTC_DATA_R |=0;                        //drive zero on the Tx pin
}

void initTimer1()
{

    UART1_CTL_R = 0;
   // GPIO_PORTC_DATA_R |=0;                        //drive zero on the Tx pin
    PIN_U1TX=0;
    PIN_DEN=0;
    //Configure Timer 1 for the timer1 interrupt
   // SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer
    // enable TX, RX, and module
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 0x1B80;                           // set load value to 1B80 for interrupt every 176 us
    //TIMER1_TAILR_R = 0x2625A;
    //TIMER1_TAILR_R =  0x2625A00;
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    phase = 0;

   // GPIO_PORTC_AFSEL_R |= 0x0;                         //clear the AFSEL register on port c for normal GPIO mode


}

//code adopted from Dr. losh's provided example
// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);               // wait if uart0 tx fifo full
    UART0_DR_R = c;                                  // write character to fifo
}

//code adopted from Dr. losh's provided example
// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
        putcUart0(str[i]);
}

//code adopted from Dr. losh's provided example
// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);               // wait if uart0 rx fifo empty
    return UART0_DR_R & 0xFF;                        // get character from fifo
}

//code adopted from Dr. losh's provided example
void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
    // 40 clocks/us + error
}


//Step1: Flash LED to show that the code is up and running proper
void flashRedLed(uint16_t x)
{
    RED_LED=1;      //turn ON red led
    waitMicrosecond(x);
    RED_LED=0;
}

//Step 1: Flash LED to show that the code is up and running
void flashGreenLed(uint16_t x)
{
    GREEN_LED=1;      //turn ON green led
    waitMicrosecond(x);
    GREEN_LED=0;
}

//step 2: get the string from the UART0
void getsUart0(char *str)
{
    int counter=0;
    char uartChar;
    uint16_t len = MAX_CHAR;
    uartChar = getcUart0();
    while(uartChar!= 13 && counter < len)   //is it c/r or the MAX_CHAR?
    {
        if(uartChar == 8)         //is it backspace?
        {
            if(counter)         //if counter is not zero
            {
                str--;
                counter--;
            }
        }
        else if(uartChar >= 32)     //is the char printable??
        {
            *str = tolower(uartChar);   //save the char to string as upper case
            str++;
            counter++;
        }
        uartChar = getcUart0();
    }
    *str = 0;       //add the null terminator
}


//the parse routine that goes through the string and
uint8_t parseCommand(char str1[], uint8_t pos[], char type[])
{
    uint8_t i;
    uint8_t d=1;
    uint8_t fCount;
    uint8_t j;
    uint8_t length =strlen(str1);
    for (i=0; i<length; i++)
    {
        if (str1[i]>96 && str1[i]<123)      //if the character of the string is a alpha a-z
        {
            if(d==1)
            {
                pos[j]=i;
                type[j]='a';
                fCount++;
                j++;
                d=0;
                // putsUart0("d->a\n");
            }
            else
            {
                d=0;
            }

        }
        else if (str1[i]>47 && str1[i]<58)     //if the character of the string is a number 0-9
        {
            if(d==1)
            {
                pos[j]=i;      //add the current index to the pos[]
                type[j]='n';
                fCount++;
                j++;
                d=0;
                // putsUart0("d->n\n");
            }
            else
            {
                d=0;
            }
        }
        else    //if the character of the string is a delimiter
        {

            d=1;
            str1[i]=0;  //replace all delimiters with null
        }
    }
    type[j]=0;   //end the type[] with null
    return fCount;

}
//
bool isCommand(char str[], uint8_t pos[], char *cmd)
{
    return ((strcmp(&str[pos[0]],cmd))==0);
}

uint16_t getValue (char str[], uint8_t pos[], uint8_t argNum)
{
    uint16_t value = atoi(&str[*(pos+1+argNum)]);
    putsUart0(&str[*(pos+1+argNum)]);
    putsUart0("\n");
    return value;
}

char getString (char str[], uint8_t pos[], uint8_t argNum)
{
    putsUart0(&str[*(pos+1+argNum)]);
    putsUart0("\n");
    return (&str[*(pos+1+argNum)]);
}

void toString(char str[], uint16_t num)
{
    uint16_t i, rem, len = 0, n;

    n = num;
    if (num==0)
    {
        str[len]=0+'0';
        len++;
    }
    while (n != 0)
    {
        len++;
        n /= 10;
    }
    for (i = 0; i < len; i++)
    {
        rem = num % 10;
        num = num / 10;
        str[len - (i + 1)] = rem + '0';
    }
    str[len] = 0;
}

void clearDmxData()
{
    ZERO_ANY_A(uint8_t, dmxData);
}

void timer1Isr()
{
    if (phase==0)
    {
      //  putsUart0("the first timer fired\n");
     //   GPIO_PORTC_DATA_R |=0x20;                        //drive 1 on the Tx pin
        PIN_U1TX=1;
        TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
        TIMER1_TAILR_R = 0x01E0;                         // set load value to 1EO for interrupt every 12 us
        TIMER1_CTL_R |= TIMER_CTL_TAEN;
        TIMER1_ICR_R = TIMER_ICR_TATOCINT;
        phase = 1;
    }
    else if (phase==1)
    {
        TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
       //putsUart0("the second timer fired\n");
        //phase=2;
        //GPIO_PORTC_AFSEL_R |= 0x20;                     //turn on UART on the pin
       UART1_CTL_R = UART_CTL_TXE | UART_CTL_UARTEN| UART_CTL_EOT; // enable TX, RX, and module and end of transmission
       NVIC_EN0_R |= 1 << (INT_UART1-16);               // turn-on interrupt 22 (UART0)
        //putsUart0("the second timer fired\n");
      //

       //UART1_IM_R = UART_IM_TXIM;
       //TIMER1_ICR_R = TIMER_ICR_TATOCINT;
       UART1_IM_R = UART_RIS_TXRIS;
        //while (UART1_FR_R & UART_FR_TXFF);
        UART1_DR_R = 0;
        //initTimer1();

        TIMER1_ICR_R = TIMER_ICR_TATOCINT;
        phase = 2;
       // initTimer1();
       // while (UART1_FR_R & UART_FR_TXFF);

    }
        //TIMER1_ICR_R = TIMER_ICR_TATOCINT;
}
void uart1Isr()
{
    //putsUart0("ISR 1 FIRED");
    if ((phase-2)<maxM)
      {
//        while (UART1_FR_R & UART_FR_TXFF);
        //putsUart0("ISR 1 FIRED\n");
        UART1_DR_R = dmxData[phase-2];
        phase++;
        UART1_ICR_R = UART_ICR_TXIC;
      }
    else
    {
        UART1_CTL_R =0;
       // putsUart0("ISR 2 FIRED");

       while (UART1_FR_R & UART_FR_TXFF);
       if (ON==1)
       {
       initTimer1();
       }
       UART1_ICR_R = UART_ICR_TXIC;
    }

}
//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    // Initialize hardware
    initHw();
    clearDmxData();     //intialize the dmx data table to zero
   // initTimer1();       //intialize the timer settings
    // Display greeting
    char uartString[MAX_CHAR+1];
    putsUart0("\n-------------------+---------------------------------------\n");
    putsUart0("             CSE4342 Embedded Systems II\n                 Spring 2019 Project\n");
    putsUart0("------------------------------------------------------------\n");
    putsUart0("Name  : Prabesh Khadka\nUTA ID: 1001201007\n");
    putsUart0("------------------------------------------------------------\n");
    putsUart0("Welcome to my project!!!\n\n");
    if (MODE==1)
    {
        putsUart0("The MC is currently in DEVICE MODE!\n");
    }
    else if (MODE==0)
    {
        putsUart0("The MC is currently in CONTROLLER MODE\n");
    }
    putcUart0('>');


    //step1
    //quickly flash the red and Green LED for 500ms  to check the LED are working
    flashRedLed(500000);
    waitMicrosecond(500000);
    flashGreenLed(500000);
    waitMicrosecond(500000);
    //end of step1

    while(1)
    {
        uint8_t i;
        uint16_t address;
        uint16_t data;
        char type[MAX_CHAR+1];
        char uartString[MAX_CHAR+1];
        char buffer[20];
        bool OK;

        //step2
        getsUart0(uartString);
        flashGreenLed(50000);       //blink green LED Request
        putsUart0(uartString);      //display to the screen for debug
        putsUart0("\n");
        //end of step 2

        //step 3
        fieldCount=parseCommand(uartString,pos,type);

        //testing
        for(i=0; i<fieldCount; i++)
        {
            putsUart0(&uartString[*(pos+i)]);
            putsUart0("\n");
        }
        putsUart0("\n");
        //testing end
        //uint8_t hi = atoi(&uartString[*(pos+1)]);
        //end of step 3
        OK = false;

        //step 4
        if(isCommand(uartString, pos, "clear") && (fieldCount>0))        //clear command
        {
            putsUart0("The command that you entered was CLEAR");
            putsUart0("\n");
            clearDmxData();
            OK=true;
        }


        if(isCommand(uartString, pos, "set") && (fieldCount>2))          //set command
        {
            putsUart0("The command that you entered was SET");
            putsUart0("\n");
            address= getValue(uartString, pos, 0);
            data= getValue(uartString, pos, 1);
            if (!(address>=1 && address<=512))
            {
                putsUart0("Invalid Address; 1-512 required!");
                putsUart0("\n");
            }
            else if (!(data>=0 && data<=255))
            {
                putsUart0("Invalid data; 0-255 required!");
                putsUart0("\n");
            }
            else
            {
                putsUart0("correct\n");
                dmxData[address-1]=data;
                OK=true;
            }

        }

        if(isCommand(uartString, pos, "get") && (fieldCount>1))      //get
        {
            putsUart0("The command that you entered was GET");
            putsUart0("\n");
            address= getValue(uartString, pos, 0);
            if (!(address>=1 && address<=512))
            {
                putsUart0("Invalid 1-512 required!");
                putsUart0("\n");
            }
            else
            {
                putsUart0("correct\n");
                toString(buffer,dmxData[address-1]);
                putsUart0(buffer);
                putsUart0("\n");
                OK=true;
            }
        }

        if(isCommand(uartString, pos, "on") && (fieldCount>0))       //on cammand
        {
            putsUart0("The command that you entered was ON");
            putsUart0("\n");
            ON =1;
            OK=true;
            initTimer1();
        }

        if(isCommand(uartString, pos, "off") && (fieldCount>0))      //off command
        {
            putsUart0("The command that you entered was OFF");
            putsUart0("\n");
            ON =0;
            OK=true;
        }

        if(isCommand(uartString, pos, "max") && (fieldCount>0))      //max command
        {
            putsUart0("The command that you entered was MAX");
            putsUart0("\n");
            address= getValue(uartString, pos, 0);
            if (!(address>=0 && address<=512))
            {
                putsUart0("Invalid 0-512 required!");
                putsUart0("\n");
            }
            else
            {
                OK=true;
                maxM=address;
            }

        }
        if(!OK)      //if the command wasn't processed or was unidentified
        {

            {
                putsUart0("Error");

                putsUart0("\n");
            }

        }

        //end of step 4 and 5


    }

}

