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
// System Clock:    40 MHzz

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
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define PIN_U1RX     (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 4*4)))
#define PIN_U1TX     (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 5*4)))
#define PIN_DEN      (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4)))

#define GREEN_LED_MASK 8
#define BLUE_LED_MASK 4
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
uint16_t tempData;
uint16_t rxData[512];
uint8_t fieldCount;
uint8_t pos[MAX_CHAR+1];
uint16_t deviceAddress =1; //have to write to eeprom 1 by default
uint8_t ON =0;      //DMX stream initialized to off //have to write to EEPROM
uint8_t MODE= 1;              //define a global variable for controller mode (0: Controller and 1: Device) //have to write to eeprom
uint16_t phase;
uint16_t rxPhase;
uint16_t glob=0;
uint16_t timeout;
uint16_t timeout1=1000;
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
    GPIO_PORTF_DIR_R = GREEN_LED_MASK | RED_LED_MASK |BLUE_LED_MASK;  // bits 1 and 3 are outputs
    GPIO_PORTF_DR2R_R = GREEN_LED_MASK | RED_LED_MASK|BLUE_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = GREEN_LED_MASK | RED_LED_MASK |BLUE_LED_MASK;  // enable LEDs
    
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
    GPIO_PORTC_DIR_R |= 0x60;                        //set the bit 5 and 6 as output and rest as input
    GPIO_PORTC_DEN_R |= 0x70;                        //set digital enable on bits 5 and 6 //drives zero by default
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;         // turn-on UART1, leave other UARTs in same status
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC5_U1TX | GPIO_PCTL_PC4_U1RX;
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1 | SYSCTL_RCGCTIMER_R0;       // turn-on timer
    
}

//intialize hardware for the UART1 RX/ DMX512 receive data
void initUart1RX()
{
    rxPhase=0;
    PIN_DEN=0;
    GPIO_PORTC_AFSEL_R |= 0x10;                       //turn on UART on the pin
    // Configure UART1 to 250000 baud, 8N2 format (must be 3 clocks from clock enable and config writes)
    UART1_CTL_R = 0;                                  // turn-off UART0 to allow safe programming
    // delay4Cycles();
    UART1_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART1_IBRD_R = 10;                               // r = 40 MHz / (Nx250kHz), set floor(r)=21, where N=16
    UART1_FBRD_R = 0;                               // round(fract(r)*64)=45
    UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN |UART_LCRH_STP2 ; // configure for 8N1 w/o FIFO
    UART1_CTL_R = UART_CTL_RXE | UART_CTL_UARTEN;//| UART_CTL_EOT; // enable TX, RX, and module and end of transmission
    UART1_IM_R = UART_IM_RXIM;
    NVIC_EN0_R |= 1 << (INT_UART1-16);               // turn-on interrupt 22 (UART0)
    timeout1=40;
    
    //UART1_IM_R = UART_RIS_TXRIS;                     //enable the UART1 interrupt
    
    
}

void initEEPROM()
{
    SYSCTL_RCGCEEPROM_R |=SYSCTL_RCGCEEPROM_R0;
    // EEPROM_EEDBGME_R|=(0xE37B<<16)|EEPROM_EEDBGME_ME;
    delay4Cycles(); //add 6 delay cycle plus the function overhead
    delay4Cycles();
    while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);  //poll the EEDONE register, continue when working=0;
    if ((EEPROM_EESUPP_R & EEPROM_EESUPP_PRETRY)||(EEPROM_EESUPP_R & EEPROM_EESUPP_ERETRY))
    {
        initEEPROM();       //error if the PRETRY and ERETRY bits are set
    }
    //    else
    //    {
    //        //do nothing
    //    }
    SYSCTL_SREEPROM_R|=SYSCTL_SREEPROM_R0;      //EEROM software reset
    SYSCTL_SREEPROM_R&=0;
    
    delay4Cycles();
    delay4Cycles();
    
    while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);  //poll the EEDONE register, continue when working=0;
    //  //  if (EEPROM_EESUPP_R==EEPROM_EESUPP_PRETRY||EEPROM_EESUPP_R==EEPROM_EESUPP_ERETRY)
    //    {
    //        initEEPROM();       //error if the PRETRY and ERETRY bits are set
    //    }
    //    else
    //    {
    //        continue;
    //    }
}

void initTimer1()
{
    PIN_DEN=1;                                       //set PC6 (Data enable pin to high)
    UART1_CTL_R = 0;                                //turn off UART1
    GPIO_PORTC_AFSEL_R = 0;                         //clear the AFSEL register on port c for normal GPIO mode
    GPIO_PORTC_DATA_R &=0xDF;                        //drive zero on the Tx pin
    //Configure Timer 1 for the timer1 interrupt
    //send break
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 0x1B80;                           // set load value to 1B80 for interrupt every 176 us
    
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    phase = 0;
    
}

void initTimer0()
{
    TIMER0_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off timer before reconfiguring
    TIMER0_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER0_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER0_TAILR_R = 0x1E8480;                       // set load value to 1B80 for interrupt every 176 us
    
    TIMER0_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER0A-16);             // turn-on interrupt 37 (TIMER1A)
    TIMER0_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    
    
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
    //putsUart0(&str[*(pos+1+argNum)]);
    // putsUart0("\n");
    return value;
}

char getString (char str[], uint8_t pos[], uint8_t argNum)
{
    //  putsUart0(&str[*(pos+1+argNum)]);
    // putsUart0("\n");
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



void timer0Isr()
{
    if (timeout==0)
    {
        GREEN_LED=0;
    }
    else
    {
        timeout--;
    }
    if (MODE==1)
    {
        if (timeout1==0)
        {
            GREEN_LED^=1;
            timeout1=20;
            
        }
        else
        {
            timeout1--;
        }
    }
    TIMER0_ICR_R = TIMER_ICR_TATOCINT;                  //clear timer interrupt
}

void timer1Isr()
{
    if (phase==0)               //when 176us has elapsed
    {
        //send MAB
        GPIO_PORTC_DATA_R |=0x20;                        //drive 1 on the Tx pin
        TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
        TIMER1_TAILR_R = 0x01E0;                         // set load value to 1EO for interrupt every 12 us
        TIMER1_CTL_R |= TIMER_CTL_TAEN;
        TIMER1_ICR_R = TIMER_ICR_TATOCINT;              //clear interrupt
        phase = 1;
    }
    else if (phase==1)
    {
        phase=2;
        GPIO_PORTC_AFSEL_R |= 0x30;                       //turn on UART on the pin
        // Configure UART1 to 250000 baud, 8N2 format (must be 3 clocks from clock enable and config writes)
        UART1_CTL_R = 0;                                  // turn-off UART0 to allow safe programming
        // delay4Cycles();
        UART1_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
        UART1_IBRD_R = 10;                               // r = 40 MHz / (Nx250kHz), set floor(r)=21, where N=16
        UART1_FBRD_R = 0;                               // round(fract(r)*64)=45
        UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN |UART_LCRH_STP2 ; // configure for 8N1 w/o FIFO
        UART1_CTL_R = UART_CTL_TXE | UART_CTL_UARTEN| UART_CTL_EOT; // enable TX, RX, and module and end of transmission
        NVIC_EN0_R |= 1 << (INT_UART1-16);               // turn-on interrupt 22 (UART1)
        
        TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer
        //    UART1_IM_R = UART_RIS_TXRIS;                     //enable the UART1 interrupt
        UART1_IM_R = UART_IM_TXIM;
        while (UART1_FR_R & UART_FR_TXFF);
        UART1_DR_R = 0;
        
    }
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;                  //clear timer interrupt
}

//UART interrupt handler
void processCommand()
{
    if (rxData[deviceAddress-1]>0)
    {
        BLUE_LED=1;
    }
    else if (rxData[deviceAddress-1]==0)
    {
        BLUE_LED=0;
    }
}


void uart1Isr()
{
    if (UART1_MIS_R==UART_MIS_TXMIS)  //if the interrupt was triggered by UART TX
    {
        if ((phase-2)<maxM)                                 //transmit all 512 data
        {
            UART1_DR_R = dmxData[phase-2];
            phase++;
            UART1_ICR_R = UART_ICR_TXIC;                    //clear the uart interrupt
        }
        else
        {
            UART1_CTL_R =0;
            //putsUart0("ISR 2 FIRED");
            
            while (UART1_FR_R & UART_FR_BUSY);
            if (ON==1)
            {
                initTimer1();
            }
            UART1_ICR_R = UART_ICR_TXIC;
        }
    }
    else    //if the interrupt was triggered by UART RX
    {
        //  BLUE_LED=1;
        
        tempData=UART1_DR_R;
        if((tempData & UART_DR_BE)==UART_DR_BE)
        {
            rxPhase=1;
            processCommand();
            timeout1=40;
            GREEN_LED=1;
        }
        switch(rxPhase)
        {
            case 0: break;
            
            case 1:
            if((tempData & 0xFF)==0)
            {
                rxPhase=2;
                break;
            }
            
            case 2:
            rxData[rxPhase-2]=(tempData & 0xFF);
            rxPhase++;
            break;
            default:
            rxData[rxPhase-3]=(tempData & 0xFF);
            rxPhase++;
            break;
            //rxPhase++;
            
        }
        
    }
    UART1_ICR_R = UART_ICR_RXIC;
    //GREEN_LED=1;
}
//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    // Initialize hardware
    initHw();
    initEEPROM();
    initTimer0();
    // EEPROMinit();
    clearDmxData();     //intialize the dmx data table to zero
    // initTimer1();       //intialize the timer settings
    // Display greeting
    char uartString[MAX_CHAR+1];
    putsUart0("\n-----------------------------------------------------------\n");
    putsUart0("             CSE4342 Embedded Systems II\n                 Spring 2019 Project\n");
    putsUart0("------------------------------------------------------------\n");
    putsUart0("Name  : Prabesh Khadka\nUTA ID: 1001201007\n");
    putsUart0("------------------------------------------------------------\n");
    putsUart0("Welcome to my project!!!\n\n");
    
    
    //step1
    //quickly flash the red and Green LED for 500ms  to check the LED are working
    //    flashRedLed(500000);
    //    waitMicrosecond(500000);
    //    flashGreenLed(500000);
    //    waitMicrosecond(500000);
    //end of step1
    RED_LED=1;
    waitMicrosecond(500000);
    RED_LED=0;
    waitMicrosecond(500000);
    
    
    
    EEPROM_EEBLOCK_R &=0xFFFFFFFE;
    EEPROM_EEOFFSET_R &=0xFFFFFFFE;
    EEPROM_EEBLOCK_R|=0x0;
    EEPROM_EEOFFSET_R|=0x0;
    
    if (EEPROM_EERDWR_R==1)
    {
        putsUart0("Device");
        MODE=1;
    }
    EEPROM_EEBLOCK_R&=0xFFFFFFFE;
    EEPROM_EEOFFSET_R&=0xFFFFFFFE;
    EEPROM_EEBLOCK_R|=0x0;
    EEPROM_EEOFFSET_R|=0x0;
    if (EEPROM_EERDWR_R==0)
    {
        putsUart0("controller");
        MODE=0;
    }
    EEPROM_EEBLOCK_R&=0xFFFFFFFE;
    EEPROM_EEOFFSET_R&=0xFFFFFFFE;
    EEPROM_EEBLOCK_R|=0x0;
    EEPROM_EEOFFSET_R|=0x1;
    deviceAddress=EEPROM_EERDWR_R;
    if (deviceAddress==32)
    {
        putsUart0("Hurray");
    }
    
    if (MODE==1)
    {
        initUart1RX();
        
    }
    
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
        putsUart0("$ ");
        getsUart0(uartString);
        GREEN_LED=1;
        timeout=1;
        putsUart0(uartString);      //display to the screen for debug
        putsUart0("\n");
        //end of step 2
        
        //step 3
        fieldCount=parseCommand(uartString,pos,type);
        
        //        //testing
        //        for(i=0; i<fieldCount; i++)
        //        {
        //            putsUart0(&uartString[*(pos+i)]);
        //            putsUart0("\n");
        //        }
        //        putsUart0("\n");
        //testing end
        //uint8_t hi = atoi(&uartString[*(pos+1)]);
        //end of step 3
        OK = false;
        
        
        
        if (MODE==0)
        {
            
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
                    dmxData[address-1]=data;
                    OK=true;
                }
                
            }
            
            if(isCommand(uartString, pos, "get") && (fieldCount==2))      //get
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
                    toString(buffer,dmxData[address-1]);
                    putsUart0(buffer);
                    putsUart0("\n");
                    OK=true;
                }
            }
            
            if(isCommand(uartString, pos, "on") && (fieldCount==1))       //on cammand
            {
                putsUart0("The command that you entered was ON");
                putsUart0("\n");
                ON =1;
                OK=true;
                RED_LED=1;
                initTimer1();
            }
            
            if(isCommand(uartString, pos, "off") && (fieldCount==1))      //off command
            {
                putsUart0("The command that you entered was OFF");
                putsUart0("\n");
                ON =0;
                RED_LED=0;
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
            if(isCommand(uartString, pos, "device") && (fieldCount==1))      //controller command
            {
                putsUart0("The command that you entered was DEVICE");
                putsUart0("\n");
                ON =0;
                RED_LED=0;
                MODE=1;
                EEPROM_EEBLOCK_R&=0xFFFFFFFE;
                EEPROM_EEOFFSET_R&=0xFFFFFFFE;
                EEPROM_EEBLOCK_R|=0x0;
                EEPROM_EEOFFSET_R|=0x0;
                EEPROM_EERDWR_R=1;
                initUart1RX();
                OK=true;
                
            }
        }
        else if(MODE==1)
        {
            if(isCommand(uartString, pos, "controller") && (fieldCount==1))      //controller command
            {
                putsUart0("The command that you entered was CONTROLLER");
                putsUart0("\n");
                MODE=0;
                EEPROM_EEBLOCK_R&=0xFFFFFFFE;
                EEPROM_EEOFFSET_R&=0xFFFFFFFE;
                EEPROM_EEBLOCK_R|=0x0;
                EEPROM_EEOFFSET_R|=0x0;
                EEPROM_EERDWR_R=0;
                OK=true;
            }
            if(isCommand(uartString, pos, "address") && (fieldCount==2))      //max command
            {
                putsUart0("The command that you entered was ADDRESS");
                putsUart0("\n");
                address= getValue(uartString, pos, 0);
                if (!(address>=0 && address<=512))
                {
                    putsUart0("Invalid 0-512 required!");
                    putsUart0("\n");
                }
                else
                {
                    deviceAddress=address;
                    EEPROM_EEBLOCK_R&=0xFFFFFFFE;
                    EEPROM_EEOFFSET_R&=0xFFFFFFFE;
                    EEPROM_EEBLOCK_R|=0x0;
                    EEPROM_EEOFFSET_R|=0x1;
                    EEPROM_EERDWR_R=address;
                    OK=true;
                }
                
            }
        }
        if(!OK)      //if the command wasn't processed or was unidentified
        {
            putsUart0("Error");
            putsUart0("\n");
            
        }
        else      //if the command was processed
        {
            putsUart0("Ready");
            putsUart0("\n");
        }
        
        
        //end of step 4 and 5
        
        
    }
    
}

