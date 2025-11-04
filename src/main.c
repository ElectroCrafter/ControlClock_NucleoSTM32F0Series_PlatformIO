#include "stm32f0xx.h"
#include <stdlib.h>
#define _OPEN_SYS_ITOA_EXT
#define ledPin GPIO_PIN_5
volatile uint32_t millis_count = 0;

/*
Objective   : 
Button Press Detection
    1. Blinking without delay using upcouter timer2 channel 2 at the PA1
    2. Attaches a pin interrupt button input GPIO providing an external circuit. 
    Testing :
    - all methods are compared by interrupting or without it
    - try to use a greater delay than 1000mS

Bugs        : 
    1. Still eror the timer cannot execute. If we rebuild the code using delay function, it works well.
    → Solved (Timer using interrupt)
    Solutions
    a. check the configuration of timer2 upcouter rising edge
    b. check how to handle input capture system
    
    2. The timer still operates with interrupt mode. We still didn't find to utilize timer without interrupt
    Solutions (Timer without interrupt)
    a. Try to understand deeply about procedure counter internal clock source with internal signal response                                
    b. Combine the blinking without delay with the intteuprt UART communcation bidirectional

    3. float serial print still doesn't show correct value
    Solutions
    a. Try to understand data type converstion from float into string

    4. millis still has bug in counter overflow
    a. Simulate counter flow using small ARR or update event (max cntr less than 10s)
    b. Make sure, the interval is not zero, if interval%TIM2 ->ARR

    5. The MCU have not been changed the clock frequency into 48 MHz
    a. the flash latency or FLASH->ACR is Configured to support the 48 MHz clock frequency
Notes       :
    - re-create a basic foundation counter code only using TIM2 ->ARR and TIM2 ->PSC 
*/

typedef enum {
    INT,
    FLOAT,
    UINT32,
    UINT16,
    UINT8,
    CHR,
    STRING
} DataType;

void sysInit(void){

    /*
    Set the Flash ACR to use 1 wait-state and enable the prefetch buffer and pre-read.
    -> if SYSCLK <= 24MHz           : 0 or zero wait state     0
    -> if 24MHz<= SYSCLK <= 48MHz   : 1 or 1 wait state        FLASH_ACR_LATENCY
    ->FLASH_ACR_PRFTBS              : Prefetch buffer enable
    ->FLASH_ACR_PRFTBE              : Prefetch buffer enable
    */ 
    
    FLASH->ACR |=  (FLASH_ACR_LATENCY |
                    FLASH_ACR_PRFTBS |
                  FLASH_ACR_PRFTBE);


    /* 
    Configure the PLL to (HSI / 2) * 12 = 48MHz.
    Use a PLLMUL of 0xA for *12, and keep PLLSRC at 0
    to use (HSI / PREDIV) as the core source. HSI = 8MHz.
    */
    RCC->CFGR  &= ~(RCC_CFGR_PLLMUL |
                    RCC_CFGR_PLLSRC);
    RCC->CFGR  |=  (RCC_CFGR_PLLSRC_HSI_DIV2 |
                    RCC_CFGR_PLLMUL12);
    // Turn the PLL on and wait for it to be ready.
    RCC->CR    |=  (RCC_CR_PLLON);
    while (!(RCC->CR & RCC_CR_PLLRDY)) {
        
    };
    // Select the PLL as the system clock source.
    RCC->CFGR  &= ~(RCC_CFGR_SW);
    RCC->CFGR  |=  (RCC_CFGR_SW_PLL);
    while (!(RCC->CFGR & RCC_CFGR_SWS_PLL)) {

    };

}

void GPIOinit(){
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;    // enable GPIOA clock 
    GPIOA->MODER |= GPIO_MODER_MODER5_0 ; //general purpose output mode on pin A5

}

void USART2_Init(void) {
    // Enable USART2 clock
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    //configure system clock selected as USART2 or UART2 clock
    RCC->CFGR3 |= RCC_CFGR3_USART2SW;

    // Configure GPIOA pins for USART2
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Enable GPIOA clock
    GPIOA->MODER |= GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1; // Alternate function mode for PA2 and PA3
    GPIOA->AFR[0] |= (1 << (2 * 4)) | (1 << (3 * 4)); // AF1 for PA2 and PA3

    // Configure USART2
    USART2->CR1 &= ~USART_CR1_UE; // Disable USART2
    USART2->BRR = SystemCoreClock / (9600); // Set baud rate to 38400
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE; // Enable transmitter and receiver
    USART2->CR1 |= USART_CR1_UE; // Enable USART2
}

void USART2_SendChar(char ch) {
    // Wait until transmit data register is empty
    while (!(USART2->ISR & USART_ISR_TXE))
        ;
    USART2->TDR = (ch & 0xFF);
}

void USART2_SendString(const char *str) {
    while (*str) {
        USART2_SendChar(*str++);
    }
}

void serialPrint(void* data, DataType type) {
    char buffer[100];  // Adjust size as needed

    switch (type) {
        /*integer converts syntax*/
        /*
        IntToChar(*(int*)data, buffer,10);

        1. data: A pointer to the data passed to the SerialPrint function.
        2. (int*)data: Casts the void* pointer data to an int* (pointer to an integer).
        3. *(int*)data: Dereferences the int* pointer, accessing the integer value stored at that memory location.
        4. IntToChar(): A function that converts an integer to a string and stores the result in buffer.
        5. buffer: A character array where the converted string is stored.
        6. 10 -> decimal ; 2 -> binary ; 16 -> hexa
        */
        case INT:
            itoa(*(int*)data,buffer,10); 
            break;
        case UINT32:
            itoa(*(uint32_t*)data,buffer,10); 
            break;
        case UINT16:
            itoa(*(uint16_t*)data,buffer,10); 
            break;
        case UINT8:
            itoa(*(uint8_t*)data,buffer,2); 
            break;
        case CHR:
            // Cast to char* and assign the character
            buffer[0] = *(char*)data;

            // Null-terminate the string
            buffer[1] = '\0';
            break;
        // case FLOAT:
        //     gcvt((float*)data,4,buffer);  // error find the true casting to collect val in the buffer
        //     break;
        case STRING:
            strcpy(buffer, (char*)data);  // Copy string
            break;
        default:
            return;  // Unsupported data type
    }
    for (int i = 0; buffer[i] != '\0'; i++) {
        USART2->TDR = buffer[i];  // Load character into UART data register
        while (!(USART2->ISR & USART_ISR_TXE));  // Wait until transmission is complete
    }
}

void confTimPol(uint16_t tDelay){
/*
    Timer without interrupt using an internal clock 8 Mhz
  1. Enable timer 2
  2. enable GPIOA at GPIOinit function
  3. Select PA1 as an alternate pin TIM2_CH2 || Bit Masking Technique: Bit Clearing and Setting
  4. Set the PA1 as an alternate GPIO TIM2_CH2 on the low register AFSEL1 (AFRL) using AF2
  5. TIM prescaler register Address offset: 0x28
  6. Setting auto-reload timer2 on the 1000mS (limit count up the counter)
  7. Send an update event to reset the timer and apply settings.
  8. Enable the hardware interrupt.
  9. Start the timer
  */
    RCC ->APB1ENR |= RCC_APB1ENR_TIM2EN;   //(1)
    GPIOA ->MODER = (GPIOA->MODER &~(GPIO_MODER_MODER1))
    |(GPIO_MODER_MODER1_1);               //(3)
    GPIOA ->AFR[0] |= (0b10 <<4);         //(4)
    //time-base unit
    TIM2 ->PSC = 24000-1;            //(5)
    TIM2 ->ARR = 4000-1;            //(6)

    /*Detail Calculation*/
    /*
    1. Clock Configuration
    Assume the timer clock (CK_INT) is running at 8 MHz.
    
    2.Set the prescaler value to 7999. This will divide the timer clock by 8000.
    TIM2->PSC = 7999; // Divides the timer clock by 8000
    - CK_PSC = CK_INT / (PSC + 1)
    - CK_PSC = 8 MHz / (7999 + 1) = 1 kHz -> 1000 pulse per detik || 1 pulse per mS
    if u want create uS
    8Mhz/8 = 1Mhz -> 8*10^6 pulse per detik || 1 pulse per uS

    3.Counter Register (CNT)
    Configure the counter to count up to a value (e.g., 1000) before generating an update event.
    TIM2->ARR = 1000 - 1; // Auto-reload value
    The timer will count from 0 to 999, generating an update event at 1 kHz / 1000 = 1 Hz (once every second).
    */
    
    TIM2 ->EGR  |= TIM_EGR_UG;      //(7)
    TIM2 ->DIER |= TIM_DIER_UIE;    //(8)
    TIM2 -> CR1 |= TIM_CR1_CEN;     //(9)  
}

uint32_t millis(void) {
    // Return the current value of the global millis_count
    return millis_count;
}

int main(void) {
    // Initialize internal system clock 8Mhz
    // SystemInit();
    // Initialize internal system clock 48Mhz
    sysInit();
    // Initialize GPIO led A5
    GPIOinit();
    //Initialize UART communication
    USART2_Init();
    //initialze timer 2
    confTimPol(0);
    //millis variables
    uint32_t prevMillis = 0;            // Read current counter value;
    uint32_t interval =1000;
    volatile uint32_t currentMillis = 0;
    //overflow counter variables
    uint32_t curr_counter  = 0;
    uint32_t prev_counter = 0;
    uint8_t delta;
    while (1) {
        curr_counter = TIM2->CNT;
        // Timer using interrupt or blinking without delay or delay generation

        //clear the update flag and reset counter value
        if (TIM2->SR & TIM_SR_UIF) {    // is not zero
        TIM2->SR &= ~(TIM_SR_UIF);      // reset update interrupt flag
        }

        // Calculate elapsed time in milliseconds
        if (curr_counter  >= prev_counter) {
            delta = curr_counter - prev_counter;
        } else {
            // counter overflow handle
            delta = (4000 - prev_counter) + curr_counter + 1;
        }

        currentMillis += delta; // Update global millis count
        prev_counter = curr_counter; // Update previous counter value

        //millis
        if (currentMillis-prevMillis>=interval){
            prevMillis = currentMillis;
  
            /* Read PLL time clock*/
            //mask into bit 21:18 PLLMUL of RCC -> CFGR
            uint8_t pllMul = (((RCC -> CFGR)&(0xF<<18))>>18)+2;

            //mask into bit 16:15 PLLSRC of RCC -> CFGR
            uint8_t pllSrc = ((RCC -> CFGR)&(0b11<<15))>>15;
            
            //if bit 16:15 is 0 of RCC -> CFGR, it means HSI is divided 2 
            if (pllSrc == 0){
                pllSrc = 2;
            }

            uint32_t readClock = (SystemCoreClock/pllSrc)*pllMul;
            char symblChr = '\t';
            
            // USART2_SendChar('\t');
            // serialPrint(&readClock,UINT32);
            // serialPrint(&symblChr,CHR);
            // serialPrint(&pllSrc,UINT8);
            // serialPrint(&symblChr,CHR);
            // serialPrint(&pllMul,UINT8);
            USART2_SendChar('\n');
            USART2_SendChar(sizeof(DataType));
            
            GPIOA -> ODR ^= ledPin; 
            USART2_SendChar('\n');
        }
    }
}
