// Remeber to change CPUUUUUUUUUUUU
// Remember // Delete
// #define F_CPU 1000000UL // Define CPU clock frequency
#include "avr/io.h"
#include <util/delay.h>
#include "stdio.h"
#include <avr/interrupt.h>


#define LED_PIN PC3  // Delete
// LCD pins
#define RS PB0
#define RW PB1
#define EN PB2
#define LCD_PORT PORTB
#define LCD_DDR DDRB
#define OVERFLOWS_NEEDED 5

volatile uint8_t power_mode = 0;  // Initially in Power-down mode
volatile uint16_t overflow_count = 0;  // To count the number of overflows
char buffer[16];
uint16_t humidity;
uint16_t threshold = 512;
uint8_t pump_state = 0;


// Function prototypes
void ADC_Init();
uint16_t ADC_Read(uint8_t channel);

void LCD_Init();
void LCD_Command(unsigned char cmd);
void LCD_Char(unsigned char data);
void LCD_String(char *str);
void LCD_Clear();
void LCD_SetCursor(uint8_t row, uint8_t column);

void Relay_Init();
uint8_t Relay_Control(uint8_t state);

void my_enable_sleep();
void my_execute_sleep();
void my_disable_sleep();
void set_idle_mode(void);
void set_power_down_mode(void);
void set_power_save_mode(void);
void set_standby_mode(void);
void set_extended_standby_mode(void);

void init_timer2_async_ctc();


// Timer2 Compare Match Interrupt Service Routine
ISR(TIMER2_COMP_vect)
{
    overflow_count++;

    if (overflow_count >= OVERFLOWS_NEEDED) 
    {
        overflow_count = 0; // Reset the counter
        power_mode = 0;
        PORTC ^= (1 << PC0); // Delete
        my_disable_sleep();
        humidity = ADC_Read(0);  // Read data from channel 0 (ADC0)

        snprintf(buffer, sizeof(buffer), "Value: %4d",humidity);
        LCD_SetCursor(0, 0); // Move to second row
        LCD_String(buffer);
        
        if (humidity > threshold)   pump_state = Relay_Control(1);
        else                        pump_state = Relay_Control(0);

        if(pump_state == 1)     snprintf(buffer, sizeof(buffer), "Pump State: %s", "ON ");
        else                    snprintf(buffer, sizeof(buffer), "Pump State: %s", "OFF");
        LCD_SetCursor(1, 0); // Move to second row
        LCD_String(buffer);

        power_mode = 1;
        _delay_ms(100);
    }

}


int main(void) 
{
    ADC_Init();
    LCD_Init();
    Relay_Init();
    init_timer2_async_ctc();
    sei();

    power_mode = 0;
    my_disable_sleep();

    while (1) 
    {
        if(power_mode == 0)
        {
            my_disable_sleep();
        }
        else
        {
            set_power_save_mode();
        }
    }
}


void ADC_Init()
{
    ADMUX = (1 << REFS0);  // Use AVCC as reference
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);  // Enable ADC, prescaler = 64
}

uint16_t ADC_Read(uint8_t channel)
{
    ADMUX = (ADMUX & 0xF8) | (channel & 0x07);  // Select ADC channel
    ADCSRA |= (1 << ADSC);  // Start conversion
    while (ADCSRA & (1 << ADSC));  // Wait for conversion to complete
    return ADCW;  // Return ADC value
}

void LCD_Init()
{
    LCD_DDR = 0xFF;  // Configure LCD pins as output
    _delay_ms(20);  // Wait for LCD to initialize
    LCD_Command(0x02);  // Initialize in 4-bit mode
    LCD_Command(0x28);  // 2 lines, 5x7 matrix
    LCD_Command(0x0C);  // Display ON, cursor OFF
    LCD_Command(0x06);  // Increment cursor
    LCD_Command(0x01);  // Clear display
    _delay_ms(2);
}

void LCD_Command(unsigned char cmd)
{
    LCD_PORT = (LCD_PORT & 0x0F) | (cmd & 0xF0);  // Send upper nibble
    LCD_PORT &= ~(1 << RS);  // RS = 0 for command
    LCD_PORT &= ~(1 << RW);  // RW = 0 for write
    LCD_PORT |= (1 << EN);  // Enable pulse
    _delay_us(1);
    LCD_PORT &= ~(1 << EN);

    _delay_us(200);

    LCD_PORT = (LCD_PORT & 0x0F) | (cmd << 4);  // Send lower nibble
    LCD_PORT |= (1 << EN);
    _delay_us(1);
    LCD_PORT &= ~(1 << EN);
    _delay_ms(2);
}

void LCD_Char(unsigned char data)
{
    LCD_PORT = (LCD_PORT & 0x0F) | (data & 0xF0);  // Send upper nibble
    LCD_PORT |= (1 << RS);  // RS = 1 for data
    LCD_PORT &= ~(1 << RW);  // RW = 0 for write
    LCD_PORT |= (1 << EN);  // Enable pulse
    _delay_us(1);
    LCD_PORT &= ~(1 << EN);

    _delay_us(200);

    LCD_PORT = (LCD_PORT & 0x0F) | (data << 4);  // Send lower nibble
    LCD_PORT |= (1 << EN);
    _delay_us(1);
    LCD_PORT &= ~(1 << EN);
    _delay_ms(2);
}

void LCD_String(char *str)
{
    while (*str)
    {
        LCD_Char(*str++);
    }
}

void LCD_Clear()
{
    LCD_Command(0x01);  // Clear display
    _delay_ms(2);
}

void LCD_SetCursor(uint8_t row, uint8_t column)
{
    uint8_t positions[] = {0x80, 0xC0};  // Start addresses of rows
    LCD_Command(positions[row] + column);
}

// Relay Initialization
void Relay_Init() 
{
    DDRD |= (1 << PD4);  // Set PD0 as output for relay control
    PORTD &= ~(1 << PD4);  // Ensure the relay is off initially
}

// Relay Control
uint8_t Relay_Control(uint8_t state)
{
    if (state) 
    {
        PORTD |= (1 << PD4);  // Turn on relay
        return 1;
    }
    else 
    {
        PORTD &= ~(1 << PD4);  // Turn off relay
        return 0;
    }
}

void my_enable_sleep()
{
    MCUCR |= (1<<SE);
}

void my_execute_sleep()
{
    asm volatile("sleep");
}

void my_disable_sleep()
{
    // while (ASSR & ((1 << TCR2UB) | (1 << OCR2UB) | (1 << TCN2UB)));
    MCUCR &= ~(1<<SE);
    // while (ASSR & ((1 << TCR2UB) | (1 << OCR2UB) | (1 << TCN2UB)));

}

void set_idle_mode(void)
{
    my_disable_sleep();
    MCUCR &= ~(1 << SM2);   // Clear SM2
    MCUCR &= ~(1 << SM1);   // Clear SM1
    MCUCR &= ~(1 << SM0);   // Clear SM0
    MCUCR |= (1 << SE);     // Enable sleep mode
    my_execute_sleep();
}

// Function to set Power-down mode
void set_power_down_mode(void)
{
    my_disable_sleep();
    MCUCR &= ~(1 << SM2);   // Clear SM2
    MCUCR |= (1 << SM1);    // Set SM1
    MCUCR &= ~(1 << SM0);   // Clear SM0
    MCUCR |= (1 << SE);     // Enable sleep mode
    my_execute_sleep();
}

// Function to set Power-save mode
void set_power_save_mode(void)
{
    my_disable_sleep();
    MCUCR &= ~(1 << SM2);   // Clear SM2
    MCUCR |= (1 << SM1);    // Set SM1
    MCUCR |= (1 << SM0);    // Set SM0
    MCUCR |= (1 << SE);     // Enable sleep mode
    // while (ASSR & ((1 << TCR2UB) | (1 << OCR2UB) | (1 << TCN2UB)));
    my_execute_sleep();
    // while (ASSR & ((1 << TCR2UB) | (1 << OCR2UB) | (1 << TCN2UB)));
}

// Function to set Standby mode
void set_standby_mode(void)
{
    MCUCR |= (1 << SM2);    // Set SM2
    MCUCR |= (1 << SM1);    // Set SM1
    MCUCR &= ~(1 << SM0);   // Clear SM0
    MCUCR |= (1 << SE);     // Enable sleep mode
    my_execute_sleep();
}

// Function to set Extended Standby mode
void set_extended_standby_mode(void)
{
    MCUCR |= (1 << SM2);    // Set SM2
    MCUCR |= (1 << SM1);    // Set SM1
    MCUCR |= (1 << SM0);    // Set SM0
    MCUCR |= (1 << SE);     // Enable sleep mode
    my_execute_sleep();
}

void init_timer2_async_ctc()
{
    // Enable asynchronous mode
    ASSR |= (1 << AS2);

    // Set Timer2 to CTC mode
    TCCR2 |= (1 << WGM21);

    // Set the clock source to the 32.768 kHz crystal with a prescaler of 128
    TCCR2 |= (1 << CS22) | (1 << CS20); // Prescaler = 128

    // Set Compare Match value for 1-second interval
    OCR2 = 255;

    // Enable Compare Match Interrupt
    TIMSK |= (1 << OCIE2);

    // Wait for TOSC to stabilize
    while (ASSR & ((1 << TCR2UB) | (1 << OCR2UB) | (1 << TCN2UB)));

    // Enable global interrupts
    sei();
}

