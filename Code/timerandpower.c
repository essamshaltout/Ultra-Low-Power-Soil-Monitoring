#include <avr/io.h>
#include <avr/interrupt.h>

volatile uint8_t power_mode = 0;  // Initially in Power-down mode
volatile uint8_t overflow_count = 0; // Counter for overflow interrupts


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

// Timer2 Compare Match Interrupt Service Routine
ISR(TIMER2_COMP_vect)
{
    overflow_count++;

    if (overflow_count >= 5) 
    {
        overflow_count = 0; // Reset the counter
        power_mode = !power_mode;
        PORTB ^= (1 << PB0);
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
    MCUCR &= ~(1<<SE);
}

void set_idle_mode(void) {
    my_disable_sleep();
    MCUCR &= ~(1 << SM2);   // Clear SM2
    MCUCR &= ~(1 << SM1);   // Clear SM1
    MCUCR &= ~(1 << SM0);   // Clear SM0
    MCUCR |= (1 << SE);     // Enable sleep mode
    my_execute_sleep();
}

// Function to set Power-down mode
void set_power_down_mode(void) {
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
    MCUCR &= ~(1 << SM2);   // Clear SM2
    MCUCR |= (1 << SM1);    // Set SM1
    MCUCR |= (1 << SM0);    // Set SM0
    MCUCR |= (1 << SE);     // Enable sleep mode
    my_execute_sleep();
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


int main(void) {
    // Configure PB0 as output for LED
    DDRB |= (1 << PB0);

    // Initialize Timer2 in Asynchronous Mode
    init_timer2_async_ctc();

    while (1) {
        if (power_mode == 1) {
            //  disable_peripherals();   // Disable unused peripherals to save power
            set_power_save_mode();  // Set Power-down mode
            my_execute_sleep();
            // PORTA &= ~(1 << LED_PIN); // Turn off the LED

        } else {
            set_idle_mode();  // Set Power-down mode
            my_execute_sleep();
            // enable_peripherals();   // Disable unused peripherals to save power

        }
    }

    return 0;
}



