#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

// LED connected to PA5
#define LED_PIN PA5
#define OVERFLOWS_NEEDED 305  // Number of overflows required for 5 seconds
#define LED_PINN PC3  // Delete

// Global flag to track the current power mode (0 for Power-down, 1 for Idle)
volatile uint8_t power_mode = 0;  // Initially in Power-down mode

volatile uint16_t overflow_count = 0;  // To count the number of overflows

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
}

// Function to set Power-down mode
void set_power_down_mode(void) {
    my_disable_sleep();
    MCUCR &= ~(1 << SM2);   // Clear SM2
    MCUCR |= (1 << SM1);    // Set SM1
    MCUCR &= ~(1 << SM0);   // Clear SM0
    MCUCR |= (1 << SE);     // Enable sleep mode
}

// Function to set Power-save mode
void set_power_save_mode(void)
{
    MCUCR &= ~(1 << SM2);   // Clear SM2
    MCUCR |= (1 << SM1);    // Set SM1
    MCUCR |= (1 << SM0);    // Set SM0
    MCUCR |= (1 << SE);     // Enable sleep mode
}

// Function to set Standby mode
void set_standby_mode(void) {
    MCUCR |= (1 << SM2);    // Set SM2
    MCUCR |= (1 << SM1);    // Set SM1
    MCUCR &= ~(1 << SM0);   // Clear SM0
    MCUCR |= (1 << SE);     // Enable sleep mode
}

// Function to set Extended Standby mode
void set_extended_standby_mode(void) {
    MCUCR |= (1 << SM2);    // Set SM2
    MCUCR |= (1 << SM1);    // Set SM1
    MCUCR |= (1 << SM0);    // Set SM0
    MCUCR |= (1 << SE);     // Enable sleep mode
}

void my_enable_sleep()
{
    MCUCR |= (1<<SE);
}

void my_execute_sleep()
{
    asm volatile("sleep");
}

// void my_disable_sleep()
// {
//     MCUCR &= ~(1<<SE);
// }

// Interrupt Service Routine for INT1 (external interrupt on PD3)
ISR(INT2_vect)
{
    // PORTA ^= (1 << LED_PIN); // Toggle LED
    power_mode = !power_mode; // Switch power mode
    _delay_ms(50); // Debounce delay
}

void configure_interrupt_INT2(void)
{
    MCUCSR &= ~(1 << ISC2); // Configure INT2 for falling edge (ISC2 = 0)
    GICR |= (1 << INT2);    // Enable INT2
}

void disable_all_peripherals() {
    ADCSRA &= ~(1 << ADEN); // Disable ADC
    ACSR |= (1 << ACD);     // Disable Analog Comparator
    PRR |= (1 << PRSPI);    // Disable SPI
    PRR |= (1 << PRUSART);  // Disable USART
    PRR |= (1 << PRTWI);    // Disable TWI
    TCCR0 = 0;              // Disable Timer0
    TCCR1B = 0;             // Disable Timer1
    TCCR2 = 0;              // Disable Timer2
    ASSR &= ~(1 << AS2);    // Disable Timer/Counter2 Oscillator
    WDTCSR &= ~(1 << WDE);  // Disable Watchdog Timer
}

void enable_all_peripherals() {
    // Enable ADC
    ADCSRA |= (1 << ADEN); // Enable ADC

    // Enable Analog Comparator
    ACSR &= ~(1 << ACD);   // Enable Analog Comparator

    // Enable SPI
    PRR &= ~(1 << PRSPI);  // Enable SPI (clear bit to enable)

    // Enable USART
    PRR &= ~(1 << PRUSART); // Enable USART (clear bit to enable)

    // Enable TWI (I2C)
    PRR &= ~(1 << PRTWI);   // Enable TWI (clear bit to enable)

    // Enable Timer0
    TCCR0 = (1 << CS00);    // Enable Timer0 (set prescaler to 1, adjust as needed)

    // Enable Timer1
    TCCR1B = (1 << CS10);   // Enable Timer1 (set prescaler to 1, adjust as needed)

    // Enable Timer2
    TCCR2 = (1 << CS20);    // Enable Timer2 (set prescaler to 1, adjust as needed)

    // Enable Timer/Counter2 Oscillator (if using asynchronous mode)
    ASSR |= (1 << AS2);     // Enable Timer/Counter2 Oscillator

    // Enable Watchdog Timer
    WDTCSR |= (1 << WDE);   // Enable Watchdog Timer (if needed)
}

void Timer2_Init(void)
{
    // Set Timer0 to normal mode (default mode)
    TCCR2 = (1 << CS22);  // Set CS22 to 1
    TCCR2 &= ~((1 << CS21) | (1 << CS20));  // Clear CS21 and CS20 (make them 0)
  // Prescaler = 64 (CS01 = 1, CS00 = 1)

    // Enable Timer0 Overflow interrupt
    TIMSK |= (1 << TOIE2);  // Enable overflow interrupt for Timer2
}

ISR(TIMER2_OVF_vect)
{
    // This interrupt is triggered when Timer0 overflows
    overflow_count++;
    
    if (overflow_count >= OVERFLOWS_NEEDED)
    {
        // 5 seconds have passed, toggle the LED
        PORTC ^= (1 << LED_PINN);
        
        // Reset overflow count
        overflow_count = 0;
    }
}

// Main function
int main(void)
{
    // Set PA5 as output (LED)
    DDRA |= (1 << LED_PIN);
    PORTA |= (1 << LED_PIN); // Turn LED on initially
    PORTA |= (1 << LED_PIN); // Turn on the LED

    Timer2_Init();
    sei();

    DDRC |= (1 << LED_PINN); // Delete


    // Configure INT2
    configure_interrupt_INT2();

    // Enable global interrupts
    sei();

    // Main loop
    while (1)
    {
        if (power_mode == 1) {
             disable_peripherals();   // Disable unused peripherals to save power
            set_power_down_mode();  // Set Power-down mode
            my_execute_sleep();
            PORTA &= ~(1 << LED_PIN); // Turn off the LED

        } else {
            set_idle_mode();  // Set Power-down mode
            my_execute_sleep();
            enable_peripherals();   // Disable unused peripherals to save power

        }

        // Execute sleep mode
        // my_execute_sleep();
    }

    return 0;
}



