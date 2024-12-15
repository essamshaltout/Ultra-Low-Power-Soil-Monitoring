#include <avr/io.h>
#include <avr/interrupt.h>

#define LED_PIN PB0  // LED connected to PORTB0
#define OVERFLOWS_NEEDED 305  // Number of overflows required for 5 seconds

volatile uint16_t overflow_count = 0;  // To count the number of overflows

void Timer0_Init(void)
{
    // Set Timer0 to normal mode (default mode)
    TCCR0 = (1 << CS01) | (1 << CS00);  // Prescaler = 64 (CS01 = 1, CS00 = 1)

    // Enable Timer0 Overflow interrupt
    TIMSK |= (1 << TOIE0);  // Enable overflow interrupt for Timer0
}

ISR(TIMER0_OVF_vect)
{
    // This interrupt is triggered when Timer0 overflows
    overflow_count++;
    
    if (overflow_count >= OVERFLOWS_NEEDED)
    {
        // 5 seconds have passed, toggle the LED
        PORTB ^= (1 << LED_PIN);
        
        // Reset overflow count
        overflow_count = 0;
    }
}

int main(void)
{
    // Set LED_PIN as an output pin
    DDRB |= (1 << LED_PIN);

    // Initialize Timer0
    Timer0_Init();

    // Enable global interrupts
    sei();

    // Main loop
    while (1)
    {
        // Main loop does nothing, all the action happens in the ISR
    }

    return 0;
}
