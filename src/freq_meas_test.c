
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define F_CPU 8000000L
#define SetBit(ADDRESS,BIT) (ADDRESS |= (1<<BIT))
#define ClearBit(ADDRESS,BIT) (ADDRESS &= ~(1<<BIT))
#define ToggleBit(ADDRESS,BIT) (ADDRESS ^= (1<<BIT))


#define STIMULUS_PIN PIN6
#define SENSE_PIN PIN7

// -----------------------------------------------------------------------------
// Main Function
// -----------------------------------------------------------------------------
volatile uint16_t counter_value = 0;
volatile uint8_t capture_flag = 0;

ISR(TCB0_INT_vect)
{
    counter_value = TCB0_CCMP;
    capture_flag = 1;
}

void setup()
{
    /* configure main clock */
    cli();
    CPU_CCP = CCP_IOREG_gc;                       // Un-protect protected IO registers
    CLKCTRL.MCLKCTRLA = CLKCTRL_CLKSEL_EXTCLK_gc; // Select EXTCLK as main clock source
    CPU_CCP = CCP_IOREG_gc;                       // Un-protect protected IO registers
    ClearBit(CLKCTRL.MCLKCTRLB, CLKCTRL_PEN_bp);  // Disable main clock prescaler

    /* configure GPIO */
    SetBit(PORTA.DIRCLR, SENSE_PIN); // Configure sense pin as input
    SetBit(PORTA.DIRSET, STIMULUS_PIN); // Configure stimulus pin as output
    SetBit(PORTA.DIRSET, PIN2); // Configure stimulus pin as output
    PORTA.PIN7CTRL |= PORT_PULLUPEN_bm; // Disable digital buffer for AC0 P input

    /* configure Event Multiplexer */
    EVSYS.ASYNCCH0 |= EVSYS_ASYNCCH0_PORTA_PIN7_gc;
    EVSYS.ASYNCUSER0 |= EVSYS_ASYNCUSER0_ASYNCCH0_gc;

    /* configure timer B in frequency measurement mode */
    TCB0.CTRLB = TCB_CNTMODE_FRQ_gc; // configure mode
    TCB0.EVCTRL = TCB_CAPTEI_bm; // enable filter, trigger on falling edge, enable input capture event
    TCB0.INTCTRL = TCB_CAPT_bm;
    TCB0.CTRLA = TCB_ENABLE_bm;
}

int main() {
    // Setup
    setup();

    SetBit(TCB0.INTFLAGS, 0);
    SetBit(PORTA.OUTSET, STIMULUS_PIN);
    sei();

    while(1) {                              // loop until forever                         
        capture_flag = 0;
        SetBit(TCB0.INTFLAGS, 0);
        SetBit(PORTA.OUTCLR, STIMULUS_PIN);

        //for (uint16_t i = 16383; i; i--) {
        while (!capture_flag) { }

        SetBit(PORTA.OUTCLR, PIN2);

        SetBit(PORTA.OUTSET, STIMULUS_PIN);
        _delay_ms(10);
    }
}
