/*

    SIMPLE SEQUENCER, v0.2

    (c) Obviously International

*/ 


# ifndef SIMPLESEQ_H_
# define SIMPLESEQ_H_

# define PROCSPEED                  16000000                    // Arduino's 16MHz clock speed in Hz

# define TICKSPEED                  1000                        // Main program timer speed in Hz 

# define FILTERLENGTH               4                           // Clock and reset input filter length in samples (max. 8):
                                                                // how many continuous detections of true or false are needed
                                                                // for FilterInput () to produce a certain output (FILTERED_TRUE
                                                                // or FILTERED_FALSE).
# define FILTERED_FALSE             0                           // Return value from FilterInput (): input is certain false
# define FILTERED_TRUE              1                           // Return value from FilterInput (): input is certain true
# define FILTERED_UNDEF             -1                          // Return value from FilterInput (): input is noisy; no clear value

# define BPM_MINIMUM                40                          // The internal clock's adjustment range is from this...
# define BPM_MAXIMUM                200                         // ... to this beats per minute (middle range = 120BPM)

# define MAX_FULL_STEP              7                           // Sequencer has steps 0...7 in "full" mode (GetSequenceLength () = false)
# define MAX_HALF_STEP              3                           // Sequencer has steps 0...3 in "half" mode (GetSequenceLength () = true)

# define SetGateOutputs(x)          { PORTD = (x); }            // Set the 8 pins driving the Gate switches; 
                                                                // they're connected to Arduino pins D0...D7,
                                                                // corresponding AVR pins PORTD.0 ... PORTD.7

                                                                // PORTB PINS
# define SEQDIRPIN                  3                           // sequence direction switch; PORTB.3 = Arduino D11
# define SEQMODEPIN                 4                           // sequence mode (4/8 steps) switch; PORTB.4 = Arduino D12
# define CLKSOURCEPIN               0                           // clock source select switch (int/ext): PORTB.0 = Arduino D8
# define ARDUINOLEDPIN              5                           // Arduino's yellow LED; PORTB.5 = Arduino D13

# define GetSequenceDirection()     (PINB & (1 << SEQDIRPIN))   // Reads the sequence direction switch; FALSE =
                                                                // counts up, TRUE = counts down. 
# define GetSequenceLength()        (PINB & (1 << SEQMODEPIN))  // Reads the sequence mode switch; FALSE =
                                                                // 8 step mode, TRUE = 4 step mode. 
# define GetClockSource()           (PINB & (1 << CLKSOURCEPIN))// Reads the clock source switch; FALSE =
                                                                // external clock, TRUE = internal clock.

                                                                // PORTC PINS
# define CLKINPIN                   4                           // external clock input. PORTC.4 = Arduino A4
# define RESETINPIN                 5                           // external reset input; PORTC.5 = Arduino A5
# define UNUSEDPIN                  3                           // not used; defined here, so that it can be disabled
# define MUX0PIN                    0                           // multiplexer address 0 pin; defined to remind of its use
# define MUX1PIN                    1                           // multiplexer address 1 pin; defined to remind of its use
# define MUX2PIN                    2                           // multiplexer address 2 pin; defined to remind of its use

# define GetClockInput()            (PINC & (1 << CLKINPIN))    // Reads the (gate level) CLOCK input; FALSE =
                                                                // clock low, TRUE = clock high. 
# define GetResetInput()            (PINC & (1 << RESETINPIN))  // Reads the (gate level) RESET input; FALSE =
                                                                // reset low, TRUE = reset high. 

# define ADC_MAXIMUM                1023                        // Maximum conversion result for processor's 10b ADC; corresponds +5V
# define MAXADCCHANNEL              15                          // Maximum channel number for processor's internal ADC
# define SPEEDPOTCHANNEL            7                           // Speed setting potentiometer input, when internal
                                                                // clock is in use. This is Arduino pin A7, which
                                                                // is also ADC channel 7

# define PORTMUXMASK                0x07                        // The 3 lowest pins of PORTC = control voltage pot 
                                                                // selection outputs (MUX0PIN, MUX1PIN, MUX2PIN above).

# define SetPotMultiplexer(x)       { PORTC = (PORTC & ~PORTMUXMASK) | ((x) & PORTMUXMASK); }
                                                                // Sets the 3 pins used to drive the control voltage
                                                                // potentiometer multiplexers. These pins go to Arduino
                                                                // pins A0...2, which are conveniently the 3 lowest bits
                                                                // PORTC. Thus, the 3 pins can be set by simply zeroing
                                                                // their old values with a 3b bit mask (0x07) and then by
                                                                // adding the desired value (x) to them. X is also masked
                                                                // with 0x07, so that values beyond the allowed maximum 
                                                                // won't mess the other port pins.

# endif
