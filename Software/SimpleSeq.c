/*

    SIMPLE SEQUENCER, v0.2

    (c) Obviously International

*/ 

# include <avr/io.h>                                            // General AVR hardware definitions
# include <avr/interrupt.h>                                     // Interrupt-related definitions (required for the timer interrupt)
# include <stdbool.h>                                           // Required for the "bool" data type and values "true" and "false"

# include "SimpleSeq.h"                                         // For our own definitions


void InitializeHW (void);						                // Set all relevant processor config registers to their initial states
uint8_t GetTicks (void);                                        // Read the tick counter; how many TICKSPEED (1ms) ticks since last call
int8_t FilterInput (bool bInput, uint8_t * pState);             // Filter bInput values to remove noise and contact bounce
void GetFilteredInputs (bool * pClkEdge, bool * pReset,         // Read the clock & reset inputs and the three switches; filter the noise away
                        bool * pDirection, bool * pSeqLength, bool * pClkSource);
uint16_t GetADCValue (uint8_t Channel);                         // Read voltage (0...5V in = 0...1023 result) using processor's ADC
uint16_t GetIntClockSpeedAsTicks (void);                        // Read the internal clock speed potentiometer value, convert the period to ticks
bool InternalClockOscillator (uint16_t TickSpeed, uint8_t TicksPerLastCall);    // Internal oscillator state machine
void SetSequencerOutputs (uint8_t Step);                        // Sets the outputs driving the gate switches, CV pots and LEDs

/*

    The main loop of the Simple Sequencer.

    The operation is rather straightforward: the sequencer 
    has two main inputs, clock and reset. Of these, the reset 
    input returns the sequence to its starting state. When the 
    sequencer is in reset state, it won't advance. When it is 
    not in reset state, a rising edge in clock input makes 
    it advance to the next step. 

    There are several complications to this, though:

        - The clock can come from the external clock input or from
          the internal oscillator. The clock source switch selects
          which one is active.

        - The sequence length can be either full (8 steps) or half
          (4 steps). This is decided by the sequence length (mode) 
          switch.
          
        - The sequence can run either forwards or backwards. This
          is decided by the direction switch. 

        - Also, if the sequence runs backwards, the reset input resets
          the state to the highest allowed state (depending on sequence
          length switch). In forward mode it always resets to state 0.

    The main loop uses a timer ticker to produce a suitable sampling 
    clock for both the inputs and for the internal clock generator.
    The internal clock counts the ticks and produces a clock pulse
    after an adjustable number of ticks have been elapsed. The normal
    time resolution of the tick timing is 1ms, which should be fine 
    enough for anything but the most extreme forms of music.

    Since both the clock and reset inputs and the three mode switches
    can be noisy or have contact bounce, they're filtered digitally.
    This is much less important for the mode selection switches, as
    the user generally can't see if they have noise during transitions.
    However, when the instrument is in reset state, the noise in direction
    and sequence length switches might audibly reflect to the output
    (as contact bounce/noise can activate two different steps several
    times before settling to the correct one).

*/

int main (void)
 {
  int8_t Step = 0;                                              // Sequence step, 0...7 in normal mode and 0...3 in 4-step mode
  int8_t MaximumStep;                                           // Currently selected highest allowed step number
  uint8_t TicksElapsed;                                         // Ticks elapsed after the last run of the main loop
  bool bClockInput;                                             // Clock signal coming from the clock input
  bool bIntClock;                                               // Clock signal coming from the internal clock generator
  bool bResetInput;                                             // Reset input state
  bool bDirection;                                              // Sequence direction selection: false = forwards
  bool bSeqLength;                                              // Sequence length selection; false = full sequence, true = half
  bool bClockSelect;                                            // Clock source selection; false = external clock, true = internal
  bool bClock;                                                  // Actual clock, based on clock source selection and the two clock inputs

  InitializeHW ();                                              // Initialize processor IO pins, timer, ADC and some other registers
  SetSequencerOutputs (0);                                      // Initialize the sequencer sequence outputs

  while (1)
   {
    if ((TicksElapsed = GetTicks ()) == 0) continue;            // Run the loop only once per tick -> if no ticks elapsed since last call, wait
                                                                // (TicksElapsed is normally either 0 or 1 - unless the loop does something which takes time)
    GetFilteredInputs (&bClockInput, &bResetInput,              // Filter the clock and reset inputs and the 3 mode switches
                       &bDirection, &bSeqLength, &bClockSelect);
                                                                // Run the internal clock oscillator (even when it's not selected...)
    bIntClock = InternalClockOscillator (GetIntClockSpeedAsTicks (), TicksElapsed);     

    if (bClockSelect) bClock = bIntClock;                       // Internal clock selected: bClock comes from the internal oscillator
                 else bClock = bClockInput;                     // External clock selected: bClock comes from the clock input
                            
    if (bSeqLength) MaximumStep = MAX_HALF_STEP;                // The last allowed sequence step, depending on the length switch
               else MaximumStep = MAX_FULL_STEP;                // (switch on = half length (4 steps), switch off = full sequence (8 steps))

    if (bResetInput)                                            // The reset input is high: this overrides clock and resets the sequencer state
     {
      if (!bDirection) Step = 0;                                // Forward counting: reset state is step 0 for both full and half sequence mode
      if (bDirection) Step = MaximumStep;                       // Backwards counting: reset to the last step (depends on sequence length)
     }
    else                                                        // The reset input was low: sequencer operates normally. 
     {
      if (bClock)                                               // A clock pulse has arrived: advance the sequencer
       {
        if (bDirection) Step--;                                 // Advance backwards
                   else Step++;                                 // Advance forwards
       }
                                                    
      if (Step < 0) Step = MaximumStep;                         // Wrap to the end; "end" depends on the selected sequence lenght
      if (Step > MaximumStep) Step = 0;                         // Wrap to the beginning
     }

    SetSequencerOutputs (Step);                                 // Set the sequencer outputs to reflect the current step
   }
 }

/*

    Initializes all the relevant processor registers:

        - sets the input and output directions so that they correspond to the hardware
        - enables port pin pull-ups on unused pins so that they stay in well-defined states
        - configures the internal analog-to-digital converter so that in can measure the
	      speed setting potentiometer's value
        - configure and enable Timer 1 so that it will produce interrupts at TICKSPEED
	      rate (1ms period); these ticks are used for timing in the main program.
	    - disables the serial port, used by Arduino bootloader, just in case.
        - powers up all the peripherals, so that they can be immediately used

    This function must be called at the very beginning of the program. 

*/

void InitializeHW (void)
 {
  cli ();                                                       // Disable interrupts in case the Arduino bootloader left them on

  PRR = 0x00;                                                   // Power up all the internal peripherals, so that they can be used.
                                                                // Alternative: PRR = ~((1 << PRTIM1) | (1 << PRADC)) for minimum 
                                                                // power consumption (this software uses just ADC and timer 1).

  UCSR0B = 0x00;                                                // Disable the serial port (on pins PORTD.0 and PORTD.1, used by the
                                                                // Gate switch logic), in case the Arduino bootloader left it on

  DDRB = 0x00;                                                  // PORTB is all switch inputs or unused pins; configure all as inputs
  PORTB = ~(1 << ARDUINOLEDPIN);                                // Enable pull-ups on all pins (except the LED pin) so that 
                                                                // a) the switch pins will always be in well-defined state and
                                                                // b) that the unused pin's won't float and waste power

  DDRC = PORTMUXMASK;                                           // PORTC contains the 3 control voltage potentiometer multiplexer
                                                                // pins (PORTMUXMASK), two inputs (clock and reset) and one unused pin.
  PORTC = (1 << UNUSEDPIN);                                     // Pull-ups aren't enabled on inputs, because the clock and reset
                                                                // pins have quite high impedance. However, pull-up is enabled on 
                                                                // unused pin to prevent it from floating.

  DDRD = 0xFF;                                                  // PORTD drives the Gate switches. All 8 pins are used as outputs.
  PORTD = 0x00;

  ADMUX = (1 << REFS0) | SPEEDPOTCHANNEL;                       // Configure the ADC: use +5V power as reference, select the Speed
                                                                // potentiometer input (which is the only ADC input in use)
  ADCSRA = (1 << ADEN) | (1 << ADPS0) |                         // Enable ADC: clock = 16MHz/128 = 125kHz (must be 50...200kHz)
		   (1 << ADPS1) | (1 << ADPS2);
  ADCSRB = 0x00;                                                // ADC trigger source: conversion is started manually

  OCR1A = PROCSPEED / TICKSPEED - 1;                            // 16b Timer 1: run the timer in "clear timer on compare match" mode,
                                                                // so that it counts from 0 to ORC1A value. If the timer is set to use
                                                                // full processor clock, this value produces TICKSPEED overflows per
                                                                // second. With TICKSPEED = 1000, this is 1ms per tick.
  TCCR1A = 0x00;                                                // "clear timer on compare match" mode, no PWM output
  TCCR1B = (1 << WGM12) | (1 << CS10);                          // "clear timer on compare match" mode, running at full processor clock
  TIMSK1 = (1 << OCIE1A);                                       // Configure Timer 1 to produce interrupts at TICKSPEED

  sei ();                                                       // Enable interrupts, so that we get Timer 1 ticks
 }

/*

    Timer 1 compare interrupt

    This gets called when Timer 1 reaches its maximum value, 
    which happens TICKSPEED times per second - that is, at
    1000Hz or once per 1ms.

    The interrupt increments the _TickCount timer on every call
    and GetTicks () reads and zeroes this counter. Thus, code like

        while (1) MilliSeconds += GetTicks ();

    will count milliseconds since start.

*/

volatile uint8_t _TickCount;                                    // Increments on every TICKSPEED (1ms) tick; used as main timebase
                                                                // Must be volatile, or the compiler might optimize it or
                                                                // changes made to it away.
ISR (TIMER1_COMPA_vect)
 {
  _TickCount++;
 }

/*

    Returns the number of TICKSPEED ticks elapsed
    since the last call. This is intended to be used
    as the main program timebase.

    Note that the returned value is an increment since
    the last call. Thus, if called several times in rapid
    succession, the most calls will return 0.

    Code like:

        while (1) Ticks += GetTicks ();

    will count ticks/milliseconds since start.

*/

uint8_t GetTicks (void)
 {
  uint8_t i;

  cli ();                                                       // Disable interrupts, so that the timer interrupt
  i = _TickCount;                                               // won't change the value of _TickCount when we are
  _TickCount = 0;                                               // reading and zeroing it.
  sei ();                                                       // Re-enable interrupts.

  return (i);                                                   // Number of TICKSPEED ticks since last call
 }

/*

    Input filtering function. This takes the boolean
    input value bInput and remembers its last FILTERLENGTH
    values (actually 8, but only FILTERLENGTH bits matter). 
    These values are stored as bits in *pState, least significant
    bit being the newest value.
    If all the stored FILTERLENGT values are true, function 
    detects this as a valid true input and returns FILTERED_TRUE.
    If all the stored values are false, this is detected
    as a valid false input and the function returns FILTERED_FALSE.
    If neither is true (transition or noise in stored bits),
    function returns FILTERED_UNDEF. In this case the caller 
    should wait for the input to stabilize and use the last
    known valid value for it instead.
  
    The caller must take care that the variable pointed by
    pState remains valid between the calls. Also, there should
    be some *reasonable* time between calls, because if all the
    samples are very close to each other, short noise spikes
    might pass as valid values.

    With 1ms time between calls (once per TICKSPEED tick),
    something like FILTERLENGTH = 4...8 should be enough.

    Note 1: filtering adds delay to the signals. 
    Note 2: this function is limited to 8 samples; longer 
    filter would require changing the type of the state 
    variable (pState), among the other changes.

*/

int8_t FilterInput (bool bInput, uint8_t * pState)
 {
  uint8_t Mask;

  *pState <<= 1;                                                // The old bits are stored in *pState; make room for the new bit...
  if (bInput) *pState |= 0x01;                                  // ... and add the bit at the end of the buffer.
                                                                // (bInput == false: there's a zero already after the bit shift)
  Mask = (1u << FILTERLENGTH) - 1;                              // Create a bit mask which has FILTERLEN bits set in it
  if ((Mask & *pState) == 0x00) return (FILTERED_FALSE);        // All the FILTERLENGTH bits in *pState are 0 -> detect as false
  if ((Mask & *pState) == Mask) return (FILTERED_TRUE);         // All the FILTERLENGTH bits in *pState are 1 -> detect as true

  return (FILTERED_UNDEF);                                      // Input is not clearly true or false
 }

/*

    Simple Sequencer has two gate-level inputs: clock and reset. There
    are also 3 mode selection switches. This function filters them so 
    that short noise spikes and indeterminate levels in them won't 
    cause spurious operation in main program.

    Since only the rising edge of the clock input matters, this 
    detects the low to high transitions in it and sets *pClkEdge
    only when a rising edge is detected. That is, unlike all the
    other outputs, *pClkEdge does not directly reflect the clock 
    input state. The other outputs are:

        - *pReset: the reset input state
        - *pDirection: the direction selection switch state
        - *pSeqLength: the sequence length (mode) switch state
        - *pClkSource: the clock source selection switch state

    Note: while this function handles clock and reset as digital 
    inputs, the actual AVR input pins include an option for 
    measuring the inputs with an ADC. Thus, an alternative to
    this digital filtering would be to emulate hysteresis with 
    ADC readings. The benefit of this method is that it would 
    avoid the filtering delay. On the other hand, ADC conversion
    itself is a rather lengthy operation.

*/
                                                                // Convenient names for numbers; defined here instead
                                                                // of the .h file, as they have no use elsewhere.
# define NFILTERED_INPUTS           5                           // 5 inputs use filtering:
# define FILT_CLOCK                 0                           // 1st filtered input, clock input
# define FILT_RESET                 1                           // 2nd filtered input, reset input
# define FILT_DIR                   2                           // 3rd filtered input, sequence direction switch
# define FILT_LENGTH                3                           // 4th filtered input, sequence length switch
# define FILT_SOURCE                4                           // The last filtered input, clock source selection switch

void GetFilteredInputs (bool * pClkEdge, bool * pReset,
                        bool * pDirection, bool * pSeqLength, bool * pClkSource)
 {
  int8_t i;                                                     // Counter for the for() loop
  int8_t f;                                                     // Temporary filtered result (FILTERED_TRUE, FILTERED_FALSE, FILTERED_UNDEF)
  static bool bOldClock;                                        // The previous valid state of the clock input; used for edge detection
  static uint8_t FilterBuffer [NFILTERED_INPUTS];               // For remembering the filter states between calls
  static bool bValidStates [NFILTERED_INPUTS];                  // The last detected valid (steady) states for the inputs
  bool bInputs [NFILTERED_INPUTS] = { GetClockInput (),         // The current states of the 5 inputs to be filtered;
                                      GetResetInput (),         // these must be in the same order as FILT_xxx defines.
                                      GetSequenceDirection (),
                                      GetSequenceLength (),
                                      GetClockSource () };

  for (i = 0; i < NFILTERED_INPUTS; i++)                        // Go through all the inputs,
   {                                                            // filter them and if the filtering produces
    f = FilterInput (bInputs [i], &FilterBuffer [i]);           // a definite result (FILTERED_TRUE/FILTERED_FALSE)
                                                                // store this value; ignore unclear values.
    if (f != FILTERED_UNDEF) bValidStates [i] = (f == FILTERED_TRUE) ? true : false;
   }

  *pReset = bValidStates [FILT_RESET];                          // Pass all the filtered ("certain") 
  *pDirection = bValidStates [FILT_DIR];                        // results to the caller
  *pSeqLength = bValidStates [FILT_LENGTH];
  *pClkSource = bValidStates [FILT_SOURCE];

  *pClkEdge = false;                                            // *pClkEdge is different; it indicates a rising edge
  if (bValidStates [FILT_CLOCK] && !bOldClock) *pClkEdge = true;// Rising edge = the current state is true, previous was false
  bOldClock = bValidStates [FILT_CLOCK];
 }

/*

    Measures voltage using processor's internal ADC.
    The ADC uses the +5V supply as a reference and has 10b
    resolution. Thus, full scale is 1023, 0 = 0V, 512 = +2.5V
    and so on. If the channel number is invalid, function 
    returns 0.

    Note: ADC conversion is quite slow, taking around 100us.

*/

uint16_t GetADCValue (uint8_t Channel)
 {      
  if (Channel > MAXADCCHANNEL) return (0);                      // Non-existent channel (which would mess the ADMUX register) -> ignore it

  ADMUX &= (1 << REFS0) | (1 << REFS1) | (1 << ADLAR);          // Set the multiplexer before measurement; retain REFSx and ADLAR configuration
  ADMUX |= Channel;

  ADCSRA |= 1 << ADSC;                                          // Start conversion
  while (ADCSRA & (1 << ADSC));                                 // Wait; the "start conversion" bit returns to 0 on conversion completion

  return (ADC);                                                 // Return the 10b result
 }

/*

    Read the speed adjustment potentiometer of the internal 
    (sequencer) clock oscillator and convert the value to
    TICKSPEED ticks between beats.

    The speed adjustment potentiometer's value is read with
    the ADC, returning a value between 0 and ADC_MAXIMUM (1023).
    This is then scaled to beats per minute, using #defines 
    BPM_MINIMUM (0 input) and BPM_MAXIMUM (ADC_MAXIMUM input).
    This value is then converted to number of TICKSPEED ticks
    per beats.

    The function uses floating point math for the sake of 
    simplicity. The usual alternative for it would be suitably
    scaled integer (fixed point) math. However, since the 
    execution time is dominated by the ADC conversion (100us 
    or so) and because around 1kB of extra code does not really
    matter, there's not much point.
    If optimization for speed is necessary, the best starting
    point would be to convert the ADC routine above to use interrupts, 
    instead of busy-waiting the converter. In case it is found 
    desirable to convert the code below to use fixed point math, 
    please note that integer division, particularly the 32b integer 
    division, has an execution time comparable to floating point
    division. Division can be avoided by handling the potentiometer
    value as a direct "time between beats" adjustment, instead of 
    a "beats per time" adjustment. This can reduce the instrument's
    usability, though.
    
*/

uint16_t GetIntClockSpeedAsTicks (void)
 {
  float Speed;

  Speed = GetADCValue (SPEEDPOTCHANNEL);                        // "Speed" in range 0...ADC_MAXIMUM 
  Speed *= (float) (BPM_MAXIMUM - BPM_MINIMUM) / (float) ADC_MAXIMUM;
                                                                // "Speed" now in range 0...(BPM_MAXIMUM-BPM_MINIMUM)
  Speed += BPM_MINIMUM;                                         // ... and now in range BPM_MINIMUM...BPM_MAXIMUM

  return (60.0 * TICKSPEED  / Speed);                           // Convert times-per-minute (BPM) to times-per-second (60.0),
 }                                                              // then to number-of-ticks per beat (TICKSPEED/Speed)

/*

    Internal beat clock oscillator state machine.
    The function is called with TickSpeed specifying the
    number of ticks between beats and TicksPerLastCall
    indicating how many ticks have been elapsed since
    the function was called the last time.
    In case one full beat period has been elapsed, function
    returns true (beat), otherwise false (no beat).

    The most convenient way to use this is to include
    it in the main loop, which uses GetTicks () to get the
    number of ticks elapsed since last run of the loop and
    pass this value as TicksPerLastCall parameter.

    The TickSpeed parameter can be obtained via 
    GetIntClockSpeedAsTicks (). 

    Note: if TicksPerLastCall can be over 1, the produced
    beats can be inaccurate or jittery. Fixing this is 
    somewhat tedious, so the recommended solution is to make 
    sure it won't happen, at least not too often.

*/

bool InternalClockOscillator (uint16_t TickSpeed, uint8_t TicksPerLastCall)
 {
  bool bBeat = false;                                           // Function returns "beat ready" only once per cycle -> default = no beat
  static uint16_t TickCounter;                                  // Ticks elapsed since the last beat

  TickCounter += TicksPerLastCall;                              // Increment the tick counter by the time elapsed since last call
  if (TickCounter >= TickSpeed)                                 // Specified time period elapsed?
   {
    TickCounter = 0;                                            // Start a new timing cycle 
    bBeat = true;                                               // and notify the caller about an elapsed beat period
   }

  return (bBeat);                                               // Notify the caller whether one beat period has been elapsed (or not)
 }

/*

    The sequencer hardware has two groups of GPIO
    pins reflecting the current step: the 8 pins 
    driving the gate switches and the 3 pins
    driving the control voltage multiplexers.

    Of these, the multiplexer control pins use straight
    binary code (and thus can use the Step parameter as-is), 
    while the gate switch drivers use 1-out-of-8 code - that 
    is, only one of the 8 pins is high at any given time.

    This function sets those pins to reflect value Step. 
    Step must be between 0 and MAX_FULL_STEP (7). Incorrect
    values cause spurious outputs, but have no other effect.

*/

void SetSequencerOutputs (uint8_t Step)
 {
  SetPotMultiplexer (Step);                                     // Select (2 x) one of the 8 potentiometers
  SetGateOutputs (1 << Step);                                   // Set one of the 8 gate selection switch outputs (in "one hot" fashion)
 }
