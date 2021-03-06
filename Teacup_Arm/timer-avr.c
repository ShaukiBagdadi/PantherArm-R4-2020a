
/** \file
  \brief Timer management, AVR and simulator specific part.

  To be included from timer.c.

	Teacup uses timer1 to generate both step pulse clock and system clock.

	We achieve this by using the output compare registers to generate the two clocks while the timer free-runs.

	Teacup has tried numerous timer management methods, and this is the best so far.
*/

#if defined TEACUP_C_INCLUDE && (defined __AVR__ || defined SIMULATOR)

#include	"config_wrapper.h"
#include "pinio.h"
#include "clock.h"
#include "cpu.h"
#include "memory_barrier.h"

#ifdef	MOTHERBOARD
#include	"dda_queue.h"
#endif


/**
  Time until next step, as output compare register is too small for long
  step times.
*/
uint32_t	next_step_time;

#ifdef ACCELERATION_TEMPORAL
/// Unwanted extra delays, ideally always zero.
uint32_t	step_extra_time = 0;
#endif /* ACCELERATION_TEMPORAL */


/** System clock interrupt.

  Comparator B is the system clock, happens every TICK_TIME.
*/
ISR(TIMER1_COMPB_vect) {
  static volatile uint8_t busy = 0;

	// set output compare register to the next clock tick
	OCR1B = (OCR1B + TICK_TIME) & 0xFFFF;

  clock_tick();

  /**
    Lengthy calculations ahead! Make sure we didn't re-enter, then allow
    nested interrupts.
  */
  if ( ! busy) {
    busy = 1;
    sei();

    dda_clock();

    busy = 0;
  }
}

#ifdef	MOTHERBOARD

/** Step interrupt.

  Comparator A is the step timer. It has higher priority then B.
*/
ISR(TIMER1_COMPA_vect) {
	// Check if this is a real step, or just a next_step_time "overflow"
	if (next_step_time < 65536) {
		// step!
		#ifdef DEBUG_LED_PIN
			WRITE(DEBUG_LED_PIN, 1);
		#endif

		// disable this interrupt. if we set a new timeout, it will be re-enabled when appropriate
		TIMSK1 &= ~MASK(OCIE1A);

		// stepper tick
		queue_step();

		// led off
		#ifdef DEBUG_LED_PIN
			WRITE(DEBUG_LED_PIN, 0);
		#endif

		return;
	}

	next_step_time -= 65536;

  // Similar algorithm as described in timer_set() below.
	if (next_step_time < 65536) {
		OCR1A = (OCR1A + next_step_time) & 0xFFFF;
	} else if(next_step_time < 75536){
		OCR1A = (OCR1A - 10000) & 0xFFFF;
		next_step_time += 10000;
	}
	// leave OCR1A as it was
}
#endif /* ifdef MOTHERBOARD */

/** Timer initialisation.

  Initialise timer and enable system clock interrupt. Step interrupt is
  enabled later, when we start using it.
*/
void timer_init() {
	// no outputs
	TCCR1A = 0;
	// Normal Mode
	TCCR1B = MASK(CS10);
	// set up "clock" comparator for first tick
	OCR1B = TICK_TIME & 0xFFFF;
	// enable interrupt
	TIMSK1 = MASK(OCIE1B);
#ifdef SIMULATOR
  // Tell simulator
  sim_timer_set();
#endif
}

#ifdef	MOTHERBOARD
/** Specify how long until the step timer should fire.

  \param delay Delay for the next step interrupt, in CPU ticks.

  \param check_short Tell whether to check for impossibly short requests. This
         should be set to 1 for calls from the step interrupt. Short requests
         then return 1 and do not schedule a timer interrupt. The calling code
         usually wants to handle this case.

         Calls from elsewhere should set it to 0. In this case a timer
         interrupt is always scheduled.

  \return A flag whether the requested time was too short to allow scheduling
          an interrupt. This is meaningful for ACCELERATION_TEMPORAL, where
          requested delays can be zero or even negative. In this case, the
          calling code should repeat the stepping code immediately and also
          assume the timer to not change his idea of when the last step
          happened.

  Strategy of this timer is to schedule timer interrupts not starting at the
  time of the call, but starting at the time of the previous timer interrupt
  fired. This ignores the processing time taken in the step interrupt so far,
  offering smooth and even step distribution. Flipside of this coin is,
  one has to call timer_reset() before scheduling a step at an arbitrary time.

	This enables the step interrupt, but also disables interrupts globally.
	So, if you use it from inside the step interrupt, make sure to do so
	as late as possible. If you use it from outside the step interrupt,
	do a sei() after it to make the interrupt actually fire.
*/
uint8_t timer_set(int32_t delay, uint8_t check_short) {
	uint16_t step_start = 0;
	#ifdef ACCELERATION_TEMPORAL
	uint16_t current_time;
	#endif /* ACCELERATION_TEMPORAL */

	// An interrupt would make all our timing calculations invalid,
	// so stop that here.
	cli();
	CLI_SEI_BUG_MEMORY_BARRIER();

	// Assume all steps belong to one move. Within one move the delay is
	// from one step to the next one, which should be more or less the same
	// as from one step interrupt to the next one. The last step interrupt happend
	// at OCR1A, so start delay from there.
	step_start = OCR1A;
	next_step_time = delay;

	#ifdef ACCELERATION_TEMPORAL
    if (check_short) {
      current_time = TCNT1;

      // 200 = safe number of cpu cycles after current_time to allow a new
      // interrupt happening. This is mostly the time needed to complete the
      // current interrupt.
      if ((current_time - step_start) + 200 > delay)
        return 1;
    }
	#endif /* ACCELERATION_TEMPORAL */

  // From here on we assume the requested delay is long enough to allow
  // completion of the current interrupt before the next one is about to
  // happen.

  // Now we know how long we actually want to delay, so set the timer.
	if (next_step_time < 65536) {
		// set the comparator directly to the next real step
		OCR1A = (next_step_time + step_start) & 0xFFFF;
	}
	else if (next_step_time < 75536) {
		// Next comparator interrupt would have to trigger another
		// interrupt within a short time (possibly within 1 cycle).
		// Avoid the impossible by firing the interrupt earlier.
		OCR1A = (step_start - 10000) & 0xFFFF;
		next_step_time += 10000;
	}
	else {
		OCR1A = step_start;
	}

	// Enable this interrupt, but only do it after disabling
	// global interrupts (see above). This will cause push any possible
	// timer1a interrupt to the far side of the return, protecting the
	// stack from recursively clobbering memory.
	TIMSK1 |= MASK(OCIE1A);
  #ifdef SIMULATOR
    // Tell simulator
    sim_timer_set();
  #endif

  return 0;
}

/** Timer reset.

  Reset the timer, so step interrupts scheduled at an arbitrary point in time
  don't lead to a full round through the timer counter.

  On AVR we simply do nothing, such a full round through the timer is just
  2^16 / F_CPU = 3 to 4 milliseconds.
*/
void timer_reset() {
}

/** Stop timers.

  This means to be an emergency stop.
*/
void timer_stop() {
	// disable all interrupts
	TIMSK1 = 0;
  #ifdef SIMULATOR
    // Tell simulator
    sim_timer_stop();
  #endif
}
#endif /* ifdef MOTHERBOARD */

#endif /* defined TEACUP_C_INCLUDE && (defined __AVR__ || defined SIMULATOR) */
