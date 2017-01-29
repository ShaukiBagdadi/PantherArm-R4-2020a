#include	"home.h"

/** \file
	\brief Homing routines
*/

#include <math.h>
#include	"dda.h"
#include	"dda_queue.h"
#include	"pinio.h"
#include	"gcode_parse.h"

// Check configuration.
#if defined X_MIN_PIN || defined X_MAX_PIN
  #ifndef SEARCH_FEEDRATE_X
    #error SEARCH_FEEDRATE_X undefined. It should be defined in config.h.
  #endif
  #ifndef ENDSTOP_CLEARANCE_X
    #error ENDSTOP_CLEARANCE_X undefined. It should be defined in config.h.
  #endif
#endif
#if defined Y_MIN_PIN || defined Y_MAX_PIN
  #ifndef SEARCH_FEEDRATE_Y
    #error SEARCH_FEEDRATE_Y undefined. It should be defined in config.h.
  #endif
  #ifndef ENDSTOP_CLEARANCE_Y
    #error ENDSTOP_CLEARANCE_Y undefined. It should be defined in config.h.
  #endif
#endif
#if defined Z_MIN_PIN || defined Z_MAX_PIN
  #ifndef SEARCH_FEEDRATE_Z
    #error SEARCH_FEEDRATE_Z undefined. It should be defined in config.h.
  #endif
  #ifndef ENDSTOP_CLEARANCE_Z
    #error ENDSTOP_CLEARANCE_Z undefined. It should be defined in config.h.
  #endif
#endif
#if defined U_MIN_PIN || defined U_MAX_PIN
  #ifndef SEARCH_FEEDRATE_U
    #error SEARCH_FEEDRATE_U undefined. It should be defined in config.h.
  #endif
  #ifndef ENDSTOP_CLEARANCE_U
    #error ENDSTOP_CLEARANCE_U undefined. It should be defined in config.h.
  #endif
#endif

// Calculate feedrates according to clearance and deceleration.
// For a description, see #define ENDSTOP_CLEARANCE_{XYZ} in config.h.
//   s = 1/2 * a * t^2; t = v / a  <==> v = sqrt(2 * a * s))
//   units: / 1000 for um -> mm; * 60 for mm/s -> mm/min
#ifdef ENDSTOP_CLEARANCE_X
  #define SEARCH_FAST_X (uint32_t)((double)60. * \
            sqrt((double)2 * ACCELERATION * ENDSTOP_CLEARANCE_X / 1000.))
#endif
#ifdef ENDSTOP_CLEARANCE_Y
  #define SEARCH_FAST_Y (uint32_t)((double)60. * \
            sqrt((double)2 * ACCELERATION * ENDSTOP_CLEARANCE_Y / 1000.))
#endif
#ifdef ENDSTOP_CLEARANCE_Z
  #define SEARCH_FAST_Z (uint32_t)((double)60. * \
            sqrt((double)2 * ACCELERATION * ENDSTOP_CLEARANCE_Z / 1000.))
#endif
#ifdef ENDSTOP_CLEARANCE_U
#define SEARCH_FAST_U (uint32_t)((double)60. * \
            sqrt((double)2 * ACCELERATION * ENDSTOP_CLEARANCE_U / 1000.))
#endif


/// home all 4 axes
void home() {

#if defined	X_MIN_PIN
    home_x_negative();
#elif defined X_MAX_PIN
    home_x_positive();
#endif

#if defined	Y_MIN_PIN
    home_y_negative();
#elif defined Y_MAX_PIN
    home_y_positive();
#endif

#if defined	Z_MIN_PIN
    home_z_negative();
#elif defined Z_MAX_PIN
    home_z_positive();
#endif

#if defined	U_MIN_PIN
    home_u_negative();
#elif defined U_MAX_PIN
    home_u_positive();
#endif
}

/// find X MIN endstop
void home_x_negative() {
	#if defined X_MIN_PIN
		TARGET t = startpoint;

    t.axis[X] = -1000000;
    if (SEARCH_FAST_X > SEARCH_FEEDRATE_X) // Preprocessor can't check this :-/
      t.F = SEARCH_FAST_X;
    else
      t.F = SEARCH_FEEDRATE_X;
    enqueue_home(&t, 0x01, 1);

    if (SEARCH_FAST_X > SEARCH_FEEDRATE_X) {
			// back off slowly
      t.axis[X] = +1000000;
			t.F = SEARCH_FEEDRATE_X;
      enqueue_home(&t, 0x01, 0);
    }

		// set X home
		queue_wait(); // we have to wait here, see G92
		#ifdef X_MIN
      startpoint.axis[X] = next_target.target.axis[X] = (int32_t)(X_MIN * 1000.0);
		#else
      startpoint.axis[X] = next_target.target.axis[X] = 0;
		#endif
		dda_new_startpoint();
	#endif
}

/// find X_MAX endstop
void home_x_positive() {
	#if defined X_MAX_PIN && ! defined X_MAX
		#error X_MAX_PIN defined, but not X_MAX. home_x_positive() disabled.
	#endif
	#if defined X_MAX_PIN && defined X_MAX
		TARGET t = startpoint;

    t.axis[X] = +1000000;
    if (SEARCH_FAST_X > SEARCH_FEEDRATE_X)
      t.F = SEARCH_FAST_X;
    else
      t.F = SEARCH_FEEDRATE_X;
    enqueue_home(&t, 0x02, 1);

    if (SEARCH_FAST_X > SEARCH_FEEDRATE_X) {
      t.axis[X] = -1000000;
			t.F = SEARCH_FEEDRATE_X;
      enqueue_home(&t, 0x02, 0);
    }

		// set X home
		queue_wait();
		// set position to MAX
    startpoint.axis[X] = next_target.target.axis[X] = (int32_t)(X_MAX * 1000.);
		dda_new_startpoint();
	#endif
}

/// fund Y MIN endstop
void home_y_negative() {
	#if defined Y_MIN_PIN
		TARGET t = startpoint;

    t.axis[Y] = -1000000;
    if (SEARCH_FAST_Y > SEARCH_FEEDRATE_Y)
      t.F = SEARCH_FAST_Y;
    else
      t.F = SEARCH_FEEDRATE_Y;
    enqueue_home(&t, 0x04, 1);

    if (SEARCH_FAST_Y > SEARCH_FEEDRATE_Y) {
      t.axis[Y] = +1000000;
			t.F = SEARCH_FEEDRATE_Y;
      enqueue_home(&t, 0x04, 0);
    }

		// set Y home
		queue_wait();
		#ifdef	Y_MIN
      startpoint.axis[Y] = next_target.target.axis[Y] = (int32_t)(Y_MIN * 1000.);
		#else
      startpoint.axis[Y] = next_target.target.axis[Y] = 0;
		#endif
		dda_new_startpoint();
	#endif
}

/// find Y MAX endstop
void home_y_positive() {
	#if defined Y_MAX_PIN && ! defined Y_MAX
		#error Y_MAX_PIN defined, but not Y_MAX. home_y_positive() disabled.
	#endif
	#if defined Y_MAX_PIN && defined Y_MAX
		TARGET t = startpoint;

    t.axis[Y] = +1000000;
    if (SEARCH_FAST_Y > SEARCH_FEEDRATE_Y)
      t.F = SEARCH_FAST_Y;
    else
      t.F = SEARCH_FEEDRATE_Y;
    enqueue_home(&t, 0x08, 1);

    if (SEARCH_FAST_Y > SEARCH_FEEDRATE_Y) {
      t.axis[Y] = -1000000;
			t.F = SEARCH_FEEDRATE_Y;
      enqueue_home(&t, 0x08, 0);
    }

		// set Y home
		queue_wait();
		// set position to MAX
    startpoint.axis[Y] = next_target.target.axis[Y] = (int32_t)(Y_MAX * 1000.);
		dda_new_startpoint();
	#endif
}

/// find Z MIN endstop
void home_z_negative() {
	#if defined Z_MIN_PIN
		TARGET t = startpoint;

    t.axis[Z] = -1000000;
    if (SEARCH_FAST_Z > SEARCH_FEEDRATE_Z)
      t.F = SEARCH_FAST_Z;
    else
      t.F = SEARCH_FEEDRATE_Z;
    enqueue_home(&t, 0x10, 1);

    if (SEARCH_FAST_Z > SEARCH_FEEDRATE_Z) {
      t.axis[Z] = +1000000;
			t.F = SEARCH_FEEDRATE_Z;
      enqueue_home(&t, 0x10, 0);
    }

		// set Z home
		queue_wait();
		#ifdef Z_MIN
      startpoint.axis[Z] = next_target.target.axis[Z] = (int32_t)(Z_MIN * 1000.);
		#else
      startpoint.axis[Z] = next_target.target.axis[Z] = 0;
		#endif
		dda_new_startpoint();
	#endif
}

/// find Z MAX endstop
void home_z_positive() {
	#if defined Z_MAX_PIN && ! defined Z_MAX
		#error Z_MAX_PIN defined, but not Z_MAX. home_z_positive() disabled.
	#endif
	#if defined Z_MAX_PIN && defined Z_MAX
		TARGET t = startpoint;

    t.axis[Z] = +1000000;
    if (SEARCH_FAST_Z > SEARCH_FEEDRATE_Z)
      t.F = SEARCH_FAST_Z;
    else
      t.F = SEARCH_FEEDRATE_Z;
    enqueue_home(&t, 0x20, 1);

    if (SEARCH_FAST_Z > SEARCH_FEEDRATE_Z) {
      t.axis[Z] = -1000000;
			t.F = SEARCH_FEEDRATE_Z;
      enqueue_home(&t, 0x20, 0);
    }

		// set Z home
		queue_wait();
		// set position to MAX
    startpoint.axis[Z] = next_target.target.axis[Z] = (int32_t)(Z_MAX * 1000.);
		dda_new_startpoint();
	#endif
}

/// find U MIN endstop
void home_u_negative() {
#if defined U_MIN_PIN
    TARGET t = startpoint;

    t.axis[U] = -1000000;
    if (SEARCH_FAST_U > SEARCH_FEEDRATE_U)
        t.F = SEARCH_FAST_U;
    else
        t.F = SEARCH_FEEDRATE_U;
    enqueue_home(&t, 0x10, 1);

    if (SEARCH_FAST_U > SEARCH_FEEDRATE_U) {
        t.axis[U] = +1000000;
        t.F = SEARCH_FEEDRATE_U;
        enqueue_home(&t, 0x10, 0);
    }

    // set U home
    queue_wait();
#ifdef U_MIN
    startpoint.axis[U] = next_target.target.axis[U] = (int32_t)(U_MIN * 1000.);
#else
    startpoint.axis[U] = next_target.target.axis[U] = 0;
#endif
    dda_new_startpoint();
#endif
}

/// find U MAX endstop
void home_u_positive() {
#if defined U_MAX_PIN && ! defined U_MAX
    #error U_MAX_PIN defined, but not U_MAX.home_u_positive() disabled.
#endif
#if defined U_MAX_PIN && defined U_MAX
        TARGET t = startpoint;

    t.axis[U] = +1000000;
    if (SEARCH_FAST_U > SEARCH_FEEDRATE_U)
        t.F = SEARCH_FAST_U;
    else
        t.F = SEARCH_FEEDRATE_U;
    enqueue_home(&t, 0x20, 1);

    if (SEARCH_FAST_U > SEARCH_FEEDRATE_U) {
        t.axis[U] = -1000000;
        t.F = SEARCH_FEEDRATE_U;
        enqueue_home(&t, 0x20, 0);
    }

    // set U home
    queue_wait();
    // set position to MAX
    startpoint.axis[U] = next_target.target.axis[U] = (int32_t)(U_MAX * 1000.);
    dda_new_startpoint();
#endif
}
