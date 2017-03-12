/* Auth: Megan Bird & Gary Miller
 * File: main.c
 * Course: CEEN-3450 – Mobile Robotics I – University of Nebraska-Lincoln
 * Lab: Lab 6
 * Date: 3/8/2017
 * Desc: Behavior Based Control
 */

// Behavior-Based Control Skeleton code.
//
// Desc: Provides a C program structure that emulates multi-tasking and
//       modularity for Behavior-based control with easy scalability.
//
// Supplied for: Students of Mobile Robotics I, Fall 2013.
// University of Nebraska-Lincoln Dept. of Computer & Electronics Engineering
// Alisa N. Gilmore, P.E., Instructor, Course Developer.  Jose Santos, T.A.
// Version 1.3  Updated 10/11/2011.
// Version 1.4  Updated 12/2/2013
//
//      - Updated __MOTOR_ACTION() macro-function to invoke new functions
//        added to the API: `STEPPER_set_accel2()' and `STEPPER_run2()'.
//        In particular `STEPPER_run2()' now takes positive or negative
//        speed values to imply the direction of each wheel.
//
// Version 1.5  Updated 2/25/2015
//
//      - Fixed an error in __MOTOR_ACTION() and __RESET_ACTION() macros
//        where there was an extra curly brace ({) that should have been
//        removed.
//
// Version 1.6  Updated 2/24/2016
//
//      - In the 'IR_sense()' function, we now make use of the 'TIMER_ALARM()'
//        and 'TIMER_SNOOZE()' macros that were introduced in API version 2.x,
//        which makes usage of the timer object clear.   Before, students
//        manipulated the 'tc' flag inside the timer object directly, but this
//        always caused confusion, the 'TIMER_ALARM()' and 'TIMER_SNOOZE()'
//        macros achieve the same thing, but transparently.
//
//      - Also fixed __MOTOR_ACTION() macro, which previously invoked
//        'STEPPER_run2()', but the API has been modified so that this same
//        effect is now achieved with the function 'STEPPER_runn()', which
//        means to run the stepper motors and allow negative values to mean
//        reverse motion (that's what the second 'n' in 'runn' stands for).
//        This might change again in the future, because for consistency in the
//        API, it should have been called 'STEPPER_runn2()' since it takes two
//        parameters and not just one.
//

#include "capi324v221.h"

// ---------------------- Defines:

#define DEG_90  135     /* Number of steps for a 90-degree (in place) turn. */


// Desc: This macro-function can be used to reset a motor-action structure
//       easily.  It is a helper macro-function.
#define __RESET_ACTION( motor_action )    \
do {									  \
	( motor_action ).speed_L = 0;         \
	( motor_action ).speed_R = 0;         \
	( motor_action ).accel_L = 0;         \
	( motor_action ).accel_R = 0;         \
	( motor_action ).state = STARTUP;     \
	} while( 0 ) /* end __RESET_ACTION() */



	// Desc: This macro-fuction translates action to motion -- it is a helper
	//       macro-function.
	#define __MOTOR_ACTION( motor_action )   \
	do {                                     \
		STEPPER_set_accel2( ( motor_action ).accel_L, ( motor_action ).accel_R ); \
		STEPPER_runn( ( motor_action ).speed_L, ( motor_action ).speed_R );       \
		} while( 0 ) /* end __MOTOR_ACTION() */

		// Desc: This macro-function is used to set the action, in a more natural
		//       manner (as if it was a function).



		// ---------------------- Type Declarations:


		// Desc: The following custom enumerated type can be used to specify the
		//       current state of the robot.  This parameter can be expanded upon
		//       as complexity grows without intefering with the 'act()' function.
		//		 It is a new type which can take the values of 0, 1, or 2 using
		//		 the SYMBOLIC representations of STARTUP, EXPLORING, etc.
		typedef enum ROBOT_STATE_TYPE {

			STARTUP = 0,    // 'Startup' state -- initial state upon RESET.
			CRUISING,       // 'Cruising' state -- the robot is 'roaming around'.
			HOMING,		    // 'Homing' state -- the robot is 'homing towards the light'.
			AVOIDING        // 'Avoiding' state -- the robot is avoiding a collision.

		} ROBOT_STATE;



		
		// Desc: Structure encapsulates a 'motor' action. It contains parameters that
		//       controls the motors 'down the line' with information depicting the
		//       current state of the robot.  The 'state' variable is useful to
		//       'print' information on the LCD based on the current 'state', for
		//       example.
		typedef struct MOTOR_ACTION_TYPE {

			ROBOT_STATE state;              // Holds the current STATE of the robot.
			signed short int speed_L;       // SPEED for LEFT  motor.
			signed short int speed_R;       // SPEED for RIGHT motor.
			unsigned short int accel_L;     // ACCELERATION for LEFT  motor.
			unsigned short int accel_R;     // ACCELERATION for RIGHT motor.
			
		} MOTOR_ACTION;


		
		// Desc: Structure encapsulates 'sensed' data.  Right now that only consists
		//       of the state of the left & right IR sensors when queried.  You can
		//       expand this structure and add additional custom fields as needed.
		typedef struct SENSOR_DATA_TYPE {

			BOOL left_IR;       // Holds the state of the left IR.
			BOOL right_IR;      // Holds the state of the right IR.

			float left_photo_voltage;	// Holds the value of the left photo-sensor
			float right_photo_voltage;	// Holds the value of the right photo-sensor
			
			float left_photo_ambient;	// Holds the initial ambient value of the right photo-sensor
			float right_photo_ambient;	// Holds the initial ambient value of the left photo-sensor

			float sonar_dist;	// Holds the value for the sonar distance, in centimeters

		} SENSOR_DATA;


		// ------------------------------
		// ---------------------- Globals:
		volatile MOTOR_ACTION action;  	// This variable holds parameters that determine
		// the current action that is taking place.
		// Here, a structure named "action" of type
		// MOTOR_ACTION is declared.

		// ---------------------------------
		// ---------------------- Prototypes:
		void IR_sense( volatile SENSOR_DATA *pSensors, TIMER16 interval_ms );
		void Sonar_sense( volatile SENSOR_DATA *pSensors, TIMER16 interval_ms);
		void Cruise( volatile MOTOR_ACTION *pAction );
		void Light_Follow( volatile MOTOR_ACTION *pAction, volatile SENSOR_DATA *pSensors );
		void IR_avoid( volatile MOTOR_ACTION *pAction, volatile SENSOR_DATA *pSensors );
		void Sonar_Avoid( volatile MOTOR_ACTION *pAction, volatile SENSOR_DATA *pSensors);
		void act( volatile MOTOR_ACTION *pAction );
		void info_display( volatile MOTOR_ACTION *pAction );
		BOOL compare_actions( volatile MOTOR_ACTION *a, volatile MOTOR_ACTION *b );




		// ---------------------- Convenience Functions: -----------------------------------------------------------------------------------------------------//
		// ---------------------------------------------------------------------------------------------------------------------------------------------------//
		void info_display( volatile MOTOR_ACTION *pAction )
		{

			// NOTE:  We keep track of the 'previous' state to prevent the LCD
			//        display from being needlessly written, if there's  nothing
			//        new to display.  Otherwise, the screen will 'flicker' from
			//        too many writes.
			static ROBOT_STATE previous_state = STARTUP;

			if ( ( pAction->state != previous_state ) || ( pAction->state == STARTUP ) )
			{

				LCD_clear();

				//  Display information based on the current 'ROBOT STATE'.
				switch( pAction->state )
				{

					case STARTUP:
					LCD_printf( "STARTING...\n");
					break;

					case CRUISING:
					LCD_printf( "CRUISING...\n" );
					break;

					case AVOIDING:
					LCD_printf( "AVOIDING...\n");
					break;

					case HOMING:
					LCD_printf( "HOMING...\n");
					break;

					default:
					LCD_printf( "Unknown state!\n" );

				} // end switch()

				// Note the new state in effect.
				previous_state = pAction->state;

			} // end if()

		} // end info_display()


		// ------------------------------------------------------------------------------------------------------------------------------------------------ //
		BOOL compare_actions( volatile MOTOR_ACTION *a, volatile MOTOR_ACTION *b )
		{

			// NOTE:  The 'sole' purpose of this function is to
			//        compare the 'elements' of MOTOR_ACTION structures
			//        'a' and 'b' and see if 'any' differ.

			// Assume these actions are equal.
			BOOL rval = TRUE;

			if ( ( a->state   != b->state )   ||
			( a->speed_L != b->speed_L ) ||
			( a->speed_R != b->speed_R ) ||
			( a->accel_L != b->accel_L ) ||
			( a->accel_R != b->accel_R ) )

			rval = FALSE;

			// Return comparison result.
			return rval;

		} // end compare_actions()


		// ---------------------- Top-Level Behaviorals: ----------------------------------------------------------------------------------------------------- //
		// --------------------------------------------------------------------------------------------------------------------------------------------------- //
		void IR_sense( volatile SENSOR_DATA *pSensors, TIMER16 interval_ms )
		{

			// Sense must know if it's already sensing.
			//
			// NOTE: 'BOOL' is a custom data type offered by the CEENBoT API.
			//
			static BOOL timer_started = FALSE;
			
			// The 'sense' timer is used to control how often gathering sensor
			// data takes place.  The pace at which this happens needs to be
			// controlled.  So we're forced to use TIMER OBJECTS along with the
			// TIMER SERVICE.  It must be 'static' because the timer object must remain
			// 'alive' even when it is out of scope -- otherwise the program will crash.
			static TIMEROBJ sense_timer;
			
			// If this is the FIRST time that sense() is running, we need to start the
			// sense timer.  We do this ONLY ONCE!
			if ( timer_started == FALSE )
			{
				
				// Start the 'sense timer' to tick on every 'interval_ms'.
				//
				// NOTE:  You can adjust the delay value to suit your needs.
				//
				TMRSRVC_new( &sense_timer, TMRFLG_NOTIFY_FLAG, TMRTCM_RESTART,
				interval_ms );
				
				// Mark that the timer has already been started.
				timer_started = TRUE;
				
			} // end if()
			
			// Otherwise, just do the usual thing and just 'sense'.
			else
			{

				// Only read the sensors when it is time to do so (e.g., every
				// 125ms).  Otherwise, do nothing.
				if ( TIMER_ALARM( sense_timer ) )
				{

					// NOTE: Just as a 'debugging' feature, let's also toggle the green LED
					//       to know that this is working for sure.  The LED will only
					//       toggle when 'it's time'.
					LED_toggle( LED_Green );


					// Read the left and right sensors, and store this
					// data in the 'SENSOR_DATA' structure.
					pSensors->left_IR  = ATTINY_get_IR_state( ATTINY_IR_LEFT  );
					pSensors->right_IR = ATTINY_get_IR_state( ATTINY_IR_RIGHT );

					

					// NOTE: You can add more stuff to 'sense' here.
					
					// Snooze the alarm so it can trigger again.
					TIMER_SNOOZE( sense_timer );
					
				} // end if()

			} // end else.

		} // end sense()

		// ----------------------------------------------------------------------------------------------------------------------------------------- //
		void Photo_sense( volatile SENSOR_DATA *pSensors, TIMER16 interval_ms )
		{
			static BOOL timer_started = FALSE;   //  Check if photo-sense is already running

			static TIMEROBJ sense_timer;         // Used to control the pace at which sensor data is gathered

			if( timer_started == FALSE )		// If this is first time sense() runs, start the photo-sense timer.  This happens only once!!!
			{
				TMRSRVC_new( &sense_timer, TMRFLG_NOTIFY_FLAG, TMRTCM_RESTART, interval_ms);	// Start the photo-sense timer, adjusted to tick every 'interval_ms'

				timer_started = TRUE;			// Mark that timer is started
			}
			else
			{
				if( TIMER_ALARM( sense_timer ) )
				{
					LED_toggle( LED_Red );		// for debugging, to make sure photo-sensing is occurring
					ADC_SAMPLE sample;

					ADC_set_channel(ADC_CHAN6);
					sample = ADC_sample();
					pSensors->left_photo_voltage = ((sample * 5.0f) / 1024);

					ADC_set_channel(ADC_CHAN4);
					sample = ADC_sample();
					pSensors->right_photo_voltage = ((sample * 5.0f) / 1024);

					// Snooze the alarm so it can trigger again.
					TIMER_SNOOZE( sense_timer );
				}
			}
		}  // end Photo_sense()

		// ----------------------------------------------------------------------------------------------------------------------------------------- //

		void Sonar_sense( volatile SENSOR_DATA *pSensors, TIMER16 interval_ms )
		{
			static BOOL timer_started = FALSE;

			static TIMEROBJ sense_timer;

			if(timer_started == FALSE)
			{
				TMRSRVC_new( &sense_timer, TMRFLG_NOTIFY_FLAG, TMRTCM_RESTART, interval_ms);
				timer_started = TRUE;
			}
			else
			{
				if(TIMER_ALARM( sense_timer))
				{
					// Led on sonar is used for debugging

					//unsigned long int usonic_time_us;		// Do we need these two?
					//SWTIME usonic_time_ticks;				// They are not used.
					float distance_cm;

					TMRSRVC_delay_ms(100);

					distance_cm = USONIC_DIST_CM(USONIC_ping());

					pSensors->sonar_dist = distance_cm;

					//LCD_clear();	// Good for sensor setup, but we want LCD to display the selected behavior
					//LCD_printf( "Dist = %.3f\n", distance_cm);
					TMRSRVC_delay_ms(100);

					TIMER_SNOOZE(sense_timer);
				}
			}
		} // end Sonar_Sense()

		// ----------------------------------------------------------------------------------------------------------------------------------------- //
		void Photo_init( volatile SENSOR_DATA *pSensors )
		{
			LED_toggle( LED_Red );		// for debugging, to make sure photo-sensing is occurring
			ADC_SAMPLE sample;

			ADC_set_channel(ADC_CHAN6);
			sample = ADC_sample();
			pSensors->left_photo_ambient = ((sample * 5.0f) / 1024);

			ADC_set_channel(ADC_CHAN4);
			sample = ADC_sample();
			pSensors->right_photo_ambient = ((sample * 5.0f) / 1024);
		} // end Photo_init()

		// ----------------------------------------------------------------------------------------------------------------------------------------- //
		void Cruise( volatile MOTOR_ACTION *pAction )
		{
			// Nothing to do, but set the parameters to explore.  'act()' will do
			// the rest down the line.
			pAction->state = CRUISING;
			pAction->speed_L = 200;
			pAction->speed_R = 200;
			pAction->accel_L = 400;
			pAction->accel_R = 400;
			
			// That's it -- let 'act()' do the rest.
			
		} // end Cruise()

		// ------------------------------------------------------------------------------------------------------------------------------------------ //
		void IR_avoid( volatile MOTOR_ACTION *pAction, volatile SENSOR_DATA *pSensors )
		{

			// NOTE: Here we have NO CHOICE, but to do this 'ballistically'.
			//       **NOTHING** else can happen while we're 'avoiding'.
			
			// If the LEFT sensor tripped...
			if( pSensors->left_IR == TRUE )
			{

				// Note that we're avoiding...
				pAction->state = AVOIDING;
				LCD_clear();
				LCD_printf( "AVOIDING...\n");

				// STOP!
				STEPPER_stop( STEPPER_BOTH, STEPPER_BRK_OFF );
				
				// Back up...
				STEPPER_move_stwt( STEPPER_BOTH,
						STEPPER_REV, 250, 200, 400, STEPPER_BRK_OFF,
						STEPPER_REV, 250, 200, 400, STEPPER_BRK_OFF );
				
				// ... and turn RIGHT ~90-deg.
				STEPPER_move_stwt( STEPPER_BOTH,
						STEPPER_FWD, DEG_90, 200, 400, STEPPER_BRK_OFF,
						STEPPER_REV, DEG_90, 200, 400, STEPPER_BRK_OFF );

				// ... and set the motor action structure with variables to move forward.
				pAction->state = AVOIDING;
				pAction->speed_L = 200;
				pAction->speed_R = 200;
				pAction->accel_L = 400;
				pAction->accel_R = 400;
				
			} 
			if( pSensors->right_IR == TRUE)
			{
				pAction->state = AVOIDING;
				LCD_clear();
				LCD_printf( "AVOIDING...\n");

				STEPPER_stop(STEPPER_BOTH, STEPPER_BRK_OFF);

				// Back up...
				STEPPER_move_stwt( STEPPER_BOTH,
						STEPPER_REV, 250, 200, 400, STEPPER_BRK_OFF,
						STEPPER_REV, 250, 200, 400, STEPPER_BRK_OFF );	
						
				// ... and turn LEFT ~90-deg
				STEPPER_move_stwt( STEPPER_BOTH,
						STEPPER_REV, DEG_90, 200, 400, STEPPER_BRK_OFF,
						STEPPER_FWD, DEG_90, 200, 400, STEPPER_BRK_OFF);	

				// ... and set the motor action structure with variables to move forward.
				pAction->state = AVOIDING;
				pAction->speed_L = 200;
				pAction->speed_R = 200;
				pAction->accel_L = 400;
				pAction->accel_R = 400;
			}
			if( pSensors->right_IR == TRUE && pSensors->left_IR == TRUE)
			{
				pAction->state = AVOIDING;
				LCD_clear();
				LCD_printf( "AVOIDING...\n");

				STEPPER_stop(STEPPER_BOTH, STEPPER_BRK_OFF);

				// Back up... further
				STEPPER_move_stwt( STEPPER_BOTH,
						STEPPER_REV, 500, 200, 400, STEPPER_BRK_OFF,
						STEPPER_REV, 500, 200, 400, STEPPER_BRK_OFF );

				// ... and turn LEFT ~120-deg
				STEPPER_move_stwt( STEPPER_BOTH,
						STEPPER_REV, 175, 200, 400, STEPPER_BRK_OFF,
						STEPPER_FWD, 175, 200, 400, STEPPER_BRK_OFF);


				// ... and set the motor action structure with variables to move forward.
				pAction->state = AVOIDING;
				pAction->speed_L = 200;
				pAction->speed_R = 200;
				pAction->accel_L = 400;
				pAction->accel_R = 400;

			}
		} // end avoid()

		// --------------------------------------------------------------------------------------------------------------------------- //
		void Light_Follow(volatile MOTOR_ACTION *pAction, volatile SENSOR_DATA *pSensors)
		{
			float light_min = (( 5 - (pSensors->left_photo_ambient + pSensors->right_photo_ambient)/2 ) * 0.2 )  
								+ ((pSensors->left_photo_ambient + pSensors->right_photo_ambient)/2);
			float base_speed = 200;

			float adjusted_left = pSensors->left_photo_voltage - pSensors->left_photo_ambient;
			float adjusted_right = pSensors->right_photo_voltage - pSensors->right_photo_ambient;
			
			// minimum adjusted value is 0
			if ( adjusted_right < 0 ) {
				adjusted_right = 0;
			}
			if ( adjusted_left < 0 ) {
				adjusted_left = 0;
			}
			
			float percentage_left = adjusted_left / ( 5 - pSensors->left_photo_ambient);
			float percentage_right = adjusted_right / ( 5 - pSensors->right_photo_ambient);
			
			float right_minus_left = percentage_right - percentage_left;
			
			if ( (pSensors->left_photo_voltage + pSensors->right_photo_voltage)/2 > light_min)
			{
				pAction->state = HOMING;

				pAction->speed_L = base_speed*( 1 + right_minus_left );
				pAction->speed_R = base_speed*( 1 - right_minus_left );
			}
		}  // end Light_Follow()

		// --------------------------------------------------------------------------------------------------------------------------- //
		void Sonar_Avoid( volatile MOTOR_ACTION *pAction, volatile SENSOR_DATA *pSensors)
		{

		} // end Sonar_Avoid()

		// --------------------------------------------------------------------------------------------------------------------------- //
		void act( volatile MOTOR_ACTION *pAction )
		{

			// 'act()' always keeps track of the PREVIOUS action to determine
			// if a new action must be executed, and to execute such action ONLY
			// if any parameters in the 'MOTOR_ACTION' structure have changed.
			// This is necessary to prevent motor 'jitter'.
			static MOTOR_ACTION previous_action = {

				STARTUP, 0, 0, 0, 0

			};

			if( compare_actions( pAction, &previous_action ) == FALSE )
			{

				// Perform the action.  Just call the 'free-running' version
				// of stepper move function and feed these same parameters.
				__MOTOR_ACTION( *pAction );

				// Save the previous action.
				previous_action = *pAction;

			} // end if()
			
		} // end act()

		// ---------------------- CBOT Main ---------------------------------------------------------------------------------------------- //
		// ------------------------------------------------------------------------------------------------------------------------------- //
		void CBOT_main( void )
		{

			volatile SENSOR_DATA sensor_data;
			
			// ** Open the needed modules.
			LED_open();     // Open the LED subsystem module.
			LCD_open();     // Open the LCD subsystem module.
			STEPPER_open(); // Open the STEPPER subsystem module.
			ADC_open();
			ADC_set_VREF(ADC_VREF_AVCC);	// set ADC reference to 5V

			STOPWATCH_open();
			USONIC_open();
			
			// Reset the current motor action.
			__RESET_ACTION( action );
			
			// Notify program is about to start.
			LCD_printf( "Starting...\n" );
			
			// Wait 3 seconds or so.
			TMRSRVC_delay( TMR_SECS( 3 ) );
			
			// Take initial ambient light sensor readings
			Photo_init( &sensor_data );
			
			// Clear the screen and enter the arbitration loop.
			LCD_clear();
			
			// Enter the 'arbitration' while() loop -- it is important that NONE
			// of the behavior functions listed in the arbitration loop BLOCK!
			// Behaviors are listed in increasing order of priority, with the last
			// behavior having the greatest priority (because it has the last 'say'
			// regarding motor action (or any action)).
			while( 1 )
			{
				// Sense must always happen first.
				// (IR sense happens every 125ms).
				IR_sense( &sensor_data, 125 );
				Photo_sense( &sensor_data, 250 );
				Sonar_sense( &sensor_data, 100 );
				
				// Behaviors.
				Cruise( &action );
				Light_Follow( &action, &sensor_data );
				Sonar_Avoid( &action, &sensor_data);
				IR_avoid( &action, &sensor_data );
				
				
				// Perform the action of highest priority.
				act( &action );

				// Real-time display info, should happen last, if possible (
				// except for 'ballistic' behaviors).  Technically this is sort of
				// 'optional' as it does not constitute a 'behavior'.
				info_display( &action );
				
			} // end while()
			
		} // end CBOT_main()

