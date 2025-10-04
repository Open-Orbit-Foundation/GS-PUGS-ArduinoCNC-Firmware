/*!
 * @file test_EL_homing.ino
 *
 * This is a test file for elevation homing switch readout only.
 * No stepper motor control is included in this test.
 * @section license License
 *
 *
 * Licensed under the GPLv3.
 *
 */

 #define DEFAULT_HOME_STATE LOW  ///< Change to LOW according to Home sensor

 #include <Wire.h>
 #include <globals.h>
 #include <easycomm.h>
 #include <rotator_pins.h>
 #include <endstop.h>
 // NESSECARY FOR RPI INTERFACE
 //#include <rs485.h>
 //#include <watchdog.h>
 
 // OBJECTS
 uint32_t t_run = 0; // run time of uC
 endstop switch_el(SW2, DEFAULT_HOME_STATE);
 void setup() {
     // Initialize serial communication
 
     Serial.begin(115200);
     
     // Homing switch initialization
     switch_el.init();
     
     Serial.println("[setup] Elevation homing switch test initiated.");
     Serial.print("[setup] Initial state - Elevation Endstop: ");
     Serial.println(switch_el.get_state() ? "✅ TRIGGERED" : "❌ NOT TRIGGERED");
     Serial.println("[setup] Test ready. Switch state will be displayed every second.");
 }
 
 void loop() {
     // Get end stop status
     bool switch_state = switch_el.get_state();
     
     // Display switch state
     Serial.println(switch_state ? "✅ TRIGGERED" : "❌ NOT TRIGGERED");
     
     // Wait 1 second before next reading
     delay(500);
 }



// /*!
//  * @file stepper_motor_controller.ino
//  *
//  * This is the documentation for satnogs rotator controller firmware
//  * for stepper motors configuration. The board (PCB) is placed in
//  * <a href="https://gitlab.com/librespacefoundation/satnogs/satnogs-rotator-controller">
//  * satnogs-rotator-controller </a> and is for releases:
//  * v2.0
//  * v2.1
//  * v2.2
//  * <a href="https://wiki.satnogs.org/SatNOGS_Rotator_Controller"> wiki page </a>
//  *
//  * @section dependencies Dependencies
//  *
//  * This firmware depends on <a href="http://www.airspayce.com/mikem/arduino/AccelStepper/index.htmlhttp://www.airspayce.com/mikem/arduino/AccelStepper/index.html">
//  * AccelStepper library</a> being present on your system. Please make sure you
//  * have installed the latest version before using this firmware.
//  *
//  * @section license License
//  *
//  * Licensed under the GPLv3.
//  *
//  */

// #define SAMPLE_TIME        0.1   ///< Control loop in s
// #define RATIO              20    ///< Gear ratio of rotator gear box                                 default 54
// #define MICROSTEP          2     ///< Set Microstep
// #define MIN_PULSE_WIDTH    20    ///< In microsecond for AccelStepper
// #define MAX_SPEED          3200  ///< In steps/s, consider the microstep
// #define MAX_ACCELERATION   1600  ///< In steps/s^2, consider the microstep
// #define SPR                200L ///< Step Per Revolution, consider the microstep
// #define MIN_M1_ANGLE       0     ///< Minimum angle of azimuth
// #define MAX_M1_ANGLE       360   ///< Maximum angle of azimuth
// #define MIN_M2_ANGLE       0     ///< Minimum angle of elevation
// #define MAX_M2_ANGLE       180   ///< Maximum angle of elevation
// #define DEFAULT_HOME_STATE LOW  ///< Change to LOW according to Home sensor
// #define HOME_DELAY         12000 ///< Time for homing Deceleration in millisecond