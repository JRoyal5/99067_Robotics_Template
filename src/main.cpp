#include "main.h"
using namespace pros;
using namespace std;
#include <iostream>
#include <string>
#include "robot.h"

/*  This is the constructor of the robot object.
*   You can name it whatever you want. In this
*   case it is named robot. To initialize the 
*   motors, use {} for the left and right motors.
*   Then type the port number that the motor uses.
*   If you need to reverse the direction of the 
*   motor, simply put a negative before the port
*   number.
*/

Robot robot = Robot({-1,-3,-5}, {2,4,6}, 16, {7.25, 0.6, 3.25, 1800});
//This is how you define a piston

//ADIAnalogOut piston('A');

// This is how you define a sensor that use the three wire connection that pistons use
 
// ADIAnalogIn sensor('A');

//Don't mess with these
void on_center_button() {
    static bool pressed = false;
    pressed = !pressed;
    if (pressed) {
        lcd::set_text(2, "I was pressed!");
    } else {
        lcd::clear_line(2);
    }
}
void initialize() {
    lcd::initialize();
}

//This is the segment that allows you to control your robot during the driver period
void opcontrol() {
    Controller master(E_CONTROLLER_MASTER);

    while (true) {
        /*This is how to use a trigger or a button, specifically the R1 trigger.
        * When it just says get_digital, that means it will be true 
        * while the trigger is pressed and false when released. 
        * If you set it to get_digital_new_press it will become true 
        * when you press it, then false when you press it again.
        */

        if (master.get_digital(DIGITAL_R1)) {
            robot.driveFront(24,100);
        } //else {
        if (master.get_digital(DIGITAL_L1)) {
            robot.driveBack(24,100);
        }
        if (master.get_digital(DIGITAL_R2)) {
            robot.arcFront(24,270,80);
        }
        if (master.get_digital(DIGITAL_L2)) {
            robot.arcFront(24,90,80);
        }
        if (master.get_digital(DIGITAL_A)) {
            robot.turn(90,100);
        }
        // }

        // if (master.get_digital_new_press(DIGITAL_X)) {
        
        // } else{

        // }
        
        /*This is the method that reads the joystick input to
        * control your drive. Currently it is in arcade mode(controlled by one joystick)
        * and reads the left joystick. To change this replace the LEFT in 
        * E_CONTROLLER_ANALOG_LEFT_Y with RIGHT, then do the same to the 
        * E_CONTROLLER_ANALOG_LEFT_X. To do what is called tank drive(where
        * each side of the robot is controlled by a different joystick)
        * change LEFT in E_CONTROLLER_ANALOG_LEFT_X with RIGHT and the X with Y.
        * Then change the (yval+xval) with yval and (yval-xval) with xval.
        */
        int yval = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
        int xval = master.get_analog(E_CONTROLLER_ANALOG_LEFT_X);
        // Drive function for 4-motor drive
        robot.drive((yval+xval),(yval-xval));
    }
    pros::delay(15);
}

//This is the segment that allows your robot to move during the autonomous period
void autonomous() {
    robot.arcFront(24, 90, 40);
}
