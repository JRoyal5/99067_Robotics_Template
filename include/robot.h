#include "main.h"
using namespace std;
using namespace pros;

class Robot{
    private:
        double driveBaseRadius = 0;
        double gearRatio = 0;
        double wheelDiameter = 0;
        double rpmMod = 0;
        double tickPerRev = 0;
    public:
        //Drive Motors
        MotorGroup leftMotors;
        MotorGroup rightMotors;
        MotorGroup assorted;
        IMU imu_sensor;
        IMU imu_sensor2;
        
                
        //Constructor
        Robot(std::initializer_list<std::int8_t> leftMotors, std::initializer_list<std::int8_t> rightMotors, int imu_sensor, int imu_sensor2, std::vector<int8_t> ports) 
        : leftMotors(leftMotors), rightMotors(rightMotors), imu_sensor(imu_sensor), imu_sensor2(imu_sensor2), assorted(ports) {

        }

        //Methods
        void addConstants(double d, double g, double w, double r, double t){
            driveBaseRadius = d;
            gearRatio = g;
            wheelDiameter = w;
            rpmMod = r;
            tickPerRev = t;
        }

        void setBrakeMode(int i){
            if(i = 0){
                leftMotors.set_brake_modes(E_MOTOR_BRAKE_COAST);
                rightMotors.set_brake_modes(E_MOTOR_BRAKE_COAST);
                assorted.set_brake_modes(E_MOTOR_BRAKE_COAST);
            } else if(i = 1){
                leftMotors.set_brake_modes(E_MOTOR_BRAKE_BRAKE);
                rightMotors.set_brake_modes(E_MOTOR_BRAKE_BRAKE);
                assorted.set_brake_modes(E_MOTOR_BRAKE_BRAKE);
            } else if(i = 2){
                leftMotors.set_brake_modes(E_MOTOR_BRAKE_HOLD);
                rightMotors.set_brake_modes(E_MOTOR_BRAKE_HOLD);
                assorted.set_brake_modes(E_MOTOR_BRAKE_HOLD);
            }
        }
        //Driving methods
        void drive(double left, double right){
            leftMotors.move_velocity(left*rpmMod);
            rightMotors.move_velocity(right*rpmMod);
        }

        void driveFront(double inches, double velocity){
            leftMotors.set_zero_position(0);
            rightMotors.set_zero_position(0);

            
            vector<double> positionsL = leftMotors.get_positions();
            vector<double> positionsR = rightMotors.get_positions();
            int counter = 1;
            double currentDis = ((positionsL[0] + positionsR[0])/2);
            double currentDis2 = 0;
            double c = 0;
            int currentVelocity = (int) velocity;
            double tickPerInch = tickPerRev/(6*3.1415*wheelDiameter*gearRatio);
            double targetDis = (inches*tickPerInch);

            while(!(currentDis <= (targetDis+(tickPerInch/4)) && currentDis >= (targetDis-(tickPerInch/4))) || !(counter == 5)){
                positionsL = leftMotors.get_positions();
                positionsR = rightMotors.get_positions();
                currentDis = (positionsL[0] + positionsR[0])/2;
                if(currentDis>targetDis){
                    currentVelocity = -velocity;
                    drive((currentVelocity * rpmMod), (currentVelocity * rpmMod));
                } else if (currentDis < targetDis/2){
                    drive(velocity * rpmMod, velocity * rpmMod);
                } else if(currentDis < (((targetDis/2) * c) + (targetDis/2))){
                    c = ((currentDis - (targetDis/2))/(targetDis/2));
                    currentVelocity = velocity * (1-(c*c));
                    drive(currentVelocity * rpmMod,currentVelocity * rpmMod);
                } else {
                    c = ((currentDis+(targetDis/2))/(targetDis/2));
                    drive(currentVelocity * rpmMod,currentVelocity * rpmMod);
                }

                positionsL = leftMotors.get_positions();
                positionsR = rightMotors.get_positions();
                currentDis2 = (positionsL[0] + positionsR[0])/2;
                if ((currentDis2 <= (targetDis + (tickPerInch/4))) && (currentDis2 >= (targetDis - (tickPerInch/4)))){
                    counter++;
                    if(counter == 5){
                        drive(-velocity * rpmMod, -velocity * rpmMod);
                        pros::delay(50);
                        drive(0,0);
                        return;
                    }
                } else {
                    counter = 0;
                }
            }
            drive(0,0);
        }

        void driveBack(double inches, double velocity){
            //reset motor encoders
            leftMotors.set_zero_position(0);
            rightMotors.set_zero_position(0);

            vector<double> positionsL = leftMotors.get_positions();
            vector<double> positionsR = rightMotors.get_positions();
            int counter = 1;
            double currentDis = ((positionsL[0] + positionsR[0])/2);
            double currentDis2 = 0;
            double c = 0;
            double currentVelocity = velocity;
            double tickPerInch = tickPerRev/(6*3.1415*wheelDiameter*gearRatio);
            double targetDis = -(inches*tickPerInch);

            while(!(currentDis <= (targetDis-(tickPerInch/4)) && currentDis >= (targetDis+(tickPerInch/4))) || !(counter == 5)){
                positionsL = leftMotors.get_positions();
                positionsR = rightMotors.get_positions();
                currentDis = (positionsL[0] + positionsR[0])/2;
                if(currentDis < targetDis){
                    currentVelocity = velocity;
                    drive((currentVelocity * rpmMod),(currentVelocity * rpmMod));
                } else if (currentDis > targetDis/2){
                    drive(-velocity * rpmMod, -velocity * rpmMod);
                } else if(currentDis > (((targetDis/2)*c)+(targetDis/2))){
                    c = ((currentDis - (targetDis/2))/(targetDis/2));
                    currentVelocity = velocity * (1-(c*c));
                    drive(-currentVelocity * rpmMod, -currentVelocity * rpmMod);
                } else {
                    c = ((currentDis+(targetDis/2))/(targetDis/2));
                    drive(-currentVelocity * rpmMod, -currentVelocity * rpmMod);
                }

                currentDis2 = (positionsL[0] + positionsR[0])/2;
                if ((currentDis2 <= (targetDis + (tickPerInch/4))) && (currentDis2 >= (targetDis - (tickPerInch/4)))){
                    counter++;
                    if(counter == 5){
                        drive(velocity * rpmMod, velocity * rpmMod);
                        pros::delay(50);
                        drive(0,0);
                        return;
                    }
                } else {
                    counter = 0;
                }
            }
            drive(0,0);
        }

        //Turn methods
        //Requires inertial sensor
        void offSet(double offset){
            imu_sensor.set_heading(offset);
            imu_sensor2.set_heading(offset);
        }
        
        void turn(int target, int velocity){
   
            //Initializes everything
            int startingDis = 0;
            int currentDeg = (imu_sensor.get_heading() + imu_sensor2.get_heading())/2;
            int lcloseVal = 0;
            int rcloseVal = 0;
            int count = 0;

            //Determins what direction to turn
            if(target > currentDeg){
                lcloseVal = abs(currentDeg + abs(360 - target));
                //since current is less than target it needs to add on the distance target is from 360
                rcloseVal = abs(target - currentDeg);
                //gets the distance it is if it where to turn Right
            }else {
                lcloseVal = abs(currentDeg - target);
                //gets the distance it is if it where to turn Left
                rcloseVal = abs(target + abs(360 - currentDeg));
                //since Target is greater than current it needs to add on the distance current is from 360
            }

            if(lcloseVal < rcloseVal){
                startingDis = lcloseVal;
            } else{
                startingDis = rcloseVal;
            }
            //sets the starting distance based on which ever is shortest

            while((!(((currentDeg<=(target+5))) && (currentDeg>=(target-5))))){
                //Gives a little room for error so doesn't fight forever
                //Count so it dosent exit on firt read
                currentDeg = (imu_sensor.get_heading() + imu_sensor2.get_heading())/2;

                //Updates the current heading between 0 and 360
                // updateHeading(currentDeg);

                //Same logic as above to calulate the distance
                if(target > currentDeg){
                    lcloseVal = abs(currentDeg + abs(360 - target));
                    rcloseVal = abs(target - currentDeg);
                }else {
                    lcloseVal = abs(currentDeg - target);
                    rcloseVal = abs(target + abs(360 - currentDeg));
                }

                //Left Turn
                if(lcloseVal<rcloseVal){
                    //create a ratio of distance left and stat distance
                    double ratio = lcloseVal/startingDis;
                    string data = to_string(lcloseVal);
                    lcd::set_text(2, data);
                    if(lcloseVal < 15 && lcloseVal > -15){
                        drive(-20, 20);
                    }
                    else if(lcloseVal < startingDis*ratio){
                        drive(-velocity*ratio, velocity*ratio);
                    }
                }

                //Right Turn
                else {
                    //create a ratio of distance left and stat distance
                    double ratio = rcloseVal/startingDis;
                    string data = to_string(lcloseVal);
                    lcd::set_text(2, data);
                    if(rcloseVal < 15 && rcloseVal > -15){
                        drive(20, -20);
                    }
                    else if(rcloseVal < startingDis*ratio){
                        drive(velocity*ratio, -velocity*ratio);
                    }
                }

                //If within range increase count
                if (((currentDeg <= (target+5)) && (currentDeg >= (target-5)))){
                    count++;
                    if(count == 5){
                        return;
                    }
                }
                else {
                    count = 0;
                    //If not reset count
                }
                
                pros::delay(10);
                // delay matches the update delay of inertial sensor so it is not using one read for multiple times
            drive(0,0);
            }

        }

        void turnBy(int target, int velocity){
   
            //Initializes everything
            int startingDis = 0;
            int currentDeg = (imu_sensor.get_heading() + imu_sensor2.get_heading())/2;
            int lcloseVal = 0;
            int rcloseVal = 0;
            int count = 0;
            float lastPosition = 0;
            bool moving = false;
            bool left = false;

            // updateHeading(currentDeg);

            //Determins what direction to turn
            if(abs(target) != target){
                left = true;
            }

            if(target > currentDeg){
                lcloseVal = abs(currentDeg + abs(360 - target));
                //since current is less than target it needs to add on the distance target is from 360
                rcloseVal = abs(target - currentDeg);
                //gets the distance it is if it where to turn Right
            }else {
                lcloseVal = abs(currentDeg - target);
                //gets the distance it is if it where to turn Left
                rcloseVal = abs(target + abs(360 - currentDeg));
                //since Target is greater than current it needs to add on the distance current is from 360
            }

            while((!(((currentDeg<=(target+5))) && (currentDeg>=(target-5))))){
                //Gives a little room for error so doesn't fight forever
                //Count so it dosent exit on firt read
                currentDeg = (imu_sensor.get_heading() + imu_sensor2.get_heading())/2;
                if(target > currentDeg){
                    lcloseVal = abs(currentDeg + abs(360 - target));
                    //since current is less than target it needs to add on the distance target is from 360
                    rcloseVal = abs(target - currentDeg);
                    //gets the distance it is if it where to turn Right
                }else {
                    lcloseVal = abs(currentDeg - target);
                    //gets the distance it is if it where to turn Left
                    rcloseVal = abs(target + abs(360 - currentDeg));
                    //since Target is greater than current it needs to add on the distance current is from 360
                }

                //Left Turn
                if(left){
                    //create a ratio of distance left and stat distance
                    double ratio = lcloseVal/startingDis;
                    string data = to_string(lcloseVal);
                    lcd::set_text(2, data);
                    if(lcloseVal < 15 && lcloseVal > -15){
                        drive(-20, 20);
                    }
                    else if(lcloseVal < startingDis*ratio){
                        drive(-velocity*ratio, velocity*ratio);
                    }
                }

                //Right Turn
                else {
                    //create a ratio of distance left and stat distance
                    double ratio = rcloseVal/startingDis;
                    string data = to_string(lcloseVal);
                    lcd::set_text(2, data);
                    if(rcloseVal < 15 && rcloseVal > -15){
                        drive(-20, 20);
                    }
                    else if(rcloseVal < startingDis*ratio){
                        drive(velocity*ratio, -velocity*ratio);
                    }
                }

                //If within range increase count
                if (((currentDeg<=(target+5)) && (currentDeg>=(target-5)))){
                    count++;
                    if(count == 5){
                        return;
                    }
                }
                else {
                    count=0;
                    //If not reset count
                }
                
                pros::delay(10);
                // delay matches the update delay of inertial sensor so it is not using one read for multiple times
            drive(0,0);
            }

        }
};
