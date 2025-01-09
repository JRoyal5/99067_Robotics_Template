#include "main.h"
using namespace std;
using namespace pros;

class Robot{
    private:
        double driveBaseRadius = 0;
        double gearRatio = 0;
        double wheelDiameter = 0;
        double tickPerRev = 0;
    public:
        //Drive Motors
        MotorGroup leftMotors;
        MotorGroup rightMotors;
        IMU imu_sensor;
                
        //Constructor
        Robot(std::initializer_list<std::int8_t> leftMotors, std::initializer_list<std::int8_t> rightMotors, int imu_sensor) 
        : leftMotors(leftMotors), rightMotors(rightMotors), imu_sensor(imu_sensor) {
            
        }

        Robot(std::initializer_list<std::int8_t> leftMotors, std::initializer_list<std::int8_t> rightMotors, int imu_sensor, std::initializer_list<std::double_t> constants) 
        : leftMotors(leftMotors), rightMotors(rightMotors), imu_sensor(imu_sensor) {
            vector<double> constant = constants;
            driveBaseRadius = constant[0];
            gearRatio = constant[1];
            wheelDiameter = constant[2];
            tickPerRev = constant[3];
        }

        //Methods

        //Driving methods
        // @param Values range from -127 to 127
        void drive(double left, double right){
            leftMotors.move(left);
            rightMotors.move(right);
        }

        void driveFront(double inches, double power){
            leftMotors.set_zero_position(0);
            rightMotors.set_zero_position(0);

            
            vector<double> positionsL = leftMotors.get_positions();
            vector<double> positionsR = rightMotors.get_positions();
            int counter = 1;
            double currentDis = ((positionsL[0] + positionsR[0])/2);
            double currentDis2 = 0;
            double c = 0;
            int currentPower = (int) power;
            double tickPerInch = tickPerRev/(6*3.1415*wheelDiameter*gearRatio);
            double targetDis = (inches*tickPerInch);

            while(!(currentDis <= (targetDis+(tickPerInch/4)) && currentDis >= (targetDis-(tickPerInch/4))) || !(counter == 5)){
                positionsL = leftMotors.get_positions();
                positionsR = rightMotors.get_positions();
                currentDis = (positionsL[0] + positionsR[0])/2;
                if(currentDis>targetDis){
                    currentPower = -power;
                    drive((currentPower),(currentPower));
                } else if (currentDis < targetDis/2){
                    drive(power,power);
                } else if(currentDis<(((targetDis/2)*c)+(targetDis/2))){
                    c = ((currentDis-(targetDis/2))/(targetDis/2));
                    currentPower = power*(1-(c*c));
                    drive(currentPower,currentPower);
                } else {
                    c = ((currentDis+(targetDis/2))/(targetDis/2));
                    drive(currentPower,currentPower);
                }

                positionsL = leftMotors.get_positions();
                positionsR = rightMotors.get_positions();
                currentDis2 = (positionsL[0] + positionsR[0])/2;
                if ((currentDis2 <= (targetDis + (tickPerInch/4))) && (currentDis2 >= (targetDis - (tickPerInch/4)))){
                    counter++;
                    if(counter == 5){
                        drive(-power,-power);
                        pros::delay(50);
                        drive(0,0);
                        break;
                    }
                } else {
                    counter = 0;
                }
            }
            drive(0,0);
        }

        void driveBack(double inches, double power){
            //reset motor encoders
            leftMotors.set_zero_position(0);
            rightMotors.set_zero_position(0);

            vector<double> positionsL = leftMotors.get_positions();
            vector<double> positionsR = rightMotors.get_positions();
            int counter = 1;
            double currentDis = ((positionsL[0] + positionsR[0])/2);
            double currentDis2 = 0;
            double c = 0;
            double currentPower = power;
            double tickPerInch = tickPerRev/(6*3.1415*wheelDiameter*gearRatio);
            double targetDis = -(inches*tickPerInch);

            while(!(currentDis <= (targetDis-(tickPerInch/4)) && currentDis >= (targetDis+(tickPerInch/4))) || !(counter == 5)){
                positionsL = leftMotors.get_positions();
                positionsR = rightMotors.get_positions();
                currentDis = (positionsL[0] + positionsR[0])/2;
                if(currentDis < targetDis){
                    currentPower = power;
                    drive((currentPower),(currentPower));
                } else if (currentDis > targetDis/2){
                    drive(-power,-power);
                } else if(currentDis > (((targetDis/2)*c)+(targetDis/2))){
                    c = ((currentDis-(targetDis/2))/(targetDis/2));
                    currentPower = power*(1-(c*c));
                    drive(-currentPower,-currentPower);
                } else {
                    c = ((currentDis+(targetDis/2))/(targetDis/2));
                    drive(-currentPower,-currentPower);
                }

                currentDis2 = (positionsL[0] + positionsR[0])/2;
                if ((currentDis2 <= (targetDis + (tickPerInch/4))) && (currentDis2 >= (targetDis - (tickPerInch/4)))){
                    counter++;
                    if(counter == 5){
                        drive(power,power);
                        pros::delay(50);
                        drive(0,0);
                        break;
                    }
                } else {
                    counter = 0;
                }
            }
            drive(0,0);
        }

        //Turn method
        //Requires inertial sensor
        void turn(int target, int Mpower){
   
            //Initializes everything
            int stuckCount=0;
            int startingDis = 0;
            int currentDeg= imu_sensor.get_heading();
            int lcloseVal =0;
            int rcloseVal=0;
            int count=0;
            int currentPwr = 0;
            float lastPosition=0;
            bool moving = false;

            if(currentDeg > 0){
                if(currentDeg >= 360){
                    while(currentDeg >= 360){
                        currentDeg = currentDeg - 360;
                    }
                }
            } else {
                while(currentDeg <= -360){
                        currentDeg = currentDeg + 360;
                    }
                currentDeg = 360 + currentDeg;
            }

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
            if (startingDis < 45){
                startingDis = 45;
            }
            //sets the starting distance based on which ever is shortest

            while((!(((currentDeg<=(target+5))) && (currentDeg>=(target-5))))){
                //Gives a little room for error so doesn't fight forever
                //Count so it dosent exit on firt read
                currentDeg= imu_sensor.get_heading();
                std::cout<<(currentDeg)<<std::endl;

                //Updates the current heading between 0 and 360
                if(currentDeg > 0){
                    if(currentDeg >= 360){
                        while(currentDeg >= 360){
                            currentDeg = currentDeg - 360;
                        }
                    }
                } else {
                    while(currentDeg <= -360){
                        currentDeg = currentDeg + 360;
                    }
                    currentDeg = 360 + currentDeg;
                }

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

                    //if the distance is less than a 8th of the starting distance
                    if(lcloseVal<=(startingDis/8)){
                        if(lcloseVal<startingDis/8){
                            if(currentPwr>10){
                                currentPwr-=4;
                            } else {
                                currentPwr=10;
                            }
                        }
                        drive(-1*currentPwr,currentPwr);
                    }

                    //if the distance is less than a 3rd of the starting distance
                    else if(lcloseVal<=(startingDis/2)){
                        if(lcloseVal<startingDis/2){
                            if(currentPwr>(Mpower/3)){
                                currentPwr-=4;
                            } else {
                                currentPwr=(Mpower/3);
                            }
                        }
                        drive(-1*currentPwr,currentPwr);
                    }

                    //if the distance is less than half of the starting distance
                    else if(lcloseVal<=(startingDis*3/4)){
                        if(lcloseVal<startingDis*3/4){
                            if(currentPwr>(Mpower/4)*3){
                                currentPwr-=4;
                            } else {
                                currentPwr=(Mpower/4)*3;
                            }
                        }
                        drive(-1*currentPwr,currentPwr);
                    }
                
                    else{
                        drive(-1*Mpower,Mpower);
                    }

                    if(lcloseVal>startingDis/2){
                        if(currentPwr<Mpower){
                            currentPwr+=10;
                        } else {
                            currentPwr=Mpower;
                        }
                    }
                    //broken up so it will slow down as it gets closer and also not do rapid turn back
                }

                //Right Turn
                else {

                    //if the distance less is less than a 8th of the starting distance
                    if(rcloseVal<=(startingDis/8)){
                        if(rcloseVal<startingDis/8){
                            if(currentPwr>10){
                                currentPwr-=4;
                            } else {
                                currentPwr=10;
                            }
                        }
                        drive(currentPwr,-1*currentPwr);
                    }
                
                    //if the distance less is less than a 3rd of the starting distance
                    else if(rcloseVal<=(startingDis/3)){
                        if(rcloseVal<startingDis/3){
                            if(currentPwr>(Mpower/3)){
                                currentPwr-=4;
                            } else {
                                currentPwr=(Mpower/3);
                            }
                        }
                        drive(currentPwr,-1*currentPwr);
                    }
                
                    //if the distance less is less than a half of the starting distance
                    else if(rcloseVal<=(startingDis*3/4)){
                        if(rcloseVal<startingDis*3/4){
                            if(currentPwr>(Mpower/4)*3){
                                currentPwr-=4;
                            } else {
                                currentPwr=(Mpower/4)*3;
                            }
                        }
                        drive(currentPwr,-1*currentPwr);
                    }
                
                    else{
                        drive(Mpower,-1*Mpower);
                    }

                    if(rcloseVal>startingDis/2){
                        if(currentPwr<Mpower){
                            currentPwr+=10;
                        } else {
                            currentPwr=Mpower;
                        }
                    }
                    //broken up so it will slow down as it gets closer and also not do rapid turn back
                }

                //If within range increase count
                if (((currentDeg<=(target+5)) && (currentDeg>=(target-5)))){
                    count++;
                    if(count == 5){
                        break;
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

        /*Arcturn method
        * Requires driveBaseRadius, gearRatio, wheelDiameter, and tickPerRev to be initialized
        * Requires inertial sensor
        */
        void arcFront(double radius, double target, double power){
            leftMotors.set_zero_position(0);
            rightMotors.set_zero_position(0);

            bool left;

            //Funtion doubles
            vector<double> positionsL = leftMotors.get_positions();
            vector<double> positionsR = rightMotors.get_positions();
            double pi = 2 * acos(0);
            double targetRads = target * (pi/180);
            double modr = 0;
            double modl = 0;
            double powerRatio = radius/(radius+(2*driveBaseRadius));
            double c = 0;
            double tickPerInch = tickPerRev/(6 * pi * wheelDiameter * gearRatio);

            //Function Integers
            int maxRightSpeed = 0;
            int currentRightPower = 0;
            int arcLengthRight = 0;
            int maxLeftSpeed = 0;
            int currentLeftPower = 0;
            int arcLengthLeft = 0;
            int count = 0;
            int initialDeg = imu_sensor.get_heading();
            int currentDeg = imu_sensor.get_heading();
            int lDegDis;
            int rDegDis;
            int degDis;
            int lAdjust;
            int rAdjust;


            if(currentDeg > 0){
                if(currentDeg >= 360){
                    while(currentDeg >= 360){
                        currentDeg = currentDeg - 360;
                    }
                }
            } else {
                while(currentDeg <= -360){
                        currentDeg = currentDeg + 360;
                    }
                currentDeg = 360 + currentDeg;
            }

            //Determins what direction to turn
            if(target > currentDeg){
                lDegDis = abs(currentDeg + abs(360 - target));
                //since current is less than target it needs to add on the distance target is from 360
                rDegDis = abs(target - currentDeg);
                //gets the distance it is if it where to turn Right
            }else {
                lDegDis = abs(currentDeg - target);
                //gets the distance it is if it where to turn Left
                rDegDis = abs(target + abs(360 - currentDeg));
                //since Target is greater than current it needs to add on the distance current is from 360
            }

            if(lDegDis < rDegDis){
                degDis = lDegDis;

                arcLengthLeft = radius * targetRads * tickPerInch;
                arcLengthRight = (radius + (2 * driveBaseRadius)) * targetRads * tickPerInch;

                maxLeftSpeed = power * powerRatio;
                maxRightSpeed = power;

                left = true;
            } else{
                degDis = rDegDis;

                arcLengthLeft = (radius + (2 * driveBaseRadius)) * targetRads * tickPerInch;
                arcLengthRight = radius * targetRads * tickPerInch;

                maxLeftSpeed = power;
                maxRightSpeed = power * powerRatio;

                left = false;
            }

            while(!((currentDeg <= (target + 5) && currentDeg >= (target - 5)))){
                positionsL = leftMotors.get_positions();
                positionsR = rightMotors.get_positions();
                currentDeg = imu_sensor.get_heading();

                //Determins what direction to turn
                if(target > currentDeg){
                    lDegDis = abs(currentDeg + abs(360 - target));
                    //since current is less than target it needs to add on the distance target is from 360
                    rDegDis = abs(target - currentDeg);
                    //gets the distance it is if it where to turn Right
                }else {
                    lDegDis = abs(currentDeg - target);
                    //gets the distance it is if it where to turn Left
                    rDegDis = abs(target + abs(360 - currentDeg));
                    //since Target is greater than current it needs to add on the distance current is from 360
                }

                //Left turn
                if(left){
                    if(lDegDis < degDis/2){
                        drive(maxLeftSpeed, maxRightSpeed);
                    } else if(lDegDis < (((degDis/2)*c)+(degDis/2))){
                        c = ((lDegDis - (degDis/2))/(degDis/2));
                        modr = maxRightSpeed*(1-(c*c));
                        modl = maxLeftSpeed*(1-(c*c));
                        currentLeftPower = maxLeftSpeed * modl;
                        currentRightPower = maxRightSpeed * modr;
                        drive(currentLeftPower + lAdjust, currentRightPower + rAdjust);
                    } else {
                        c = ((lDegDis - (degDis/2))/(degDis/2));
                        drive(currentLeftPower + lAdjust, currentRightPower + rAdjust);
                    }

                    double currentArcL = (positionsL[0] + positionsL[1])/2;
                    double currentArcR = (positionsR[0] + positionsR[1])/2;
                    double changeInDeg = abs((initialDeg + 360) - currentDeg);
                    if(changeInDeg > 360){
                        changeInDeg -= 360;
                    }
                    // if ((currentArcL < (radius * ((changeInDeg*pi)/180) * tickPerInch)) || currentArcR > (radius + (2 * driveBaseRadius)) * ((changeInDeg*pi)/180) * tickPerInch){
                    //     lAdjust += currentLeftPower/10;
                    // } else if((currentArcL > (radius * ((changeInDeg*pi)/180) * tickPerInch)) || currentArcR < (radius + (2 * driveBaseRadius)) * ((changeInDeg*pi)/180) * tickPerInch) {
                    //     lAdjust -= currentLeftPower/10;
                    // } else{
                    //     lAdjust = 0;
                    // }
                }
                //Right Turn
                else{
                    if(rDegDis > degDis/2){
                        drive(maxLeftSpeed, maxRightSpeed);
                    } else if(rDegDis > (((degDis/2)*c)+(degDis/2))){
                        c = ((rDegDis - (degDis/2))/(degDis/2));
                        modr = maxRightSpeed*(1-(c*c));
                        modl = maxLeftSpeed*(1-(c*c));
                        currentLeftPower = maxLeftSpeed * modl;
                        currentRightPower = maxRightSpeed * modr;
                        drive(currentLeftPower + lAdjust, currentRightPower + rAdjust);
                    } else {
                        c = ((rDegDis - (degDis/2))/(degDis/2));
                        drive(currentLeftPower + lAdjust, currentRightPower + rAdjust);
                    }

                    double currentArcL = (positionsL[0] + positionsL[1])/2;
                    double currentArcR = (positionsR[0] + positionsR[1])/2;
                    double changeInDeg = abs((initialDeg - 360) - currentDeg);
                    if(changeInDeg < -360){
                        changeInDeg += 360;
                    }
                    // if ((currentArcL > (radius * ((changeInDeg*pi)/180) * tickPerInch)) || currentArcR < (radius + (2 * driveBaseRadius)) * ((changeInDeg*pi)/180) * tickPerInch){
                    //     rAdjust += currentRightPower/10;
                    // } else if((currentArcL < (radius * ((changeInDeg*pi)/180) * tickPerInch)) || currentArcR > (radius + (2 * driveBaseRadius)) * ((changeInDeg*pi)/180) * tickPerInch) {
                    //     rAdjust -= currentRightPower/10;
                    // } else{
                    //     rAdjust = 0;
                    // }
                }
                    double currentArcL = (positionsL[0] + positionsL[1])/2;
                    double currentArcR = (positionsR[0] + positionsR[1])/2;

                    if ((currentDeg <= (target + 5) && currentDeg >= (target - 5))){
                        count++;
                        if(count == 5){
                            drive(-power,-power);
                            pros::delay(50);
                            drive(0,0);
                            break;
                        }
                    } else {
                        count = 0;
                    }
                    
                    pros::delay(10);
                    // delay matches the update delay of inertial sensor so it is not using one read for multiple times
                drive(0,0);
            }
        }

        void arcBack(double radius, double target, double power){
            //Funtion doubles
            double pi = 2*acos(0);

            //Function Floating Point Numbers
            float lTurnVal = 0;
            float rTurnVal = 0;
            float targetRads = target * (pi/180);
            float currentDeg = imu_sensor.get_heading();

            //Function Integers
            int startDis = 0;
            int maxRightSpeed = 0;
            int maxLeftSpeed = 0;
            int currentLeftPower = 0;
            int currentRightPower = 0;
            int arcLengthLeft = 0;
            int arcLengthRight = 0;
            int powerRatio = 0;
            int maxOppPower = 0;
            int c = 0;
            int mod = 0;

            //Function Booleans
            bool left = false;

            //Function Constants
            const double robotWidth = 14;

            if(currentDeg > 0){
                while(currentDeg >= 360){
                currentDeg = currentDeg - 360;
                }
            } else{
                while(currentDeg <= -360){
                currentDeg = 360 + currentDeg;
                }
            }

            if(target > currentDeg){
            lTurnVal = abs(currentDeg + abs(360-target));
            rTurnVal = abs(target - currentDeg);
            } else{
            rTurnVal = abs(currentDeg + abs(360-target));
            lTurnVal = abs(target - currentDeg);
            }

            if(lTurnVal < rTurnVal){ //Left Turn
                left = true;
                startDis = abs(lTurnVal);
                arcLengthLeft = radius * targetRads;
                arcLengthRight = (radius + robotWidth) * targetRads;

                powerRatio = arcLengthRight/arcLengthLeft;

                maxLeftSpeed = power * powerRatio;
                maxRightSpeed = power;
            } else{ //Right Turn
                startDis = abs(rTurnVal);
                left = false;
                arcLengthLeft = (radius + robotWidth) * targetRads;
                arcLengthRight = radius * targetRads;

                powerRatio = arcLengthLeft/arcLengthRight;

                maxLeftSpeed = power;
                maxRightSpeed = power * powerRatio;
            }

            while(!(((currentDeg<=(target+3))) && (currentDeg>=(target-3)))){
                currentDeg = (imu_sensor.get_heading()/*+imu_sensor_2.get_heading()*/)/2;

                if(currentDeg > 0){
                    while(currentDeg >= 360){
                        currentDeg = currentDeg - 360;
                    }
                } else{
                    while(currentDeg <= -360){
                        currentDeg = 360 + currentDeg;
                    }
                }

                if(target > currentDeg){
                    lTurnVal = fabs(currentDeg + abs(360-target));
                    rTurnVal = fabs(target - currentDeg);
                } else{
                    rTurnVal = fabs(currentDeg + abs(360-target));
                    lTurnVal = fabs(target - currentDeg);
                }

                //Left turn
                if(left){
                    if(currentDeg > startDis){
                        // drive(-maxLeftSpeed,-maxRightSpeed);
                    } else if(currentDeg < startDis/2){
                        drive(-maxLeftSpeed, -maxRightSpeed);
                    } else if(currentDeg<(((startDis/2)*c)+(startDis/2))){
                        c = ((currentDeg-(startDis/2))/(startDis/2));
                        mod = power*(1-(c*c));
                        currentLeftPower = maxLeftSpeed * mod;
                        currentRightPower = maxRightSpeed * mod;
                        drive(-currentLeftPower, -currentRightPower);
                    } else {
                        c = ((currentDeg-(startDis/2))/(startDis/2));
                        drive(-currentLeftPower, -currentRightPower);
                    }
                } else{
                    if(currentDeg > startDis){
                        // drive(-maxLeftSpeed,-maxRightSpeed);
                    } else if(currentDeg < startDis/2){
                        drive(-maxLeftSpeed, -maxRightSpeed);
                    } else if(currentDeg<(((startDis/2)*c)+(startDis/2))){
                        c = ((currentDeg-(startDis/2))/(startDis/2));
                        mod = power*(1-(c*c));
                        currentLeftPower = maxLeftSpeed * mod;
                        currentRightPower = maxRightSpeed * mod;
                        drive(-currentLeftPower, -currentRightPower);
                    } else {
                        c = ((currentDeg-(startDis/2))/(startDis/2));
                        drive(-currentLeftPower, -currentRightPower);
                    }
                }
            }
        }

};
