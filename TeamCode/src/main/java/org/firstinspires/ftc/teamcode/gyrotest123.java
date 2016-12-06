package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by 5661 on 9/21/2016.
 *
 * Full Autonomous program for the 5661 programming robot on the blue side
 *
 */
@Autonomous(name = "gyro test", group = "Autonomous OpMode")
public class gyrotest123 extends LinearOpMode{
    //declares motors, servos, and other electronics
    DcMotor motorRight;
    DcMotor motorLeft;
    CRServo beaconPoker;
    ColorSensor colorSensor;
    ColorSensor highColorSensor;
    DeviceInterfaceModule CDI;
    GyroSensor sensorGyro;
    ModernRoboticsI2cGyro mrGyro;
    //value when robot color sensor is centered on white
    int maxFindWhite = 3;
    //value when robot color sensor is half-way on white
    int minFindWhite = 2;
    //NOTE: For AndyMark NeveRest motors, 1120 is one revolution, which is 9.42inches/23.9268cm in distance
    int encoderTicks = 1120;
    double wheelCircumference = 23.9268;
    //motor and servo powers
    double powerFull = 1;
    double powerHalf = 0.5;
    double CRServoStop = 0;
    double CRServoForward = 1;
    double CRServoBackward = -1;
    double slowSpeed = 0.13;
    int shortSleep = 200;

    @Override
    public void runOpMode() throws InterruptedException {
        //connects motors, servos, and other electronics code names to config file names
        motorRight = hardwareMap.dcMotor.get("right_motor");
        motorLeft = hardwareMap.dcMotor.get("left_motor");
        //reverse right motor to drive correctly
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        beaconPoker = hardwareMap.crservo.get("beacon_poker");
        CDI = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");
        //set correct color sensor I2cAddress names
        colorSensor = hardwareMap.colorSensor.get("color");
        colorSensor.setI2cAddress(I2cAddr.create8bit(0x3c));
        highColorSensor = hardwareMap.colorSensor.get("high_color");
        highColorSensor.setI2cAddress(I2cAddr.create8bit(0x74));
        sensorGyro = hardwareMap.gyroSensor.get("gyro");
        mrGyro = (ModernRoboticsI2cGyro) sensorGyro; //allows us to get .getIntegratedZValue()

        //turns off color sensors' LEDs
        turnOffLEDs();

        //sets start position of servos
        beaconPoker.setPower(CRServoStop);

        // start calibrating the gyro.
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        mrGyro.calibrate();
        sleep(50);
        // DO NOT MOVE SENSOR WHILE BLUE LIGHT IS SOLID

        while (mrGyro.isCalibrating()){
            //Ensure calibration is complete (usually 2 seconds)
        }

        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();

        //wait for the game to start(press the play button)
        waitForStart();

        /*
         *          AUTONOMOUS MOVEMENTS BEGIN HERE
         */


        while(true){
            telemetry.addData("IntegratedZValue", mrGyro.getIntegratedZValue());
            telemetry.update();
        }

        /*
         *          AUTONOMOUS MOVEMENTS END HERE
         */

    }

    public void driveForwardDistance(double power, int distance, int sleepTime) throws InterruptedException {
        //DriveForwardDistance is used to move the robot forward a specific distance

        modeResetEncoders();

        //distance in cm divided by the wheel circumference times motor encoder ticks
        double distanceSet = ((distance/(wheelCircumference))* encoderTicks);
        //set target position
        motorLeft.setTargetPosition ((int)distanceSet);
        motorRight.setTargetPosition ((int)distanceSet);

        modeRunToPosition();

        driveForward(power);

        while(!isStopRequested() && motorLeft.isBusy() && motorRight.isBusy()){
            //wait until target position is reached
        }

        //stop and change modes back to normal
        stopDriving();
        modeRunUsingEncoder();
        sleep(sleepTime);
    }
    public void turnLeftDistance(double power, int distance, int sleepTime) throws InterruptedException {
        //TurnLeftDistance  is used to turn the robot left a specific distance

        modeResetEncoders();

        //distance in cm divided by the wheel circumference times motor encoder ticks
        double distanceSet = ((distance/(wheelCircumference))* encoderTicks);
        //set target position
        motorLeft.setTargetPosition ((int)distanceSet);
        motorRight.setTargetPosition ((int)-distanceSet);

        modeRunToPosition();

        driveForward(power);

        sleep(50);

        while(!isStopRequested() && motorLeft.isBusy() && motorRight.isBusy()){
            //wait until target position is reached
        }

        //stop and change modes back to normal
        stopDriving();
        modeRunUsingEncoder();
        sleep(sleepTime);
    }
    public void turnRightDistance(double power, int distance, int sleepTime) throws InterruptedException {
        //TurnRightDistance is used to turn the robot right a specific distance

        modeResetEncoders();

        //distance in cm divided by the wheel circumference times motor encoder ticks
        double distanceSet = ((distance/(wheelCircumference))* encoderTicks);
        //set target position
        motorLeft.setTargetPosition ((int)-distanceSet);
        motorRight.setTargetPosition ((int)distanceSet);

        modeRunToPosition();

        driveForward(power);

        sleep(50);

        while(!isStopRequested() && motorLeft.isBusy() && motorRight.isBusy()){
            //wait until target position is reached
        }

        //stop and change modes back to normal
        stopDriving();
        modeRunUsingEncoder();
        sleep(sleepTime);
    }
    public void driveForward(double power){
        //sets the motor speed to 'power'
        motorLeft.setPower(power);
        motorRight.setPower(power);
        sleep(50);
    }
    public void stopDriving(){
        //stops driving by making speed 0
        driveForward(0);
    }
    public void modeResetEncoders(){
        //reset encoders by setting to STOP_AND_RESET_ENCODER mode
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void modeRunToPosition(){
        //set to RUN_TO_POSITION mode
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void modeRunUsingEncoder(){
        //set to RUN_USING_ENCODER mode
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void modeRunWithoutEncoders(){
        //set to RUN_WITHOUT_ENCODER mode
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void findWhite() throws InterruptedException{

        //notifies driver robot is moving to find white
        telemetry.addData(">", "Moving to find white...");
        telemetry.update();

        modeRunWithoutEncoders();

        //while color sensor see no white
        while(colorSensor.alpha() < maxFindWhite){
            //Turn on LED
            colorSensor.enableLed(true);

            //notify driver robot's current color value
            telemetry.addData(">", "Moving to find white...");
            telemetry.addData("Color Sensor", colorSensor.alpha());
            telemetry.update();

            //move robot to find white line
            driveForward(0.15);
            idle();
        }

        //stopDriving();

        //while color sensor goes too far
        /*
        while(!isStopRequested() && colorSensor.alpha() < maxFindWhite){
            //Turn on LED
            colorSensor.enableLed(true);

            //notifies driver robot's current color value
            telemetry.addData("Color Sensor", colorSensor.alpha());
            telemetry.update();

            //move robot slowly backwards to find white line
            driveForward(-0.15);
            idle();
        }
        */

        /*
        if(colorSensor.alpha() > minFindWhite){
            stopDriving();
        }
        */

        //stops robot
        stopDriving();
        turnOffLEDs();
        modeRunUsingEncoder();
    }
    public void findWhiteBackwards() throws InterruptedException{

        //notifies driver robot is moving to find white
        telemetry.addData(">", "Moving backwards to find white...");
        telemetry.update();

        modeRunWithoutEncoders();

        //while color sensor see no white
        while(colorSensor.alpha() < maxFindWhite){
            //Turn on LED
            colorSensor.enableLed(true);

            //notify driver robot's current color value
            telemetry.addData(">", "Moving backwards to find white...");
            telemetry.addData("Color Sensor", colorSensor.alpha());
            telemetry.update();

            //move robot to find white line
            driveForward(-0.15);
            idle();
        }

        // stopDriving();

        /*
        //while color sensor goes too far
        while(!isStopRequested() && colorSensor.alpha() < maxFindWhite){
            //Turn on LED
            colorSensor.enableLed(true);

            //notifies driver robot's current color value
            telemetry.addData("Color Sensor", colorSensor.alpha());
            telemetry.update();

            //move robot slowly backwards to find white line
            driveForward(slowSpeed);
            idle();
        }
        */

        /*
        if(colorSensor.alpha() > minFindWhite){
            stopDriving();
        }
        */

        //stops robot
        stopDriving();
        turnOffLEDs();
        modeRunUsingEncoder();
    }
    public void findHighColor() throws InterruptedException {

        modeRunWithoutEncoders();

        while(!isStopRequested() && highColorSensor.red() == 0 && highColorSensor.blue() == 0){
            //while highColorSensor cannot read any color values, it moves the robot back

            CDI.setLED(0, false);       //Blue OFF
            CDI.setLED(1, false);       //Red OFF

            //notify driver robot's current color value
            telemetry.addData("Red", highColorSensor.red());
            telemetry.addData("Blue", highColorSensor.blue());
            //notifies driver of error status
            telemetry.addData("ERROR","sensor cannot find color values");
            telemetry.addData(">", "Moving backwards...");
            telemetry.update();
            //error, equal color value, moving back
            driveForwardDistance(0.15, -5, 250);
        }

        while(!isStopRequested() && highColorSensor.red() == highColorSensor.blue()){
            //highColorSensor reads red and blue as equal values

            CDI.setLED(0, true);       //Blue ON
            CDI.setLED(1, true);       //Red ON

            //notify driver robot's current color value
            telemetry.addData("Red", highColorSensor.red());
            telemetry.addData("Blue", highColorSensor.blue());
            //notifies driver of error status
            telemetry.addData("ERROR","colors are equal value");
            telemetry.addData(">", "Moving backwards...");
            telemetry.update();
            //error, equal color value, moving back
            driveForwardDistance(0.15, -5, 250);
        }

        stopDriving();

        if (highColorSensor.red() > highColorSensor.blue()){
            //Right side of beacon is red

            stopDriving();

            CDI.setLED(0, false);       //Blue OFF
            CDI.setLED(1, true);        //Red ON

            //notify driver robot's current color value
            telemetry.addData("Red", highColorSensor.red());
            telemetry.addData("Blue", highColorSensor.blue());
            //notifies driver of beacon status
            telemetry.addData("Right side of beacon is","RED");
            telemetry.update();

            driveForwardDistance(slowSpeed, 11, 0);
            pokeBeacon();

        } else if (highColorSensor.red() < highColorSensor.blue()){
            //Right side of beacon is blue

            stopDriving();

            CDI.setLED(0, true);        //Blue ON
            CDI.setLED(1, false);       //Red OFF

            //notify driver robot's current color value
            telemetry.addData("Red", highColorSensor.red());
            telemetry.addData("Blue", highColorSensor.blue());
            //notifies driver of beacon status
            telemetry.addData("Right side of beacon is","BLUE");
            telemetry.update();

            pokeBeacon();
        }

        //stops robot and turns off LEDs
        turnOffLEDs();
        CDI.setLED(0, false);       //Blue OFF
        CDI.setLED(1, false);       //Red OFF
        stopDriving();
        modeRunUsingEncoder();
    }
    public void findHighColorBackwards() throws InterruptedException {

        modeRunWithoutEncoders();

        while(!isStopRequested() && highColorSensor.red() == 0 && highColorSensor.blue() == 0){
            //while highColorSensor cannot read any color values, it moves the robot back

            CDI.setLED(0, false);       //Blue OFF
            CDI.setLED(1, false);       //Red OFF

            //notify driver robot's current color value
            telemetry.addData("Red", highColorSensor.red());
            telemetry.addData("Blue", highColorSensor.blue());
            //notifies driver of error status
            telemetry.addData("ERROR","sensor cannot find color values");
            telemetry.addData(">", "Moving backwards...");
            telemetry.update();
            //error, equal color value, moving back
            driveForwardDistance(0.15, -5, 250);
        }

        while(!isStopRequested() && highColorSensor.red() == highColorSensor.blue()){
            //highColorSensor reads red and blue as equal values

            CDI.setLED(0, true);       //Blue ON
            CDI.setLED(1, true);       //Red ON

            //notify driver robot's current color value
            telemetry.addData("Red", highColorSensor.red());
            telemetry.addData("Blue", highColorSensor.blue());
            //notifies driver of error status
            telemetry.addData("ERROR","colors are equal value");
            telemetry.addData(">", "Moving backwards...");
            telemetry.update();
            //error, equal color value, moving back
            driveForwardDistance(0.15, -5, 250);
        }

        stopDriving();

        if (highColorSensor.red() > highColorSensor.blue()){
            //Right side of beacon is red

            stopDriving();

            CDI.setLED(0, false);       //Blue OFF
            CDI.setLED(1, true);        //Red ON

            //notify driver robot's current color value
            telemetry.addData("Red", highColorSensor.red());
            telemetry.addData("Blue", highColorSensor.blue());
            //notifies driver of beacon status
            telemetry.addData("Right side of beacon is","RED");
            telemetry.update();

            driveForwardDistance(slowSpeed, 11, 0);
            pokeBeacon();

        } else if (highColorSensor.red() < highColorSensor.blue()){
            //Right side of beacon is blue

            stopDriving();

            CDI.setLED(0, true);        //Blue ON
            CDI.setLED(1, false);       //Red OFF

            //notify driver robot's current color value
            telemetry.addData("Red", highColorSensor.red());
            telemetry.addData("Blue", highColorSensor.blue());
            //notifies driver of beacon status
            telemetry.addData("Right side of beacon is","BLUE");
            telemetry.update();

            pokeBeacon();
        }

        //stops robot and turns off LEDs
        turnOffLEDs();
        CDI.setLED(0, false);       //Blue OFF
        CDI.setLED(1, false);       //Red OFF
        stopDriving();
        modeRunUsingEncoder();
    }
    public void turnOffLEDs(){
        //turns off color sensor LEDs
        colorSensor.enableLed(false);
        highColorSensor.enableLed(false);
    }
    public void pokeBeacon(){
        //Moves CRServo to hit beacon and move back
        beaconPoker.setPower(CRServoForward);
        sleep(2000);
        beaconPoker.setPower(CRServoBackward);
        sleep(1500);
        beaconPoker.setPower(CRServoStop);
        sleep(50);
    }
    public void checkGyro(int target) throws InterruptedException {
        //turnAbsoluteGyro turns robot from initial calibration, you have to know how much to turn according from your starting point

        telemetry.addData(">", "Turning robot using gyro");
        telemetry.update();

        while(Math.abs(mrGyro.getIntegratedZValue() - target) > 3) {
            if (mrGyro.getIntegratedZValue() > target) { //if gyro is positive,  the robot will turn right
                telemetry.addData(">", "Robot is currently turning right");
                telemetry.addData("IntegratedZValue", mrGyro.getIntegratedZValue());
                telemetry.update();

                //motorLeft.setPower(0.12);
                //motorRight.setPower(-0.12);
                turnRightDistance(0.1, 1, 250);
                idle();
            }

            if (mrGyro.getIntegratedZValue() < target) { //if gyro is negative, the robot will turn left
                telemetry.addData(">", "Robot is currently turning left");
                telemetry.addData("IntegratedZValue", mrGyro.getIntegratedZValue());
                telemetry.update();

                //motorLeft.setPower(-0.12);
                //motorRight.setPower(0.12);
                turnLeftDistance(0.1, 1, 250);
                idle();
            }
        }
        stopDriving();
    }
}