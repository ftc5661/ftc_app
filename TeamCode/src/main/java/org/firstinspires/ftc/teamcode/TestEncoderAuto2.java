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

/**
 * Created by 5661 on 11/10/2016.
 * Full Autonomous program for the 5661 programming robot on the red side
 */
@Autonomous(name = "Autonomous Red Side", group = "Autonomous OpMode")
public class TestEncoderAuto2 extends LinearOpMode {
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
    int maxFindWhite = 2;
    //value when robot color sensor is half-way on white
    int minFindWhite = 1;
    //NOTE: For AndyMark NeveRest motors, 1120 is one revolution, which is 9.42inches/23.9268cm in distance
    int encoderTicks = 1120;
    double wheelCircumference = 23.9268;
    //motor and servo powers
    double powerFull = 1;
    double powerHalf = 0.5;
    double CRServoStop = 0;
    double CRServoForward = 1;
    double CRServoBackward = -1;
    int turnRight = -1;
    int turnLeft = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        //connects motors, servos, and other electronics code names to config file names
        motorLeft = hardwareMap.dcMotor.get("right_motor");
        motorRight = hardwareMap.dcMotor.get("left_motor");
        //reverse right motor to drive correctly
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        beaconPoker = hardwareMap.crservo.get("beacon_poker");
        CDI = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");
        sensorGyro = hardwareMap.gyroSensor.get("gyro");
        mrGyro = (ModernRoboticsI2cGyro) sensorGyro; //allows us to get .getIntegratedZValue()
        //set correct color sensor I2cAddress names
        colorSensor = hardwareMap.colorSensor.get("color");
        colorSensor.setI2cAddress(I2cAddr.create8bit(0x3c));
        highColorSensor = hardwareMap.colorSensor.get("high_color");
        highColorSensor.setI2cAddress(I2cAddr.create8bit(0x74));

        //turns off color sensors' LEDs
        turnOffLEDs();

        //sets start position of servos
        beaconPoker.setPower(CRServoStop);

        // start calibrating the gyro.
        mrGyro.calibrate();
        sleep(50);
        idle();
        // DO NOT MOVE SENSOR WHILE BLUE LIGHT IS SOLID

        while (!isStopRequested() && mrGyro.isCalibrating()){
            //Ensure calibration is complete (usually 2 seconds)
            telemetry.addData(">", "Gyro Calibrating. Do Not move!");
            telemetry.update();
            sleep(50);
        }

        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();

        //wait for the game to start(press the play button)
        waitForStart();

        /*
         *          AUTONOMOUS MOVEMENTS BEGIN HERE
         */

        driveForwardDistance(0.8, 100); //speed then distance
        turnLeftDistance(0.15, 15);
        //turnAbsoluteGyro(48, turnRight);
        driveForwardDistance(0.8, 125);
        turnRightDistance(0.15, 12);
        //turnGyro(35, turnLeft);
        driveForwardDistance(0.3, 30);
        turnRightDistance(0.15, 6);
        //turnGyro(14, turnLeft);
        driveForwardDistance(0.2, 40);
        findWhite();
        findHighColor();
        driveForwardDistance(0.3, -95);
        findWhiteBackwards();
        findHighColor();
        stopDriving();

        /*
         *          AUTONOMOUS MOVEMENTS END HERE
         */

        turnOffLEDs();
        //update telemetry logs
        telemetry.addData(">", "Path Complete");
        telemetry.update();
        sleep(500);
    }
    public void driveForwardDistance(double power, int distance) throws InterruptedException {
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
    }

    public void turnLeftDistance(double power, int distance) throws InterruptedException {
        //TurnLeftDistance  is used to turn the robot left a specific distance

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
    }
    public void turnRightDistance(double power, int distance) throws InterruptedException {
        //TurnRightDistance is used to turn the robot right a specific distance

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
    public void turnAbsoluteGyro(int target, int direction){
        //turnAbsoluteGyro turns robot from initial calibration, you have to know how much to turn according from your starting point
        telemetry.addData(">", "Turning robot using gyro");
        telemetry.update();

        int fixedTarget = 0;

        if (direction < 0 ){
            //turn right
            fixedTarget = -target;
        } else if (direction > 0){
            //turn left
            fixedTarget = target;
        }

        while(!isStopRequested() && Math.abs(mrGyro.getIntegratedZValue() - fixedTarget) > 3){

            if (mrGyro.getIntegratedZValue() > fixedTarget){ //if gyro is positive,  the robot will turn right
                telemetry.addData(">", "Robot is currently turning right");
                telemetry.addData("IntegratedZValue", mrGyro.getIntegratedZValue());
                telemetry.update();

                motorLeft.setPower(0.1);
                motorRight.setPower(-0.1);
            }

            if (mrGyro.getIntegratedZValue() < fixedTarget){ //if gyro is negative, the robot will turn left
                telemetry.addData(">", "Robot is currently turning left");
                telemetry.addData("IntegratedZValue", mrGyro.getIntegratedZValue());
                telemetry.update();

                motorLeft.setPower(-0.1);
                motorRight.setPower(0.1);
            }
            idle();
        }
        //stops motors
        motorLeft.setPower(0);
        motorRight.setPower(0);
    }
    public void turnGyro(int target, int direction){ //counter-clockwise
        //turnGyro turns robot from current position. careful when using this method, small errors in movement can add up!
        turnAbsoluteGyro(target + mrGyro.getIntegratedZValue(), direction);
    }
    public void findWhite() throws InterruptedException{

        //notifies driver robot is moving to find white
        telemetry.addData(">", "Moving to find white...");
        telemetry.update();

        modeRunWithoutEncoders();

        //while color sensor see no white
        while(!isStopRequested() && colorSensor.alpha() < maxFindWhite){
            //Turn on LED
            colorSensor.enableLed(true);

            //notify driver robot's current color value
            telemetry.addData("Color Sensor", colorSensor.alpha());
            telemetry.update();

            //move robot to find white line
            driveForward(0.07);
            idle();
        }

        stopDriving();

        //while color sensor goes too far
        while(!isStopRequested() && colorSensor.alpha() < maxFindWhite){
            //Turn on LED
            colorSensor.enableLed(true);

            //notifies driver robot's current color value
            telemetry.addData("Color Sensor", colorSensor.alpha());
            telemetry.update();

            //move robot slowly backwards to find white line
            driveForward(-0.07);
            idle();
        }

        if(colorSensor.alpha() > minFindWhite){
            stopDriving();
        }

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
        while(!isStopRequested() && colorSensor.alpha() < maxFindWhite){
            //Turn on LED
            colorSensor.enableLed(true);

            //notify driver robot's current color value
            telemetry.addData("Color Sensor", colorSensor.alpha());
            telemetry.update();

            //move robot to find white line
            driveForward(-0.07);
            idle();
        }

        stopDriving();

        //while color sensor goes too far
        while(!isStopRequested() && colorSensor.alpha() < maxFindWhite){
            //Turn on LED
            colorSensor.enableLed(true);

            //notifies driver robot's current color value
            telemetry.addData("Color Sensor", colorSensor.alpha());
            telemetry.update();

            //move robot slowly backwards to find white line
            driveForward(0.07);
            idle();
        }

        if(colorSensor.alpha() > minFindWhite){
            stopDriving();
        }

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
            driveForward(-0.1);
            idle();
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
            driveForward(-0.1);
            idle();
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
            telemetry.addData("Left side of beacon is","RED");
            telemetry.update();

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
            telemetry.addData("Left side of beacon is","BLUE");
            telemetry.update();

            driveForwardDistance(0.1, 13);
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
        sleep(3000);
        beaconPoker.setPower(CRServoBackward);
        sleep(1700);
        beaconPoker.setPower(CRServoStop);
        sleep(50);
    }
}
