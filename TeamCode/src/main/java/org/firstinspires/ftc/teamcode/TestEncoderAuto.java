package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by 5661 on 9/21/2016.
 * Full Autonomous program for the 5661 programming robot
 */
@Autonomous(name = "TestEncoderAuto", group = "Autonomous OpMode")
public class TestEncoderAuto extends LinearOpMode{

    //declares motors, servos, and other electronics
    DcMotor motorRight;
    DcMotor motorLeft;
    CRServo beaconPoker;
    ColorSensor colorSensor;
    ColorSensor highColorSensor;
    DeviceInterfaceModule CDI;
    //value when robot color sensor is centered on white
    public int maxFindWhite = 2;
    //value when robot color sensor is half-way on white
    public int minFindWhite = 1;
    //NOTE: For AndyMark NeveRest motors, 1120 is one revolution, which is 9.42inches/23.9268cm in distance
    int encoderTicks = 1120;
    double wheelCircumference = 23.9268;
    //motor and servo powers
    double powerFull = 1;
    double powerHalf = 0.5;
    double powerThreeTenths = 0.3;
    double powerEightTenths = 0.8;
    double CRServoStop = 0;
    double CRServoForward = 1;
    double CRServoBackward = -1;




    @Override
    public void runOpMode() throws InterruptedException {

        //connects motors, servos, and other electronics code names to config file names
        motorRight = hardwareMap.dcMotor.get("right_motor");
        motorLeft = hardwareMap.dcMotor.get("left_motor");
        //reverse right motor to drive correctly
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        beaconPoker = hardwareMap.crservo.get("beacon_poker");
        CDI = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");
        //set correct color sensor I2cAddress names
        colorSensor = hardwareMap.colorSensor.get("color");
        colorSensor.setI2cAddress(I2cAddr.create8bit(0x3c));
        highColorSensor = hardwareMap.colorSensor.get("high_color");
        highColorSensor.setI2cAddress(I2cAddr.create8bit(0x74));

        TurnOffLEDs();
        //sets start position of servos
        beaconPoker.setPower(CRServoStop);

        //wait for the game to start(press the play button)
        waitForStart();

        /*
         *          AUTONOMOUS MOVEMENTS BEGIN HERE
         */

        DriveForwardDistance(powerEightTenths, 100);
        TurnRightDistance(powerThreeTenths, 15);
        DriveForwardDistance(powerEightTenths, 120);
        TurnLeftDistance(powerThreeTenths, 12);
        DriveForwardDistance(powerThreeTenths, 30);
        TurnLeftDistance(powerThreeTenths, 6);
        findWhite();
        findHighColor();
        DriveForwardDistance(powerThreeTenths, -90);
        findWhiteBackwards();
        findHighColor();
        StopDriving();

        /*
         *          AUTONOMOUS MOVEMENTS END HERE
         */

        TurnOffLEDs();
        //update telemetry logs
        telemetry.addData(">", "Path Complete");
        telemetry.update();
        sleep(1000);


        }

    public void DriveForwardDistance(double power, int distance) throws InterruptedException {
        //DriveForwardDistance is used to move the robot forward a specific distance

        ModeResetEncoders();

        //distance in cm divided by the wheel circumference times motor encoder ticks
        double distanceSet = ((distance/(wheelCircumference))* encoderTicks);
        //set target position
        motorLeft.setTargetPosition ((int)distanceSet);
        motorRight.setTargetPosition ((int)distanceSet);

        ModeRunToPosition();

        DriveForward(power);

        while(!isStopRequested() && motorLeft.isBusy() && motorRight.isBusy()){
            //wait until target position is reached
        }

        //stop and change modes back to normal
        StopDriving();
        ModeRunUsingEncoder();
        idle();
    }

    public void TurnLeftDistance(double power, int distance) throws InterruptedException {
        //TurnLeftDistance  is used to turn the robot left a specific distance

        ModeResetEncoders();

        //distance in cm divided by the wheel circumference times motor encoder ticks
        double distanceSet = ((distance/(wheelCircumference))* encoderTicks);
        //set target position
        motorLeft.setTargetPosition ((int)-distanceSet);
        motorRight.setTargetPosition ((int)distanceSet);

        ModeRunToPosition();

        DriveForward(power);

        while(!isStopRequested() && motorLeft.isBusy() && motorRight.isBusy()){
            //wait until target position is reached
        }

        //stop and change modes back to normal
        StopDriving();
        ModeRunUsingEncoder();
        idle();
    }
    public void TurnRightDistance(double power, int distance) throws InterruptedException {
        //TurnRightDistance is used to turn the robot right a specific distance

        ModeResetEncoders();

        //distance in cm divided by the wheel circumference times motor encoder ticks
        double distanceSet = ((distance/(wheelCircumference))* encoderTicks);
        //set target position
        motorLeft.setTargetPosition ((int)distanceSet);
        motorRight.setTargetPosition ((int)-distanceSet);

        ModeRunToPosition();

        DriveForward(power);

        while(!isStopRequested() && motorLeft.isBusy() && motorRight.isBusy()){
            //wait until target position is reached
        }

        //stop and change modes back to normal
        StopDriving();
        ModeRunUsingEncoder();
        idle();
    }

    public void DriveForward(double power) throws InterruptedException {
        //sets the motor speed to 'power'
        motorLeft.setPower(power);
        motorRight.setPower(power);
        sleep(50);
    }

    public void StopDriving() throws InterruptedException {
        //stops driving by making speed 0
        DriveForward(0);
    }

    public void ModeResetEncoders(){
        //reset encoders by setting to STOP_AND_RESET_ENCODER mode
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void ModeRunToPosition(){
        //set to RUN_TO_POSITION mode
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void ModeRunUsingEncoder(){
        //set to RUN_USING_ENCODER mode
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void ModeRunWithoutEncoders(){
        //set to RUN_WITHOUT_ENCODER mode
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void findWhite() throws InterruptedException{

        //notifies driver robot is moving to find white
        telemetry.addData(">", "Moving to find white...");
        telemetry.update();

        ModeRunWithoutEncoders();

        //while color sensor see no white
        while(!isStopRequested() && colorSensor.alpha() < maxFindWhite){
            //Turn on LED
            colorSensor.enableLed(true);

            //notify driver robot's current color value
            telemetry.addData("Color Sensor", colorSensor.alpha());
            telemetry.update();

            //move robot to find white line
            DriveForward(0.07);
            idle();
        }

        StopDriving();

        //while color sensor goes too far
        while(!isStopRequested() && colorSensor.alpha() < maxFindWhite){
            //Turn on LED
            colorSensor.enableLed(true);

            //notifies driver robot's current color value
            telemetry.addData("Color Sensor", colorSensor.alpha());
            telemetry.update();

            //move robot slowly backwards to find white line
            DriveForward(-0.07);
            idle();
        }

        if(colorSensor.alpha() > minFindWhite){
            StopDriving();
        }

        //stops robot
        StopDriving();
        TurnOffLEDs();
        ModeRunUsingEncoder();
        idle();

    }

    public void findWhiteBackwards() throws InterruptedException{

        //notifies driver robot is moving to find white
        telemetry.addData(">", "Moving backwards to find white...");
        telemetry.update();

        ModeRunWithoutEncoders();

        //while color sensor see no white
        while(!isStopRequested() && colorSensor.alpha() < maxFindWhite){
            //Turn on LED
            colorSensor.enableLed(true);

            //notify driver robot's current color value
            telemetry.addData("Color Sensor", colorSensor.alpha());
            telemetry.update();

            //move robot to find white line
            DriveForward(-0.07);
            idle();
        }

        StopDriving();

        //while color sensor goes too far
        while(!isStopRequested() && colorSensor.alpha() < maxFindWhite){
            //Turn on LED
            colorSensor.enableLed(true);

            //notifies driver robot's current color value
            telemetry.addData("Color Sensor", colorSensor.alpha());
            telemetry.update();

            //move robot slowly backwards to find white line
            DriveForward(0.07);
            idle();
        }

        if(colorSensor.alpha() > minFindWhite){
            StopDriving();
        }

        //stops robot
        StopDriving();
        TurnOffLEDs();
        ModeRunUsingEncoder();
        idle();

    }

    public void findHighColor() throws InterruptedException {

        ModeRunWithoutEncoders();

        while(!isStopRequested() && highColorSensor.red() == 0 && highColorSensor.blue() == 0){
            //while highColorSensor cannot read any color values, it moves the robot back

            CDI.setLED(0, false);       //Blue OFF
            CDI.setLED(1, false);       //Red OFF

            //notify driver robot's current color value
            telemetry.addData("Red", highColorSensor.red());
            telemetry.addData("Blue", highColorSensor.blue());
            //notifies driver of error status
            telemetry.addData("ERROR:","sensor cannot find color values");
            telemetry.addData(">", "Moving backwards...");
            telemetry.update();
            //error, equal color value, moving back
            DriveForward(-0.1);
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
            telemetry.addData("ERROR:","colors are equal value");
            telemetry.addData(">", "Moving backwards...");
            telemetry.update();
            //error, equal color value, moving back
            DriveForward(-0.1);
            idle();
        }

        StopDriving();

        if (highColorSensor.red() > highColorSensor.blue()){
            //Right side of beacon is red

            StopDriving();

            CDI.setLED(0, false);       //Blue OFF
            CDI.setLED(1, true);        //Red ON

            //notify driver robot's current color value
            telemetry.addData("Red", highColorSensor.red());
            telemetry.addData("Blue", highColorSensor.blue());
            //notifies driver of beacon status
            telemetry.addData("Left side of beacon is","RED");
            telemetry.update();

            DriveForwardDistance(0.1, -14);
            PokeBeacon();

        } else if (highColorSensor.red() < highColorSensor.blue()){
            //Right side of beacon is blue

            StopDriving();

            CDI.setLED(0, true);        //Blue ON
            CDI.setLED(1, false);       //Red OFF

            //notify driver robot's current color value
            telemetry.addData("Red", highColorSensor.red());
            telemetry.addData("Blue", highColorSensor.blue());
            //notifies driver of beacon status
            telemetry.addData("Left side of beacon is","BLUE");
            telemetry.update();

            PokeBeacon();
        }

        //stops robot and turns off LEDs
        TurnOffLEDs();
        CDI.setLED(0, false);       //Blue OFF
        CDI.setLED(1, false);       //Red OFF
        StopDriving();
        ModeRunUsingEncoder();
        idle();
    }

    public void TurnOffLEDs(){
        //turns off color sensor LEDs
        colorSensor.enableLed(false);
        highColorSensor.enableLed(false);
    }

    public void PokeBeacon(){
        //Moves CRServo to hit beacon and move back
        beaconPoker.setPower(CRServoForward);
        sleep(2000);
        beaconPoker.setPower(CRServoBackward);
        sleep(2000);
        beaconPoker.setPower(CRServoStop);
        sleep(50);
    }

}
