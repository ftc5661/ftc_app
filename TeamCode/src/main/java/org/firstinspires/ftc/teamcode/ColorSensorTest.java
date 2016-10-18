package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cAddressableDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;

/**
 * Created by 5661 on 10/3/2016.
 * Test Autonomous program for testing the color sensors on the 5661 programming robot
 */
@Autonomous(name = "ColorSensorTest", group = "Autonomous OpMode")
@Disabled
public class ColorSensorTest extends LinearOpMode{

    //Declare motors and sensors
    DcMotor motorRight;
    DcMotor motorLeft;
    Servo turnMetal;
    ColorSensor colorSensor;
    ColorSensor highColorSensor;
    DeviceInterfaceModule CDI;
    //value when robot color sensor is centered on white
    public int maxFindWhite = 2;
    public int minFindWhite = 1;


    @Override
    public void runOpMode() throws InterruptedException {
        //set motor names
        motorRight = hardwareMap.dcMotor.get("right_motor");
        motorLeft = hardwareMap.dcMotor.get("left_motor");
        turnMetal = hardwareMap.servo.get("servo1");
        CDI = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");
        //reverse right motor
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        //set color sensor name and values
        colorSensor = hardwareMap.colorSensor.get("color");
        colorSensor.setI2cAddress(I2cAddr.create8bit(0x3c));
        highColorSensor = hardwareMap.colorSensor.get("high_color");
        highColorSensor.setI2cAddress(I2cAddr.create8bit(0x74));

        //resets color sensor led
        colorSensor.enableLed(false);
        highColorSensor.enableLed(false);
        sleep(50);
        //sets start position
        turnMetal.setPosition(0.5);


        //wait for start button to be presses
        waitForStart();

        findWhite();
        findHighColor();
        findWhiteBackwards();
        findHighColor();

        TurnOffLEDs();

        }



    public void findWhite() throws InterruptedException{

        //notifies driver robot is moving to find white
        String findWhite = "Moving to find white...";
        telemetry.addData("Status", findWhite);
        telemetry.update();

        //while color sensor see no white
        while(!isStopRequested() && colorSensor.alpha() < maxFindWhite){
            //Turn on LED
            colorSensor.enableLed(true);

            //notify driver robot's current color value
            telemetry.addData("Color Sensor", colorSensor.alpha());
            telemetry.update();

            //move robot to find white line
            motorLeft.setPower(0.1);
            motorRight.setPower(0.1);
            sleep(50);
            idle();
        }

        //stop the robot
        StopDrivingNoEncoders();

        //while color sensor goes too far
        while(!isStopRequested() && colorSensor.alpha() < maxFindWhite){
            //Turn on LED
            colorSensor.enableLed(true);

            //notifies driver robot's current color value
            telemetry.addData("Color Sensor", colorSensor.alpha());
            telemetry.update();

            //move robot slowly backwards to find white line
            motorLeft.setPower(-0.1);
            motorRight.setPower(-0.1);
            sleep(50);
            idle();
        }

        if(colorSensor.alpha() > minFindWhite){
            motorLeft.setPower(0);
            motorRight.setPower(0);
            sleep(50);
        }
        //stops robot
        StopDrivingNoEncoders();
        TurnOffLEDs();

    }

    public void findWhiteBackwards() throws InterruptedException{

        //notifies driver robot is moving to find white
        String findWhiteBackwards = "Moving backwards to find white...";
        telemetry.addData("Status", findWhiteBackwards);
        telemetry.update();

        //while color sensor see no white
        while(!isStopRequested() && colorSensor.alpha() < maxFindWhite){
            //Turn on LED
            colorSensor.enableLed(true);

            //notify driver robot's current color value
            telemetry.addData("Color Sensor", colorSensor.alpha());
            telemetry.update();

            //move robot to find white line
            motorLeft.setPower(-0.1);
            motorRight.setPower(-0.1);
            sleep(50);
            idle();
        }

        //stop the robot
        StopDrivingNoEncoders();

        //while color sensor goes too far
        while(!isStopRequested() && colorSensor.alpha() < maxFindWhite){
            //Turn on LED
            colorSensor.enableLed(true);

            //notifies driver robot's current color value
            telemetry.addData("Color Sensor", colorSensor.alpha());
            telemetry.update();

            //move robot slowly backwards to find white line
            motorLeft.setPower(0.1);
            motorRight.setPower(0.1);
            sleep(50);
            idle();
        }

        if(colorSensor.alpha() > minFindWhite){
            StopDrivingNoEncoders();
        }
        //stops robot
        StopDrivingNoEncoders();
        TurnOffLEDs();

    }

    public void findHighColor() throws InterruptedException {

        while(!isStopRequested() && highColorSensor.red() == 0 && highColorSensor.blue() == 0){
            //highColorSensor cannot read any color values, it moves the robot at an angle back, then forward

            CDI.setLED(0, false);       //Blue OFF
            CDI.setLED(1, false);       //Red OFF

            //notify driver robot's current color value
            telemetry.addData("Red", highColorSensor.red());
            telemetry.addData("Blue", highColorSensor.blue());
            telemetry.update();

            //notifies driver of error status
            String sameColor = "Moving backwards...";
            telemetry.addData("ERROR: ","sensor cannot find color values");
            telemetry.addData("Status", sameColor);
            telemetry.update();
            sleep(1000);
            idle();
            stop();
        }


        while(!isStopRequested() && highColorSensor.red() == highColorSensor.blue()){
            //highColorSensor reads red and blue as equal values

            StopDrivingNoEncoders();

            CDI.setLED(0, true);       //Blue ON
            CDI.setLED(1, true);       //Red ON

            //notify driver robot's current color value
            telemetry.addData("Red", highColorSensor.red());
            telemetry.addData("Blue", highColorSensor.blue());
            telemetry.update();

            //notifies driver of error status
            String sameColor = "Moving backwards...";
            telemetry.addData("ERROR: ","colors are equal value");
            telemetry.addData("Status", sameColor);
            telemetry.update();
            //error, equal color value, moving back
            motorLeft.setPower(-0.1);
            motorRight.setPower(-0.1);
            sleep(50);

        }

        if (highColorSensor.red() > highColorSensor.blue()){
            //Right side of beacon is red

            StopDrivingNoEncoders();

            CDI.setLED(0, false);       //Blue OFF
            CDI.setLED(1, true);        //Red ON

            //notify driver robot's current color value
            telemetry.addData("Red", highColorSensor.red());
            telemetry.addData("Blue", highColorSensor.blue());
            telemetry.update();

            //notifies driver of beacon status
            telemetry.addData("Right side is ","RED");
            telemetry.update();

            //sets start position
            turnMetal.setPosition(0);
            sleep(500);

        } else if (highColorSensor.red() < highColorSensor.blue()){
            //Right side of beacon is blue

            StopDrivingNoEncoders();

            CDI.setLED(0, true);        //Blue ON
            CDI.setLED(1, false);       //Red OFF

            //notify driver robot's current color value
            telemetry.addData("Red", highColorSensor.red());
            telemetry.addData("Blue", highColorSensor.blue());
            telemetry.update();

            //notifies driver of beacon status
            telemetry.addData("Right side is ","BLUE");
            telemetry.update();

            //sets start position
            turnMetal.setPosition(1);
            sleep(500);

        }

        StopDrivingNoEncoders();
        CDI.setLED(0, false);       //Blue OFF
        CDI.setLED(1, false);       //Red OFF
        TurnOffLEDs();
    }

    public void StopDrivingNoEncoders() throws InterruptedException {
        motorLeft.setPower(0);
        motorRight.setPower(0);
        sleep(50);
    }

    public void TurnOffLEDs(){
        colorSensor.enableLed(false);
        highColorSensor.enableLed(false);
    }
}
