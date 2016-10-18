package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by 5661 on 9/10/2016.
 * TeleOp program for driving the 5661 programming robot
 */
@TeleOp(name = "Test Drive", group = "TeleOp" )
public class testdrive extends OpMode {

    //declare DcMotors
    DcMotor motorRight;
    DcMotor motorLeft;
    //declare Servos
    Servo turnMetal;
    //used for servo position
    double servoPosition;

    @Override
    public void init() {
        //set motor names
        motorRight = hardwareMap.dcMotor.get("right_motor");
        motorLeft = hardwareMap.dcMotor.get("left_motor");
        //reverse right motor
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        //set servo name
        turnMetal = hardwareMap.servo.get("servo1");
        //sets start position
        turnMetal.setPosition(0);

    }

    @Override
    public void loop() {

        //sets leftStick and rightStick as the gamepad y values
        float leftStick = -gamepad1.left_stick_y;
        float rightStick = -gamepad1.right_stick_y;

        //makes sure values don't go out of range of -1 to 1
        if (leftStick > 1.0){
            leftStick = 1;
        }
        if (leftStick < -1.0){
            leftStick = -1;
        }
        if (rightStick > 1.0){
            rightStick = 1;
        }
        if (rightStick < -1.0){
            rightStick = -1;
        }

        //sets the power to be the joystick values
        motorRight.setPower(rightStick);
        motorLeft.setPower(leftStick);

        //sets the servo position to the value set after this
        turnMetal.setPosition(servoPosition);

        //if the a button is pressed, servo goes to position 0
        if (gamepad1.a){
            servoPosition = 0;
        }

        //if the b button is pressed, servo goes to position 1
        if (gamepad1.b){
            servoPosition = 1;
        }

        //outputs the motor and servo values to the drive station
        telemetry.addData("Left Stick", leftStick);
        telemetry.addData("Right Stick", rightStick);
        telemetry.addData("Servo Position", servoPosition);
        telemetry.update();

    }
}
