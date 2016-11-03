package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by 5661 on 11/3/2016.
 * TeleOp program for driving the 5661 robot
 */
@TeleOp(name = "Drive5661", group = "TeleOp")
public class Drive5661 extends OpMode {

    //declare motors, servos, sensors
    DcMotor motorRight;
    DcMotor motorLeft;
    DcMotor motorBallCollect;
    Servo servoGateLeft;
    Servo servoGateRight;
    double servoPositionLeft;
    double servoPositionRight;

    @Override
    public void init() {
        //set java names to hardware map names
        motorLeft = hardwareMap.dcMotor.get("left_motor");
        motorRight = hardwareMap.dcMotor.get("right_motor");
        motorBallCollect = hardwareMap.dcMotor.get("collect_motor");
        servoGateLeft = hardwareMap.servo.get("left_servo");
        servoGateRight = hardwareMap.servo.get("right_servo");
        //reverse right motor to drive
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        servoPositionLeft = 1;
        servoPositionRight = 0;

    }

    @Override
    public void loop() {

        //sets leftStick and rightStick as the gamepad y values
        float leftStick = -gamepad1.left_stick_y;
        float rightStick = -gamepad1.right_stick_y;

        //makes sure values don't go out of the range of -1 to 1
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

        //sets the wheel motor powers to be the joystick values
        motorRight.setPower(rightStick);
        motorLeft.setPower(leftStick);

        //if the a button is pressed, servos close
        if (gamepad2.a){
            servoPositionLeft = 0;
            servoPositionRight = 1;
        } else if (!gamepad2.a){
            servoPositionLeft = 1;
            servoPositionRight = 0;
        }

        //sets servo position to servoPosition var
        servoGateLeft.setPosition(servoPositionLeft);
        servoGateRight.setPosition(servoPositionRight);

        //if right bumper is pressed, motor collects ball into robot
        if (gamepad2.right_bumper){
            motorBallCollect.setPower(-0.5);
        } else if (!gamepad2.right_bumper){
            motorBallCollect.setPower(0);
        }

        //outputs the motor and servo values to the drive station
        telemetry.addData("Left Stick", leftStick);
        telemetry.addData("Right Stick", rightStick);
        telemetry.addData("Left Servo Position", servoPositionLeft);
        telemetry.addData("Right Servo Position", servoPositionRight);
        telemetry.addData("Collect Ball Motor Speed", motorBallCollect.getPower());
        telemetry.update();

    }
}
