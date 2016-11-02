package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by 5661 on 9/10/2016.
 * TeleOp program for driving the 5661 robot
 */
@TeleOp(name = "Test Drive", group = "TeleOp" )
public class testdrive extends OpMode {

    //declare DcMotors
    DcMotor motorRight;
    DcMotor motorLeft;
    DcMotor motorBallCollect;
    //declare Servos
    Servo servoGateLeft;
    Servo servoGateRight;
    //used for servo position
    double servoPositionLeft;
    double servoPositionRight;

    @Override
    public void init() {
        //set motor names
        motorRight = hardwareMap.dcMotor.get("right_motor");
        motorLeft = hardwareMap.dcMotor.get("left_motor");
        motorBallCollect = hardwareMap.dcMotor.get("collect_motor");
        //reverse right motor
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        //set servo name
        servoGateLeft = hardwareMap.servo.get("left_servo");
        servoGateRight = hardwareMap.servo.get("right_servo");
        //sets start position
        servoGateLeft.setPosition(0);
        servoGateRight.setPosition(0);

        telemetry.addData(">", "Robot Initialized");
        telemetry.update();

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

        //sets the servo position to the value set from the button pressed from the driver
        servoGateLeft.setPosition(servoPositionLeft);
        servoGateRight.setPosition(servoPositionRight);

        //if the a button is pressed, servo close ball in robot gates
        if (gamepad1.a){
            servoPositionLeft = 0;
            servoPositionRight = 1;
        }

        //if the b button is pressed, servo open gates
        if (gamepad1.b){
            servoPositionLeft = 1;
            servoPositionRight = 0;
        }

        //if right bumper is pressed, motor collects ball into robot
        if (gamepad1.right_bumper){
            motorBallCollect.setPower(0.5);
        } else if (!gamepad1.right_bumper){
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