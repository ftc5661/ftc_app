package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by 5661 on 9/19/2016.
 * TeleOp program for driving the 5661 robot
 */
@TeleOp(name = "TeleOpFourMotor", group = "TeleOp")
@Disabled
public class TeleOpFourMotor extends OpMode{

    //declare DcMotors
    DcMotor motorRight1;
    DcMotor motorRight2;
    DcMotor motorLeft1;
    DcMotor motorLeft2;

    @Override
    public void init() {

        //set motor names
        motorRight1 = hardwareMap.dcMotor.get("right_motor1");
        motorRight2 = hardwareMap.dcMotor.get("right_motor2");
        motorLeft1 = hardwareMap.dcMotor.get("left_motor1");
        motorLeft2 = hardwareMap.dcMotor.get("left_motor2");
        //reverse right motors
        motorRight1.setDirection(DcMotor.Direction.REVERSE);
        motorRight2.setDirection(DcMotor.Direction.REVERSE);

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
        motorRight1.setPower(rightStick);
        motorRight2.setPower(rightStick);
        motorLeft1.setPower(leftStick);
        motorLeft2.setPower(leftStick);

        telemetry.addData("Left Stick", leftStick);
        telemetry.addData("Right Stick", rightStick);
        telemetry.update();

    }
}
