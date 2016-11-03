package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Test TeleOp program for testing the programming robot beacon presser CRServo
 */

@TeleOp(name = "IanBeaconTest_101016", group = "TeleOp")
public class IanBeaconTest_101016 extends OpMode {

    //Declare motors
    DcMotor motorRight1;
    DcMotor motorLeft1;
    CRServo beaconPoker;

    @Override
    public void init() {

        //set motor names
        motorRight1 = hardwareMap.dcMotor.get("right_motor");
        motorLeft1 = hardwareMap.dcMotor.get("left_motor");
        beaconPoker = hardwareMap.crservo.get("beacon_poker");
        //reverse right motors
        motorRight1.setDirection(DcMotor.Direction.REVERSE);

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
        motorLeft1.setPower(leftStick);

        telemetry.addData("Left Stick", leftStick);
        telemetry.addData("Right Stick", rightStick);
        telemetry.update();

        if(gamepad1.a) {
            beaconPoker.setPower(1);
        }

        if(gamepad1.b) {
            beaconPoker.setPower(-1);
        }

        if(!gamepad1.a && !gamepad1.b) {
            beaconPoker.setPower(0);
        }

    }

}
