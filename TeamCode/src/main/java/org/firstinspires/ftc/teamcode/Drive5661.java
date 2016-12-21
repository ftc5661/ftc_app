package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by 5661 on 11/3/2016.
 *
 * TeleOp program for driving the 5661 robot
 *
 */
@TeleOp(name = "Tele Op Drive 5661", group = "TeleOp")
public class Drive5661 extends OpMode {

    //declare motors, servos, sensors
    DcMotor motorDriveRight;
    DcMotor motorDriveLeft;
    DcMotor motorBallCollect;
    DcMotor motorCapBallLeft;
    DcMotor motorCapBallRight;
    DcMotor motorShootBall;
    CRServo servoGateLeft;
    CRServo servoGateRight;
    CRServo beaconPoker;
    int servoGatePowerLeft;
    int servoGatePowerRight;
    int servoPokerPower;

    @Override
    public void init() {
        //set java names to hardware map names
        motorDriveLeft = hardwareMap.dcMotor.get("left_motor");
        motorDriveRight = hardwareMap.dcMotor.get("right_motor");
        motorBallCollect = hardwareMap.dcMotor.get("collect_motor");
        motorCapBallLeft = hardwareMap.dcMotor.get("cap_ball_left");
        motorCapBallRight = hardwareMap.dcMotor.get("cap_ball_right");
        motorShootBall = hardwareMap.dcMotor.get("shoot_ball");
        servoGateLeft = hardwareMap.crservo.get("left_servo");
        servoGateRight = hardwareMap.crservo.get("right_servo");
        beaconPoker = hardwareMap.crservo.get("beacon_poker");
        //reverse right motor to drive
        motorDriveRight.setDirection(DcMotor.Direction.REVERSE);
        //reverse right cap ball motor
        motorCapBallRight.setDirection(DcMotorSimple.Direction.REVERSE);

        servoGatePowerLeft = 0;
        servoGatePowerRight = 0;
        servoPokerPower = 0;

    }

    @Override
    public void loop() {

                                    //GAMEPAD 1

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
        motorDriveRight.setPower(rightStick);
        motorDriveLeft.setPower(leftStick);

        //right and left bumper controls lift motors
        if (gamepad1.right_bumper){
            //lifts cap ball
            motorCapBallLeft.setPower(1);
            motorCapBallRight.setPower(1);
        }
        if (gamepad1.left_bumper){
            //lowers cap ball
            motorCapBallLeft.setPower(-1);
            motorCapBallRight.setPower(-1);
        }
        if (!gamepad1.right_bumper && !gamepad1.left_bumper){
            //does nothing
            motorCapBallLeft.setPower(0);
            motorCapBallRight.setPower(0);
        }

                                    //GAMEPAD 2

        //if the a button is pressed, servos close
        if (gamepad2.a){
            servoGatePowerLeft = -1;
            servoGatePowerRight = 1;
        }
        if (gamepad2.b){
            servoGatePowerLeft = 1;
            servoGatePowerRight = -1;
        }
        if (!gamepad2.a && !gamepad2.b){
            servoGatePowerLeft = 0;
            servoGatePowerRight = 0;
        }
        //sets servo position to servoPosition var
        servoGateLeft.setPower(servoGatePowerLeft);
        servoGateRight.setPower(servoGatePowerRight);

        //if right bumper is pressed, motor collects ball into robot
        if (gamepad2.right_bumper){
            motorBallCollect.setPower(-1);
        }
        if (gamepad2.left_bumper){
            motorBallCollect.setPower(1);
        }
        if (!gamepad2.right_bumper && !gamepad2.left_bumper){
            motorBallCollect.setPower(0);
        }

        //if left bumper is pressed, motor shoots ball

        if (gamepad2.dpad_right){
            motorShootBall.setPower(-1);
        }
        if (gamepad2.dpad_left){
            motorShootBall.setPower(1);
        }
        if (!gamepad2.dpad_left && !gamepad2.dpad_right){
            motorShootBall.setPower(0);
        }

        //if x is pressed pokes beacons, if y is pressed beacon poker retracts
        if (gamepad2.x){
            servoPokerPower = 1;
        }
        if (gamepad2.y){
            servoPokerPower = -1;
        }
        if (!gamepad2.x && !gamepad2.y){
            servoPokerPower = 0;
        }

        beaconPoker.setPower(servoPokerPower);

        //outputs the motor and servo values to the drive station
        telemetry.addData("Controller", "One");
        telemetry.addData("Left Stick", leftStick);
        telemetry.addData("Right Stick", rightStick);
        telemetry.addData("Cap Ball Left Motor Speed", motorCapBallLeft.getPower());
        telemetry.addData("Cap Ball Right Motor Speed", motorCapBallRight.getPower());
        telemetry.addLine();
        telemetry.addData("Controller", "Two");
        telemetry.addData("Left Servo Position", servoGatePowerLeft);
        telemetry.addData("Right Servo Position", servoGatePowerRight);
        telemetry.addData("Collect Ball Motor Speed", motorBallCollect.getPower());
        telemetry.addData("Shoot Ball Motor Speed", motorShootBall.getPower());
        telemetry.addData("Beacon Poker Status", servoPokerPower);
        telemetry.update();
    }
}
