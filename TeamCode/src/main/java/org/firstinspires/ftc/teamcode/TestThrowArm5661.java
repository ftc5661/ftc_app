package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by 5661 on 10/5/2016.
 * TeleOp program to catapult the ball using the 5661 robot
 */
@TeleOp(name = "TestThrowArm5661", group = "TeleOp")
@Disabled
public class TestThrowArm5661 extends OpMode {

    //Declare motors

    // DcMotor motorRight1;
   // DcMotor motorRight2;
   // DcMotor motorLeft1;
   // DcMotor motorLeft2;
    DcMotor motorThrowArm;
    Servo stopThrowServo;
    double stopBar = 1;
    double goBar = 0;
    double SERVO_POSITION = 1;


    @Override
    public void init() {

        //set motor names
     //   motorRight1 = hardwareMap.dcMotor.get("right_motor1");
     //   motorRight2 = hardwareMap.dcMotor.get("right_motor2");
     //   motorLeft1 = hardwareMap.dcMotor.get("left_motor1");
     //   motorLeft2 = hardwareMap.dcMotor.get("left_motor2");
        motorThrowArm = hardwareMap.dcMotor.get("throw_arm_motor");
        stopThrowServo = hardwareMap.servo.get("hold_servo");
        //reverse right motors
     //   motorRight1.setDirection(DcMotor.Direction.REVERSE);
     //   motorRight2.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {

        stopThrowServo.setPosition(SERVO_POSITION);

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
     //   motorRight1.setPower(rightStick);
     //   motorRight2.setPower(rightStick);
    //    motorLeft1.setPower(leftStick);
    //    motorLeft2.setPower(leftStick);

        telemetry.addData("Left Stick", leftStick);
        telemetry.addData("Right Stick", rightStick);
        telemetry.update();


        if (gamepad1.a){
            //notifies driver servo is holding throwing arm shut
            telemetry.addData(">", "Stopping ball throw...");
            telemetry.update();

            SERVO_POSITION = stopBar;

        } else if(!gamepad1.a) {
            //notifies driver servo is moving away from throwing arm
            telemetry.addData(">", "Letting ball be thrown...");
            telemetry.update();

            SERVO_POSITION = goBar;
        }


        while (gamepad1.x) {
            //notifies driver robot is throwing ball
            telemetry.addData(">", "Throwing ball...");
            telemetry.update();

            MoveForward(1);

            /*
            //reset encoders
            ModeResetEncodersThrowArm();

            //set target position
            motorThrowArm.setTargetPosition(120);

            //set to RUN_TO_POSITION mode
            ModeRunToPositionThrowArm();

            //set drive power
            motorThrowArm.setPower(1);

            while (motorThrowArm.isBusy()) {

            }

            //stop and change modes back to normal
            motorThrowArm.setPower(0);
            ModeRunUsingEncoderThrowArm();

            /*
            //reset encoders
            ModeResetEncodersThrowArm();

            //set target position
            motorThrowArm.setTargetPosition (90);

            //set to RUN_TO_POSITION mode
            ModeRunToPositionThrowArm();

            //set drive power
            motorThrowArm.setPower(1);

            try {
                Thread.sleep(1000);                 //1000 milliseconds is one second.
            } catch(InterruptedException ex) {
                Thread.currentThread().interrupt();
            }

            SERVO_POSITION = goBar;

            while(motorThrowArm.isBusy()){
                SERVO_POSITION = goBar;

            }

            SERVO_POSITION = goBar;

            //stop and change modes back to normal
            motorThrowArm.setPower(0);
            ModeRunUsingEncoderThrowArm();

            //ReturnArm reset throwing arm back to the initial position

            //notifies driver robot is Returning ball
            String returnBall = "Returning ball...";
            telemetry.addData("Status", returnBall);
            telemetry.update();

            //reset encoders
            ModeResetEncodersThrowArm();

            //set target position
            motorThrowArm.setTargetPosition (-90);

            //set to RUN_TO_POSITION mode
            ModeRunToPositionThrowArm();

            //set drive power
            motorThrowArm.setPower(.2);

            while(motorThrowArm.isBusy()){
                //wait until target position is reached
            }

            //stop and change modes back to normal
            motorThrowArm.setPower(0);
            ModeRunUsingEncoderThrowArm();
            SERVO_POSITION = stopBar;

            */



        }
        while (gamepad1.y) {
            //notifies driver robot is throwing ball
            telemetry.addData(">", "Throwing ball...");
            telemetry.update();

            MoveBackward(0.5);

            /*
            //reset encoders
            ModeResetEncodersThrowArm();

            //set target position
            motorThrowArm.setTargetPosition(-120);

            //set to RUN_TO_POSITION mode
            ModeRunToPositionThrowArm();

            //set drive power
            motorThrowArm.setPower(1);

            //while (motorThrowArm.isBusy()) {

            //}

            //stop and change modes back to normal
            motorThrowArm.setPower(0);
            ModeRunUsingEncoderThrowArm();
            */
        }

        if (!gamepad1.x && !gamepad1.y){
            StopMoving();
        }
    }
/*

    public void ThrowBall() throws InterruptedException {
        //ThrowBall throws a ball using a motor with encoders

        //notifies driver robot is throwing ball
        String throwBall = "Throwing ball...";
        telemetry.addData("Status", throwBall);
        telemetry.update();

        SERVO_POSITION = stopBar;

        //reset encoders
        ModeResetEncodersThrowArm();

        //set target position
        motorThrowArm.setTargetPosition (90);

        //set to RUN_TO_POSITION mode
        ModeRunToPositionThrowArm();

        //set drive power
        motorThrowArm.setPower(1);

        while(motorThrowArm.isBusy()){
            //wait until target position is reached
            long setTime2 = System.currentTimeMillis();
            boolean hasRun2 = false;
            if(System.currentTimeMillis() - setTime2 > 2000 && !hasRun2){
                //will only run after 2 seconds, and will only run once
                hasRun2 = true;
                SERVO_POSITION = goBar;
            }
        }
        SERVO_POSITION = goBar;

        //stop and change modes back to normal
        motorThrowArm.setPower(0);
        ModeRunUsingEncoderThrowArm();
        ReturnArm();
    }

    public void ReturnArm() throws InterruptedException {
        //ReturnArm reset throwing arm back to the initial position

        //notifies driver robot is Returning ball
        String returnBall = "Returning ball...";
        telemetry.addData("Status", returnBall);
        telemetry.update();

        //reset encoders
        ModeResetEncodersThrowArm();

        //set target position
        motorThrowArm.setTargetPosition (-90);

        //set to RUN_TO_POSITION mode
        ModeRunToPositionThrowArm();

        //set drive power
        motorThrowArm.setPower(.2);

        while(motorThrowArm.isBusy()){
            //wait until target position is reached
        }

        //stop and change modes back to normal
        motorThrowArm.setPower(0);
        ModeRunUsingEncoderThrowArm();

        SERVO_POSITION = stopBar;
    }

    public void ModeResetEncodersThrowArm(){
        //reset encoders
        motorThrowArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void ModeRunToPositionThrowArm(){
        //set to RUN_TO_POSITION mode
        motorThrowArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void ModeRunUsingEncoderThrowArm(){
        //set to RUN_USING_ENCODER mode
        motorThrowArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    */
    public void MoveForward(double power){
        //sets the motor speed to 'power'
        motorThrowArm.setPower(power);

    }

    public void MoveBackward(double power){
        //sets the motor speed to 'power'
        MoveForward(-power);
    }

    public void StopMoving(){
        //stops driving by making speed 0
        MoveForward(0);
    }
}
