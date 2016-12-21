package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by 5661 on 12/13/2016.
 */
@Autonomous(name = "TurnTestTwoMotor", group = "Autonomous OpMode")
public class TurnTest1 extends LinearOpMode {

    DcMotor motorRight;
    DcMotor motorLeft;
    int encoderTicks = 1120;
    double wheelCircumference = 23.9268;
    double slowSpeed = 0.13;
    int shortSleep = 100;

    @Override
    public void runOpMode() throws InterruptedException {

        motorRight = hardwareMap.dcMotor.get("right_motor");
        motorLeft = hardwareMap.dcMotor.get("left_motor");
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        driveForwardDistance(0.5, 50, shortSleep);
        turnRightDistance(slowSpeed, 40, shortSleep);
        driveForwardDistance(0.5, 100, shortSleep);
        turnLeftDistance(slowSpeed, 40, shortSleep);

    }

    public void driveForwardDistance(double power, int distance, int sleepTime) throws InterruptedException {
        //DriveForwardDistance is used to move the robot forward a specific distance

        modeResetEncoders();

        //distance in cm divided by the wheel circumference times motor encoder ticks
        double distanceSet = ((distance/(wheelCircumference))* encoderTicks);
        //set target position
        motorLeft.setTargetPosition ((int)distanceSet);
        motorRight.setTargetPosition ((int)distanceSet);

        sleep(50);

        modeRunToPosition();

        driveForward(power);

        while(motorLeft.isBusy() && motorRight.isBusy()){
            //wait until target position is reached
        }

        //stop and change modes back to normal
        stopDriving();
        modeRunUsingEncoder();
        sleep(sleepTime);
    }
    public void turnLeftDistance(double power, int distance, int sleepTime) throws InterruptedException {
        //TurnLeftDistance  is used to turn the robot left a specific distance

        modeResetEncoders();

        //distance in cm divided by the wheel circumference times motor encoder ticks
        double distanceSet = ((distance/(wheelCircumference))* encoderTicks);
        //set target position
        motorLeft.setTargetPosition ((int)-distanceSet);
        motorRight.setTargetPosition ((int)distanceSet);

        sleep(50);

        modeRunToPosition();

        driveForward(power);

        while(!isStopRequested() && motorLeft.isBusy() && motorRight.isBusy()){
            //wait until target position is reached
        }

        //stop and change modes back to normal
        stopDriving();
        modeRunUsingEncoder();
        sleep(sleepTime);
    }
    public void turnRightDistance(double power, int distance, int sleepTime) throws InterruptedException {
        //TurnRightDistance is used to turn the robot right a specific distance

        modeResetEncoders();

        //distance in cm divided by the wheel circumference times motor encoder ticks
        double distanceSet = ((distance/(wheelCircumference))* encoderTicks);
        //set target position
        motorLeft.setTargetPosition ((int)distanceSet);
        motorRight.setTargetPosition ((int)-distanceSet);

        sleep(50);

        modeRunToPosition();

        driveForward(power);

        while(!isStopRequested() && motorLeft.isBusy() && motorRight.isBusy()){
            //wait until target position is reached
        }

        //stop and change modes back to normal
        stopDriving();
        modeRunUsingEncoder();
        sleep(sleepTime);
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
}
