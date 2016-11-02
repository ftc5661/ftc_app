package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by 5661 on 10/17/2016.
 * Autonomous program for testing the Modern Robotics Gyro Sensor for the 5661 robot
 */
@Autonomous(name = "TestGyro3", group = "Autonomous OpMode")
@Disabled
public class TestGyro3 extends LinearOpMode {

    //declares motors, servos, and other data
    DcMotor motorRight;
    DcMotor motorLeft;
    ModernRoboticsI2cGyro mrGyro;
    GyroSensor sensorGyro;
    public double DegreesToTurn;
    //ZERO_OFFSET makes sure the robot doesn't stop just before the target, due to weight, friction, etc.
    //IMPORTANT: CHANGE THIS VALUE IF ROBOT HAS CHANGED SIGNIFICANTLY, I.E. NEW ROBOT, MORE MOTOR WHEEL, MORE/LESS WEIGHT, ETC.
    public double GYRO_ZERO_OFFSET = 0.05;
    //+-tolerance because of floating point numbers
    public double GYRO_TOLERANCE = 2;

    @Override
    public void runOpMode() throws InterruptedException {

        //set motor names
        motorRight = hardwareMap.dcMotor.get("right_motor");
        motorLeft = hardwareMap.dcMotor.get("left_motor");
        //reverse right motor
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        sensorGyro = hardwareMap.gyroSensor.get("gyro");
        mrGyro = (ModernRoboticsI2cGyro) sensorGyro; //allows us to get .getIntegratedZValue()

        // start calibrating the gyro.
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        mrGyro.calibrate();
        // DO NOT MOVE SENSOR WHILE BLUE LIGHT IS SOLID

        while (!isStopRequested() && mrGyro.isCalibrating()) {
            //Ensure calibration is complete (usually 2 seconds)
            sleep(100);
            idle();
        }

        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();

        //wait for the game to start(press the play button)
        waitForStart();

        //turn(90, gyroRight);
        turnGyroRight(83);

        telemetry.addData(">", "Autonomous Done");
        telemetry.update();
        sleep(300);

    }

    public void turnGyroRight(double deg) throws InterruptedException{
        //Turns robot by using MRGyro mounted on robot, the motors will slow down over time to stay accurate
        //Remember: .getHeading() goes from 1-359 clockwise
        //Remember: Mount gryo near front of robot, and lay gyro flat

        double gyroTarget;

        //current heading + degrees wanted to turn
        double target = mrGyro.getHeading() + deg;

        //Fixes .getHeading() rollover
        if (target > 359){
            gyroTarget = target - 359;
        } else {
            gyroTarget = target;
        }

        //while gyro's .getHeading() value is higher than tolerance, then turn
        while (!isStopRequested() && Math.abs(target - mrGyro.getHeading()) > GYRO_TOLERANCE) {
            telemetry.addData(">", "Robot is currently turning right");
            telemetry.addData("Heading", mrGyro.getHeading());
            telemetry.update();

            double gyroPower = (Math.abs(target - mrGyro.getHeading())/(6*target)) + GYRO_ZERO_OFFSET;
            motorLeft.setPower(gyroPower);
            motorRight.setPower(gyroPower);
                idle();
            }

        motorLeft.setPower(0);
        motorRight.setPower(0);
        telemetry.addData(">","Turning point reached");
        telemetry.update();
    }

    public void turnGyroLeft(double deg) throws InterruptedException{
        //Turns robot by using MRGyro mounted on robot, the motors will slow down over time to stay accurate
        //Remember: .getHeading() goes from 1-359 clockwise
        //Remember: Mount gryo near front of robot, and lay gyro flat

        double gyroTarget;

        //changes degrees from right to left
        DegreesToTurn = 360 - deg;

        //current heading + degrees wanted to turn
        double target = mrGyro.getHeading() + deg;

        //Fixes .getHeading() rollover
        if (target > 359){
            gyroTarget = target - 359;
        } else {
            gyroTarget = target;
        }

        //while gyro's .getHeading() value is higher than tolerance, then turn
        while (!isStopRequested() && Math.abs(target - mrGyro.getHeading()) > GYRO_TOLERANCE) {
            telemetry.addData(">", "Robot is currently turning left");
            telemetry.addData("Heading", mrGyro.getHeading());
            telemetry.update();

            double gyroPower = (Math.abs(target - mrGyro.getHeading())/(6*target)) + GYRO_ZERO_OFFSET;
            motorLeft.setPower(-gyroPower);
            motorRight.setPower(gyroPower);
            idle();
        }

        motorLeft.setPower(0);
        motorRight.setPower(0);
        telemetry.addData(">","Turning point reached");
        telemetry.update();
    }
/*
    public int getGyroHeading(){
        return mrGyro.getIntegratedZValue();
    }


    public void turn(int Degrees, float Direction){
        double wDegreesToTurn;
        if (Direction < 0){
            wDegreesToTurn = 360 - Degrees;
        } else {
            wDegreesToTurn = Degrees;
        }

        if (Direction > 0){
            while (!isStopRequested() && mrGyro.getHeading() < wDegreesToTurn){
                motorRight.setPower(-0.1);
                motorLeft.setPower(0.1);
                telemetry.addData("Heading", mrGyro.getHeading());
                telemetry.addData(">", "Turning right");
            }
            motorLeft.setPower(0);
            motorRight.setPower(0);
            telemetry.addData(">","Turning point reached");
        } else {
            while (!isStopRequested() && mrGyro.getHeading() > wDegreesToTurn){
                motorRight.setPower(0.1);
                motorLeft.setPower(-0.1);
                telemetry.addData("Heading", mrGyro.getHeading());
                telemetry.addData(">", "Turning left");
            }
            motorLeft.setPower(0);
            motorRight.setPower(0);
            telemetry.addData(">","Turning point reached");
        }
    }
    */
}
