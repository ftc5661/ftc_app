package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by 5661 on 10/17/2016.
 */
@Autonomous(name = "TestGyro3", group = "Autonomous OpMode")
public class TestGyro3 extends LinearOpMode {

    //declares motors, servos, and other data
    DcMotor motorRight;
    DcMotor motorLeft;
    ModernRoboticsI2cGyro mrGyro;
    GyroSensor sensorGyro;
    public int numRev = 0;
    public int prevHeading;
    public double DegreesToTurn;
    public float left = -1;
    public float right = 1;

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

        while (!isStopRequested() && mrGyro.isCalibrating()){
            //Ensure calibration is complete (usually 2 seconds)
            sleep(50);
            idle();
        }
        prevHeading = mrGyro.getHeading();

        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();

        //wait for the game to start(press the play button)
        waitForStart();

        //turn(90, right);
        turnGyroRight(90);

        telemetry.addData(">", "Autonomous Done");
        telemetry.update();

    }

    public void turnGyroRight(double deg) throws InterruptedException{
        //Turns robot by using MRGyro mounted on robot

        telemetry.addData(">", "Turning robot ", deg, "degees");
        telemetry.update();

        double target = mrGyro.getHeading() + deg;
        double TOLERANCE = 2;
        double slowMove = 8;
        //double error_degrees = target_angle_degrees - zAccumulated;
        //double motor_output = (error_degrees / 180.0) + (error_degrees > 0 ? -.1 : .1);

        while(!isStopRequested() && Math.abs(target - mrGyro.getHeading()) > TOLERANCE){
            telemetry.addData(">", "Robot is currently turning");
            telemetry.update();

            //double gyroPower = (target - mrGyro.getHeading())/(5*target); //((target - mrGyro.getHeading() > 0 ? -.1 : .1));
            double gyroPower = 0.0331111*1.2; //minimun motor value //add this to above gyropower so 0 is ajusted // TODO
            motorLeft.setPower(gyroPower);
            motorRight.setPower(-gyroPower);

            /*
            if (Math.abs(error_degrees) > 30) {
                motorLeft.setPower(.5 * Math.signum(error_degrees));
                motorRight.setPower(-.5 * Math.signum(error_degrees));
            } else {
                motorLeft.setPower(motor_output);
                motorRight.setPower(-motor_output);
            }
            */
            idle();
        }
        motorLeft.setPower(0);
        motorRight.setPower(0);
    }
/*
    public void checkGyro(){
        telemetry.addData(">", "Checking if gyro has turned over");
        telemetry.update();

        int currHeading = mrGyro.getHeading();
        if(Math.abs(currHeading - prevHeading) >= 180){
            //detected wrap around
            if (currHeading > prevHeading){
                numRev--;
            } else {
                numRev++;
            }
            prevHeading = currHeading;
        }
    }
    */
/*
    public int getGyroHeading(){
        return numRev*360 + mrGyro.getHeading();
    }
    */

    public void turn(int Degrees, float Direction){
        if (Direction < 0){
            DegreesToTurn = 360 - Degrees;
        } else {
            DegreesToTurn = Degrees;
        }

        if (Direction > 0){
            while (!isStopRequested() && mrGyro.getHeading() < DegreesToTurn){
                motorRight.setPower(-0.1);
                motorLeft.setPower(0.1);
                telemetry.addData("Heading", mrGyro.getHeading());
                telemetry.addData(">", "Turning right");
            }
            motorLeft.setPower(0);
            motorRight.setPower(0);
            telemetry.addData(">","Turning point reached");
        } else {
            while (!isStopRequested() && mrGyro.getHeading() > DegreesToTurn){
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

    public void turnUsingCurrentPos(int Degrees, float Direction) throws InterruptedException{ //counter-clockwise
        //turnGyro turns robot from current position. careful when using this method, small errors in movement can add up!

        turn(Degrees + mrGyro.getIntegratedZValue(), Direction);
    }

}
