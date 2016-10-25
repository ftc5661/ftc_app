package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * Created by 5661 on 10/12/2016.
 * Test Autonomous program for testing the gyro on the 5661 programming robot
 */
@Autonomous(name = "TestGyro", group = "Autonomous OpMode")
@Disabled
public class TestGyro extends LinearOpMode{

    //declares motors, servos, and other data
    DcMotor motorRight;
    DcMotor motorLeft;
    GyroSensor sensorGyro;
    ModernRoboticsI2cGyro mrGyro;

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

        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();

        //wait for the game to start(press the play button)
        waitForStart();

        turnAbsoluteGyro(90);

    }

    public void turnAbsoluteGyro(int target) throws InterruptedException{
        //turnAbsoluteGyro turns robot from initial calibration, you have to know how much to turn according from your starting point

        telemetry.addData(">", "Turning robot ", target, "degees");
        telemetry.update();

        int zAccumulated = mrGyro.getIntegratedZValue(); //set variables to gyro readings
        double turnSpeed = 0.15;

        while(!isStopRequested() && Math.abs(zAccumulated - target) > 2){

            if (zAccumulated > target){ //if gyro is positive,  the robot will turn right
                motorLeft.setPower(turnSpeed);
                motorRight.setPower(-turnSpeed);

            }

            if (zAccumulated < target){ //if gyro is negative, the robot will turn left
                motorLeft.setPower(-turnSpeed);
                motorRight.setPower(turnSpeed);

            }

            zAccumulated = mrGyro.getIntegratedZValue(); //set variables to gyro readings
            telemetry.addData("1. accu", String.format("%03d", zAccumulated));
            telemetry.update();
            idle();

        }

        //stops motors
        motorLeft.setPower(0);
        motorRight.setPower(0);

        telemetry.addData("1. accu", String.format("%03d", zAccumulated));
        telemetry.update();

        idle();

    }
    public void turnGyro(int target) throws InterruptedException{ //counter-clockwise
        //turnGyro turns robot from current position. careful when using this method, small errors in movement can add up!

        turnAbsoluteGyro(target + mrGyro.getIntegratedZValue());
    }
}
