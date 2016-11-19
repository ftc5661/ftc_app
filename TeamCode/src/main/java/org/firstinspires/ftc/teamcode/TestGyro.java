package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.Range;

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
    //GYRO_ZERO_OFFSET makes sure the robot always has enough power, so it doesn't stop just before the target, due to weight, friction, etc.
    //IMPORTANT: CHANGE THIS VALUE IF ROBOT HAS CHANGED SIGNIFICANTLY, I.E. NEW WHEELS, MORE MOTOR, MORE/LESS WEIGHT, ETC.
    public double GYRO_ZERO_OFFSET = 0.06;

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
        idle();
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

        //this should account for overturn
        turnAbsoluteGyro(80);

    }

    public void turnAbsoluteGyro(int target) throws InterruptedException{
        //turnAbsoluteGyro turns robot from initial calibration, you have to know how much to turn according from your starting point

        telemetry.addData(">", "Turning robot using gyro");
        telemetry.update();

        while(!isStopRequested() && Math.abs(mrGyro.getIntegratedZValue() - target) > 3){

            if (mrGyro.getIntegratedZValue() > target){ //if gyro is positive,  the robot will turn right
                telemetry.addData(">", "Robot is currently turning right");
                telemetry.addData("IntegratedZValue", mrGyro.getIntegratedZValue());
                telemetry.update();

                //double error_degrees = target - mrGyro.getIntegratedZValue();
                //double motor_output = (error_degrees / (7 * target)) + GYRO_ZERO_OFFSET;

                //motor_output = Range.clip(motor_output, -1, 1);

                motorLeft.setPower(0.15);
                motorRight.setPower(-0.15);
            }

            if (mrGyro.getIntegratedZValue() < target){ //if gyro is negative, the robot will turn left
                telemetry.addData(">", "Robot is currently turning left");
                telemetry.addData("IntegratedZValue", mrGyro.getIntegratedZValue());
                telemetry.update();

                //double error_degrees = target - mrGyro.getIntegratedZValue();
                //double motor_output = (error_degrees / (7 * target)) + GYRO_ZERO_OFFSET;

                //motor_output = Range.clip(motor_output, -1, 1);

                motorLeft.setPower(-0.1);
                motorRight.setPower(0.1);
            }

            idle();

        }

        //stops motors
        motorLeft.setPower(0);
        motorRight.setPower(0);

        idle();

    }
    public void turnGyro(int target) throws InterruptedException{ //counter-clockwise
        //turnGyro turns robot from current position. careful when using this method, small errors in movement can add up!

        turnAbsoluteGyro(target + mrGyro.getIntegratedZValue());
    }
}
