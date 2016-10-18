package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by 5661 on 10/17/2016.
 */
@Autonomous(name = "TestGyro2", group = "Autonomous OpMode")
public class TestGyro2 extends LinearOpMode {

    //declares motors, servos, and other data
    DcMotor motorRight;
    DcMotor motorLeft;
    GyroSensor sensorGyro;
    ModernRoboticsI2cGyro mrGyro;
    double gyroTurnSpeed = 0.3;

    @Override
    public void runOpMode() throws InterruptedException {

        //set motor names
        motorRight = hardwareMap.dcMotor.get("right_motor");
        motorLeft = hardwareMap.dcMotor.get("left_motor");
        //reverse right motor
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        sensorGyro = hardwareMap.gyroSensor.get("gyro");
        mrGyro = (ModernRoboticsI2cGyro) sensorGyro;

        // start calibrating the gyro.
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        mrGyro.calibrate();
        // DO NOT MOVE SENSOR WHILE BLUE LIGHT IS SOLID

        while (mrGyro.isCalibrating()){
            //Ensure calibration is complete (usually 2 seconds)
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();

        //wait for the game to start(press the play button)
        waitForStart();

        turnAbsoluteGyro(90, gyroTurnSpeed);


    }

    public void turnAbsoluteGyro(int deg, double speed) throws InterruptedException{ //positive values, turn right

        int gyroTarget = mrGyro.getIntegratedZValue() + deg;
        int error = deg;
        int loopnum = 0;
        double power;

        int tolerance = 2; //Two degrees of error is close enough
        double p_gain = 0.1; //Use full power when more than ten degrees

        //keep looping while not close enough to target
        while(Math.abs(error) > tolerance){

            telemetry.addData(">", "Turning robot ", deg, "degees");
            telemetry.update();

            //apply proportional gain to error
            power = Range.clip(error * p_gain, -1, 1);

            //spin based on desired power
            motorLeft.setPower(power);
            motorRight.setPower(-power);
            idle();

            error = gyroTarget - mrGyro.getIntegratedZValue();
            loopnum++;

            //display status
            telemetry.addData("Target: ", gyroTarget);
            telemetry.addData("IZ-Value", mrGyro.getIntegratedZValue());
            telemetry.addData("Power #:", power);
            telemetry.addData("Loop #:", loopnum);
            telemetry.update();

        }

        //done turing, stopping motors
        telemetry.addData(">", "Stopping Gyro turn");
        telemetry.update();
        motorLeft.setPower(0);
        motorRight.setPower(0);
        idle();

    }

    public void turnGyro(int deg, double speed) throws InterruptedException{ //counter-clockwise
        //turnGyro turns robot from current position. careful when using this method, small errors in movement can add up!

        turnAbsoluteGyro(deg + mrGyro.getIntegratedZValue(), speed);
    }
}
