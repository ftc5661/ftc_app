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

        while (mrGyro.isCalibrating()){
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

    public void turnAbsoluteGyro(double deg) throws InterruptedException{
        //Turns robot by using MRGyro mounted on robot
        int zAccumulated = mrGyro.getIntegratedZValue(); //set variables to gyro readings
        double target_angle_degrees = deg;
        double error_degrees = target_angle_degrees - zAccumulated;
        double motor_output = (error_degrees / 180.0) + (error_degrees > 0 ? -.1 : .1);

        while(Math.abs(zAccumulated - deg) > 2){

            if (Math.abs(error_degrees) > 30) {
                motorLeft.setPower(.5 * Math.signum(error_degrees));
                motorRight.setPower(-.5 * Math.signum(error_degrees));
            } else {
                motorLeft.setPower(motor_output);
                motorRight.setPower(-motor_output);
            }
        }
    }
}
