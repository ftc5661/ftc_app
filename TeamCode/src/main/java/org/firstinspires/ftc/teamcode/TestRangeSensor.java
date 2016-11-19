package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by 5661 on 10/31/2016.
 * Autonomous program for testing the Modern Robotics Range Sensor on the 5661 programming robot
 */

@Autonomous(name = "TestRangeSensor", group = "Autonomous OpMode")
@Disabled
public class TestRangeSensor extends LinearOpMode {

    //declares motors, servos, and other data
    DcMotor motorRight;
    DcMotor motorLeft;
    ModernRoboticsI2cRangeSensor rangeSensor;
    static int RangeReadingRaw;
    static double RangeReadingLinear;
    //NOTE: For AndyMark NeveRest motors, 1120 is one revolution, which is 9.42inches/23.9268cm in distance
    int encoderTicks = 1120;
    double wheelCircumference = 23.9268;

    @Override
    public void runOpMode() throws InterruptedException {

        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_sensor");
        //set motor names
        motorRight = hardwareMap.dcMotor.get("right_motor");
        motorLeft = hardwareMap.dcMotor.get("left_motor");
        //reverse right motor
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        //DriveWithWall(90);

        while (opModeIsActive()) {
            telemetry.addData("raw ultrasonic", rangeSensor.rawUltrasonic());
            telemetry.addData("raw optical", rangeSensor.rawOptical());
            telemetry.addData("cm optical", "%.2f cm", rangeSensor.cmOptical());
            telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();


            //motorLeft.setPower(0.033 * (rangeSensor.rawUltrasonic()) - 0.326); //motor speed of 0.4?
            //motorRight.setPower(-0.033 * (rangeSensor.rawUltrasonic()) + 0.726);

            motorLeft.setPower(-0.0083 * (rangeSensor.rawUltrasonic()) - 0.1); //motor speed of 0.4?
            motorRight.setPower(0.0083 * (rangeSensor.rawUltrasonic()) - 0.4);


            idle();
        }

    }

    public void DriveWithWall(int distance){

        //motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //distance in cm divided by the wheel circumference times motor encoder ticks
        //double distanceSet = ((distance/(wheelCircumference))* encoderTicks);
        //set target position
        //motorLeft.setTargetPosition ((int)distanceSet);
        //motorRight.setTargetPosition ((int)distanceSet);

        //motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        motorLeft.setPower(0.03636363636 * (rangeSensor.rawUltrasonic()) - 0.19090909090);
        motorRight.setPower(-0.03636363636 * (rangeSensor.rawUltrasonic()) + 0.79090909090);

        /*while(!isStopRequested() && motorLeft.isBusy() && motorRight.isBusy()){
            //wait until target position is reached
            telemetry.addData("RangeReadingLinear:", rangeSensor.rawUltrasonic());
            telemetry.update();
        }*/

        //motorLeft.setPower(0);
        //motorRight.setPower(0);

        //motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

}
