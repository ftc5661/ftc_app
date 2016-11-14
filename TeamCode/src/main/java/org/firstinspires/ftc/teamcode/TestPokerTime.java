package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

/**
 * Created by 5661 on 11/10/2016.
 */
@Autonomous(name = "TestPokerTime", group = "Autonomous OpMode")
public class TestPokerTime extends LinearOpMode {

    CRServo beaconPoker;
    double CRServoStop = 0;
    double CRServoForward = 1;
    double CRServoBackward = -1;

    @Override
    public void runOpMode() throws InterruptedException {

        beaconPoker = hardwareMap.crservo.get("beacon_poker");

        beaconPoker.setPower(CRServoStop);

        waitForStart();

        telemetry.addData("Moving", "123iammovingveryfast");
        telemetry.update();
        beaconPoker.setPower(CRServoForward);
        sleep(3000);
        beaconPoker.setPower(CRServoBackward);
        sleep(1700);
        beaconPoker.setPower(CRServoStop);
        sleep(50);

    }
}
