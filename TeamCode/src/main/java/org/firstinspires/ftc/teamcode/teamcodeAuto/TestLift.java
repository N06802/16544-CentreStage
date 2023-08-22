package org.firstinspires.ftc.teamcode.teamcodeAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.FinalMecanumDrive;

@Autonomous
public class TestLift extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        FinalMecanumDrive robot = new FinalMecanumDrive(hardwareMap);

        robot.initialize();

        waitForStart();

        robot.setLift(900);
        robot.setArm(0);
        sleep(6000);
        robot.setLift(0);
        robot.setArm(0);
        sleep(5000);
    }
}
