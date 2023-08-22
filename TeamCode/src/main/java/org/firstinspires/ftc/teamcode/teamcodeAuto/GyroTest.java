package org.firstinspires.ftc.teamcode.teamcodeAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
@Autonomous
public class GyroTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        robot.rotateAngle(180, 1, 0.2);
        sleep(1000);
        robot.rotateAngle(0, 1, 0.2);


    }
}
