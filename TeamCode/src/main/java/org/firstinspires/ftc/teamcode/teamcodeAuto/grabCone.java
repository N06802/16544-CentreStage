package org.firstinspires.ftc.teamcode.teamcodeAuto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
@Autonomous
public class grabCone extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);

        Trajectory traj1 = robot.trajectoryBuilder(new Pose2d())
                .back(30)
                .build();
        Trajectory traj2 = robot.trajectoryBuilder(traj1.end())
                .strafeLeft(6)
                .build();
        Trajectory traj3 = robot.trajectoryBuilder(traj2.end())
                .strafeRight(8)
                .build();

        waitForStart();
        robot.placeCone();
        robot.setClaw(false);
        robot.placeCone();
        robot.followTrajectory(traj1);
        robot.followTrajectory(traj2);
        robot.placeCone();
        robot.followTrajectory(traj3);
        robot.setClaw(true);
        robot.followTrajectory(traj2);
        robot.placeCone();
    }
}
