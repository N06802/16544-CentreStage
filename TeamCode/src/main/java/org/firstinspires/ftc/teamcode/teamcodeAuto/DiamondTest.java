package org.firstinspires.ftc.teamcode.teamcodeAuto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
@Autonomous(name = "DiamondTest", group = "test")
public class DiamondTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);
        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose = new Pose2d(-54, -36, Math.toRadians(0));

        //Be sure to add drive.setPoseEstimate(new Pose2d()) to your opmode
        //before your first motion profiling matching its start pose
        robot.setPoseEstimate(startPose);

        Trajectory traj1 = robot.trajectoryBuilder(startPose, Math.toRadians(0))
                .lineTo(new Vector2d(-50, -36))
                .build();

        Trajectory traj2 = robot.trajectoryBuilder(traj1.end(), Math.toRadians(0))
                .lineTo(new Vector2d(-50, -12))
                .build();

        Trajectory traj3 = robot.trajectoryBuilder(traj2.end(), Math.toRadians(0))
                .lineTo(new Vector2d(-12, -12))
                .build();

        Trajectory traj4 = robot.trajectoryBuilder(traj3.end(), Math.toRadians(0))
                .lineTo(new Vector2d(-12, -62))
                .build();

        Trajectory traj5 = robot.trajectoryBuilder(traj3.end(), Math.toRadians(0))
                .lineTo(new Vector2d(-12, -12))
                .build();

        Trajectory traj6 = robot.trajectoryBuilder(traj3.end(), Math.toRadians(0))
                .lineTo(new Vector2d(-12, -62))
                .build();

        Trajectory traj7 = robot.trajectoryBuilder(traj3.end(), Math.toRadians(0))
                .lineTo(new Vector2d(-12, -12))
                .build();


        waitForStart();

        robot.followTrajectory(traj1);
        robot.followTrajectory(traj2);
        robot.followTrajectory(traj3);
        sleep(1500);
        robot.followTrajectory(traj4);
        sleep(1000);
        robot.followTrajectory(traj5);
        sleep(1000);
        robot.followTrajectory(traj6);
        sleep(1000);
        robot.followTrajectory(traj7);


    }

}
