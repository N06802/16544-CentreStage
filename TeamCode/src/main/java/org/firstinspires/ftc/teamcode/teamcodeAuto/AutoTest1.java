package org.firstinspires.ftc.teamcode.teamcodeAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.trajectorysequence.TrajectorySequence;


@Autonomous(name = "AutoTest1", group = "test")
public class AutoTest1 extends LinearOpMode {

    public static int TRAJECTORY = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        //SET ORIGINAL POSITION
        Pose2d startPose = new Pose2d(0, -24.00, Math.toRadians(180.00));

        //Be sure to add drive.setPoseEstimate(new Pose2d()) to your opmode
        //before your first motion profiling matching its start pose
        robot.setPoseEstimate(startPose);

        //wierd one
        Trajectory traj1 = robot.trajectoryBuilder(startPose)
                .splineToConstantHeading(new Vector2d(10.69, 13.34), Math.toRadians(270.00))
                .splineToConstantHeading(new Vector2d(33.97, 13.72), Math.toRadians(-88.57))
                .splineToSplineHeading(new Pose2d(34.53, -11.07, Math.toRadians(183.18)), Math.toRadians(183.18))
                .splineToSplineHeading(new Pose2d(-12.58, -10.88, Math.toRadians(179.77)), Math.toRadians(179.77))
                .splineToSplineHeading(new Pose2d(-11.45, 13.34, Math.toRadians(88.61)), Math.toRadians(88.61))
                .splineTo(new Vector2d(10.45, 13.88), Math.toRadians(270))
                .build();

        //circle
        Trajectory traj2 = robot.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(0.00, 24.32), Math.toRadians(0.00))
                .splineTo(new Vector2d(23.75, 0.28), Math.toRadians(264.29))
                .splineTo(new Vector2d(0.28, -23.37), Math.toRadians(180.00))
                .splineTo(new Vector2d(-24.13, -0.00), Math.toRadians(90.00))
                .build();
        //circle
        Trajectory traj3 = robot.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-17.00, -17.00), Math.toRadians(135.00))
                .splineTo(new Vector2d(-24.00, -0.00), Math.toRadians(90.00))
                .splineTo(new Vector2d(-17.36, 17.00), Math.toRadians(45.00))
                .splineTo(new Vector2d(-0.20, 24.00), Math.toRadians(0.00))
                .splineTo(new Vector2d(16.77, 17.00), Math.toRadians(-45.00))
                .splineTo(new Vector2d(24.00, -0.00), Math.toRadians(270.00))
                .splineTo(new Vector2d(17.00, -17.00), Math.toRadians(225.00))
                .splineTo(new Vector2d(0.00, -24.00), Math.toRadians(180.00))
                .build();

        //square
        TrajectorySequence traj4 = robot.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-23.80, -24.00))
                .lineToConstantHeading(new Vector2d(24.00, -24.00))
                .lineToConstantHeading(new Vector2d(23.80, 24.00))
                .lineToConstantHeading(new Vector2d(-24.00, 24.00))
                .build();

        DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");

        waitForStart();

        switch (TRAJECTORY) {
            case 1:
                robot.followTrajectoryAsync(traj1);
                break;
            case 2:
                robot.followTrajectoryAsync(traj2);
                break;
            case 3:
                robot.followTrajectoryAsync(traj3);
                break;
            case 4:
                robot.followTrajectorySequenceAsync(traj4);
                break;
        }

        robot.update();

        telemetry.addData("LF current", leftFront.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("LB current", leftBack.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("RF current", rightFront.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("RB current", rightBack.getCurrent(CurrentUnit.AMPS));



    }


}
