package org.firstinspires.ftc.teamcode.teamcodeAuto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous(name = "RRtest", group = "auto")
public class RRTest extends LinearOpMode {

    /**
     * This OpMode was created to give a starting point for programmers who are
     * new to RoadRunner, created by team 8367 ACME Robotics. This
     */

    //import Robot Hardware and declare it as a variable
    SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);

    @Override
    public void runOpMode(){

        waitForStart();

        //Define Trajectory
        Trajectory goForwards = robot.trajectoryBuilder(new Pose2d(0,0,0))
                .forward(100)//in inches (to be tested)
                .build();
        //Use Trajectory
        robot.followTrajectory(goForwards);

        //Define Vector Trajectory
        //Vector Trajectories don't care about heading like Pose2d Trajectories do
        //They only care about what position we want to go to
        Trajectory lineToPosition = robot.trajectoryBuilder(new Pose2d(100,0,0))
                .lineTo(new Vector2d(0,0))//this will take us back to the centre of the field
                .build();
        //Use Vector Trajectory
        robot.followTrajectory(lineToPosition);

        //Define Strafing Trajectory
        Trajectory strafe = robot.trajectoryBuilder(new Pose2d(0,0,0))
                .strafeLeft(50)//strafe robot left
                .strafeRight(50)//strafe robot right
                .build();
        //use Strafe Trajectory
        robot.followTrajectory(strafe);

        //Define strafe to point trajectory
        //Again, this is a vector trajectory, so it won't care about heading
        Trajectory strafeToPoint = robot.trajectoryBuilder(new Pose2d(0,0,0))
                .strafeTo(new Vector2d(0,50))//strafe 50 inches in the y direction
                .build();
        //Use Vector Strafe Trajectory
        robot.followTrajectory(strafeToPoint);

        //Define Spline Trajectory
        //Starting in the centre of the field, but at a 90 degree angle
        //Always use Math.toRadians when using Roadrunner; Roadrunner uses radians not degrees
        /*Trajectory splineToPosition = robot.trajectoryBuilder(new Pose2d(0,0,Math.toRadians(90)))
                .splineTo(new Pose2d(50,50,Math.toRadians(90)))
                .build();
        *///Use Spline Trajectory
        //robot.followTrajectory(splineToPosition);
    }
}
