package org.firstinspires.ftc.teamcode.teamcodeAuto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
@Autonomous(name = "RRexamples", group = "examples")
public class RRexamples extends LinearOpMode {

    /**
     * This OpMode was created to give a starting point for programmers who are
     * new to RoadRunner, created by team 8367 ACME Robotics. This class shows
     * off the five different types of Trajectories, and how to define and use
     * each of them. For more instructions on RoadRunner, visit:
     * https://learnroadrunner.com/
     */

    @Override
    public void runOpMode() {
        //import Robot Hardware and declare it
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);

        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
        Pose2d startPose = new Pose2d(10, -8, Math.toRadians(90));

        //Be sure to add drive.setPoseEstimate(new Pose2d()) to your opmode
        //before your first motion profiling matching its start pose
        robot.setPoseEstimate(startPose);

        waitForStart();

        //This is a scalar trajectory, which means it will move the robot forwards
        //or backwards for the distance you desire
        //Define Trajectory
        Trajectory goForwards = robot.trajectoryBuilder(startPose)
                .forward(100)//in inches (to be tested)
                .back(100)
                .build();


        //This is a Vector trajectory, which means that it will take into account heading and
        //distance to the desired point, before executing the movement
        //Define Vector Trajectory
        Trajectory lineToPosition = robot.trajectoryBuilder(goForwards.end())
                .lineTo(new Vector2d(0, 0))//this will take us back to the centre of the field
                .build();


        //Once again, this is a scalar trajectory, which will only move a certain
        //distance, and not to a specified point
        //Define Strafing Trajectory
        Trajectory strafe = robot.trajectoryBuilder(lineToPosition.end())
                .strafeLeft(50)//strafe robot left
                .strafeRight(50)//strafe robot right
                .build();


        //This is another vector trajectory, which will take into account the desired point
        //and not a specified distance
        //Define strafe to point trajectory
        Trajectory strafeToPoint = robot.trajectoryBuilder(strafe.end())
                .strafeTo(new Vector2d(0, 50))//strafe 50 inches in the y direction
                .build();


        /*
         * Spline trajectories are different than scalar and vector trajectories
         * They sort of combine them together to create a complex curve that can be used
         * to go around objects and face certain directions at the end of the trajectory
         */
        //Define Spline Trajectory
        //Starting in the centre of the field, but at a 90 degree angle
        //Always use Math.toRadians when using Roadrunner; Roadrunner uses radians not degrees
        Trajectory splineToPosition = robot.trajectoryBuilder(strafeToPoint.end())
                //The angle at the end of the spline trajectory defines the angle at
                //which the robot will face once it reaches the end of the spline
                .splineTo(new Vector2d(0, 0), Math.toRadians(90))
                .build();

        //The true boolean in the second parameter indicates that you want the bot to follow the path in reverse.
        //This is actually just a shorthand for passing a heading of 180 degrees into the second parameter.
        //The TrajectoryBuilder supports any arbitrary heading in the second parameter
        Trajectory driveBackwards = robot.trajectoryBuilder(splineToPosition.end(), true)
                .splineTo(new Vector2d(36, 36), Math.toRadians(0))
                .build();
        //The code below will tell your bot to follow the entire path at a 90 degree angle
        Trajectory driveSideways = robot.trajectoryBuilder(driveBackwards.end(), Math.toRadians(90))
                .splineTo(new Vector2d(0,0), Math.toRadians(0))
                .build();

        //Define all your trajectories before using them
        //This allows for greater speed and fluidity when the program is running
        //Use Trajectory
        robot.followTrajectory(goForwards);
        //Use Vector Trajectory
        robot.followTrajectory(lineToPosition);

        //Use this function to turn the robot
        //It is defined in SampleMecanumDrive
        robot.turn(Math.toRadians(90));
        robot.turn(Math.toRadians(-270));

        //use Strafe Trajectory
        robot.followTrajectory(strafe);
        //Use Vector Strafe Trajectory
        robot.followTrajectory(strafeToPoint);
        //Use Spline Trajectory
        robot.followTrajectory(splineToPosition);
    }
}