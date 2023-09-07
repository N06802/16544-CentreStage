package org.firstinspires.ftc.teamcode.autoTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Autonomous(name = "CancelableTrajectoryFSM")
public class CancelableTrajectoryFSM extends LinearOpMode {
    enum State {
        TRAJ_1, //Runs trajectory 1
        TRAJ_2,
        TURN_RIGHT,
        IDLE

    }

    //set the current state to IDLE
    //it is the default
    //this ensures that nothing happens until we want it to
    State currentState = State.IDLE;

    //define the starting position
    Pose2d startPose = new Pose2d(-36.41, 59.79 , Math.toRadians(270));

    private DcMotorEx leftFront, leftBack, rightFront, rightBack;
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    public void runOpMode() throws InterruptedException {
        //initializing SampleMecanumDriveCancelable
        SampleMecanumDriveCancelable robot = new SampleMecanumDriveCancelable(hardwareMap);

        //defines all of the motors so the can used during the telemetry readings
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        //adds the telemetry to the dashboard
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

        //sets the robots current position to the starting postion
        robot.setPoseEstimate(startPose);

        TrajectorySequence traj1 = robot.trajectorySequenceBuilder(new Pose2d(-36.41, 59.79, Math.toRadians(270)))
                .splineTo(new Vector2d(-35.59, 25.03), Math.toRadians(-90))
                .splineTo(new Vector2d(9.93, 13.45), Math.toRadians(0.00))
                .build();

        //define the angle at which we want to turn to during TURN_RIGHT
        double turnAngle = -90.0;

        //We create a new pose2d that takes the position at the end of traj1 one and adds the angle to it
        //this is b/c the robot will turn to -90.0 degrees before traj2 during TURN_RIGHT
        Pose2d traj1End = traj1.end().plus(new Pose2d(0,0, turnAngle));

        //this trajectory goes in a straight line downwards to y: -37.86
        TrajectorySequence traj2 = robot.trajectorySequenceBuilder(traj1End)
                .splineTo(new Vector2d(9.93, -37.86), Math.toRadians(-90.00))
                .build();



        telemetry.addLine("Press start when ready.");

        waitForStart();
        
        //Specify the amount of time until the robot will exit the traj
        // and created the timer
        ElapsedTime stopTimer = new ElapsedTime();

        telemetry.addLine("Robot will now exit in the trajectory in 3 seconds.");

        //set the state to TRAJ_1
        //then run the first trajectory
        //it is important that it runs as Async as this is what
        //allows everything else to update in the background
        currentState = State.TRAJ_1;
        robot.followTrajectorySequenceAsync(traj1);

        while (opModeIsActive() && !isStopRequested()) {

            //state machine logic
            //you can have multiple switch statements running together for multiple state machines
            //in parallel. This is the basic idea for subsystems and commands.
            switch (currentState) {
                case TRAJ_1:
                    //check if the robot is running a trajectory
                    //if not it will run code within
                    if (!robot.isBusy()) {
                        //set the state to TURN_RIGHT
                        //and run turnAsync which turns the robot to -90 degrees or 270
                        currentState = State.TURN_RIGHT;
                        robot.turnAsync(turnAngle);
                    }
                    break;
                case TURN_RIGHT:
                    //check if the robot is running a trajectory
                    //if not it will run code within
                    if(!robot.isBusy()) {
                        //set the state to TRAJ_2
                        //and run traj2
                        currentState = State.TRAJ_2;
                        robot.followTrajectorySequenceAsync(traj2);
                    }
                    break;
                case TRAJ_2:
                    //check if the robot is running a trajectory
                    //if not it will run code within
                    if(!robot.isBusy()){
                        //set the state to IDLE
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    //this is the end of the trajectories for the robot to follow
                    //so the robot does nothing in IDLE
                    break;
            }

            if (stopTimer.seconds() >= 1) {
                //set state too idle so it wont run
                //anymore trajectories
                currentState = State.IDLE;

                // Cancel following trajectory
                robot.breakFollowing();

                // Stop the motors
                robot.setDrivePower(new Pose2d());

                //print line on dashboard that all is working
                telemetry.addLine("Robot has exited the trajectory!");
                telemetry.update();
            }

            //updates the robot in the background regardless of state
            robot.update();

            //gets the postion of the robot and saves it to posEstimate
            Pose2d poseEstimate = robot.getPoseEstimate();

            //saves the poseEstimate to PoseStorage allowing us to access the postion data across opmodes
            PoseStorage.currentPose = poseEstimate;

            //since this code always runs in the background the telemetry will show up dashboard
            //which used to not show up until the trajectories finished
            telemetry.addData("LB Current", leftBack.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("LF Current", leftFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("RB Current", rightBack.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("RF Current", rightFront.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}
