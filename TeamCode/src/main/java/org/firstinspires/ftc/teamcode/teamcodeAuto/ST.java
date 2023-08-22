package org.firstinspires.ftc.teamcode.teamcodeAuto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.FinalMecanumDrive;

@Autonomous(name = "ST", group = "test")
public class ST extends LinearOpMode {

    private DcMotorEx rightFront, rightBack, leftFront, leftBack;
    @Override
    public void runOpMode() throws InterruptedException {
        //import Robot Hardware and declare it
        FinalMecanumDrive robot = new FinalMecanumDrive(hardwareMap);
        initialize();

        // We want to start the bot at x: 10, y: -8, heading: 0 degrees
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));

        //Be sure to add drive.setPoseEstimate(new Pose2d()) to your opmode
        //before your first motion profiling matching its start pose
        robot.setPoseEstimate(startPose);

        Trajectory traj1 = robot.trajectoryBuilder(startPose)
                .back(30)
                .build();

        waitForStart();

        robot.followTrajectory(traj1);

        //robot.followTrajectory(traj1);
    }

    public void initialize(){
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setMotorPwr(double rf, double rb, double lf, double lb, int ms){
        rightFront.setPower(rf);
        rightBack.setPower(rb);
        leftFront.setPower(lf);
        leftBack.setPower(lb);
        sleep(ms);
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        leftBack.setPower(0);
    }
}
