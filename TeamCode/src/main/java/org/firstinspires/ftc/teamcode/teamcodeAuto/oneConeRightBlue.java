package org.firstinspires.ftc.teamcode.teamcodeAuto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.EasyOpCV.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.StackDetection.StackDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Disabled
@Autonomous
public class oneConeRightBlue extends LinearOpMode {


    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    private DcMotorEx leftFront, leftBack, rightBack, rightFront;

    static final double FEET_PER_METER = 3.28084;

    /*// Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;*/
    // NOTE: this is the calibration for the c270 webcam at 1280x720
    double fx = 578.272;
    double fy = 578.272;
    double cx = 640;
    double cy = 360;

    // UNITS ARE METERS
    double tagsize = 0.04;

    int ID_TAG_POS_LEFT = 4; // Tag ID 4 from the 36h11 family
    int ID_TAG_POS_CENTRE = 7; // Tag ID 7 from the 36h11 family
    int ID_TAG_POS_RIGHT = 11; // Tag ID 11 from the 36h11 family
    int tagPos = 0;

    AprilTagDetection tagOfInterest = null;

    private DistanceSensor Backdistance;

    @Override
    public void runOpMode() throws InterruptedException {


        //INITIALIZE HARDWAREMAP
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);

        //robot.resetIMU();
        robot.initTurret();

        //SET ORIGINAL POSITION
        Pose2d startBlueLeft = new Pose2d(-36, 55, Math.toRadians(-90));
        robot.setPoseEstimate(startBlueLeft);


        //CREATE TRAJECTORIES
        Trajectory traj1 = robot.trajectoryBuilder(startBlueLeft)
                // TODO: fix so there is no strafing to the right
                .lineToLinearHeading(new Pose2d(-38, 26, Math.toRadians(270))) //x should be 36
                .splineTo(new Vector2d(-36, 8), Math.toRadians(178))
                .build();
        //Strafes left
        /*Trajectory traj2 = robot.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(37, 18), Math.toRadians(-8))
                .build();*/
        //strafes right
        Pose2d startConeStack = traj1.end();
        Trajectory parkCentre = robot.trajectoryBuilder(startConeStack)
                .lineToLinearHeading(new Pose2d(6,0))
                .build();

        /*
        //This section is prototype for two-cone pickup
        Trajectory back2 = robot.trajectoryBuilder(traj3.end())
                .back(2)
                .build();
        Trajectory traj4 = robot.trajectoryBuilder(back2.end())
                .forward(18)
                .build();
        Trajectory traj5 = robot.trajectoryBuilder(new Pose2d())
                .back(30)
                .build();
        Trajectory traj6 = robot.trajectoryBuilder(traj1.end())
                .strafeLeft(6)
                .build();
        Trajectory traj7 = robot.trajectoryBuilder(traj2.end())
                .strafeRight(7)
                .build();
         */

        Trajectory parkLeft = robot.trajectoryBuilder(parkCentre.end())
                .lineToLinearHeading(new Pose2d(55, 7, Math.toRadians(0)))
                .build();
        Trajectory parkRight = robot.trajectoryBuilder(parkCentre.end())
                .lineToLinearHeading(new Pose2d(8, 7, Math.toRadians(0)))
                .build();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        StackDetectionPipeline detector = new StackDetectionPipeline(telemetry);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        if(opModeInInit()){
            robot.init2();
        }
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == ID_TAG_POS_LEFT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        tagPos = 1;
                        break;
                    } else if (tag.id == ID_TAG_POS_CENTRE) {
                        tagOfInterest = tag;
                        tagFound = true;
                        tagPos = 2;
                        break;
                    } else if (tag.id == ID_TAG_POS_RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        tagPos = 3;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }
            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        camera.closeCameraDevice();
        camera.setPipeline(detector);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        detector.getColor(StackDetectionPipeline.Color.BLUE);

        /* Actually do something useful */
        if (tagOfInterest == null) {
            /*
             * Insert your autonomous code here, presumably running some default configuration
             * since the tag was never sighted during INIT
             */
            robot.setClaw(true);
            robot.liftDown();
            robot.setArm(0.09);
            robot.followTrajectory(traj1);
            orientToCone(detector, this);
            robot.forwardToStack(this);
            //robot.followTrajectory(traj2);
            //robot.grabCone(0.32);
            robot.turnSlightly(false, true, this);
            robot.placeConeRight();
            robot.setArm(0.09);
            robot.turnForTime(true, 500);
            robot.goBackward(250);
            robot.turnForTime(true, 800);
            robot.goForward(250);
            //robot.turn(15);
            //robot.followTrajectory(parkCentre);
            //sleep(100);
            //robot.grabCone(0.8);
            //sleep(500);

            //robot.moveBack(1000);
            //robot.placeCone();
            //sleep(3000);
            /*robot.followTrajectory(parkCentre);

            if (tagPos == 1) {
                robot.followTrajectory(parkLeft);
            } else if (tagPos == 3) {
                robot.followTrajectory(parkRight);
            }*/
        } else {
            /*
             * Insert your autonomous code here, probably using the tag pose to decide your configuration.
             */

            // e.g.
            robot.setClaw(true);
            robot.liftDown();
            robot.setArm(0.09);
            robot.followTrajectory(traj1);
            orientToCone(detector, this);
            robot.forwardToStack(this);
            //robot.followTrajectory(traj2);
            //robot.grabCone(0.32);
            robot.turnSlightly(false, true, this);
            robot.placeConeRight();
            robot.setArm(0.09);
            robot.turnForTime(true, 500);

            if (tagPos == 3) {
                robot.goForward(700);
                robot.turnForTime(true, 1000);
                robot.goForward(250);
            } else if (tagPos == 1) {
                robot.goBackward(1100);
                robot.turnForTime(true, 800);
                robot.goForward(500);
            } else {
                robot.goBackward(250);
                robot.turnForTime(true, 800);
                robot.goForward(250);
            }
        }
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        /*telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));*/
    }

    private void orientToCone(StackDetectionPipeline detectionPipeline, LinearOpMode that) {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        while (detectionPipeline.getLocation() == StackDetectionPipeline.Location.LEFT && that.opModeIsActive()) {
            leftFront.setPower(0.1);
            leftBack.setPower(0.1);
            rightFront.setPower(0.1);
            rightBack.setPower(0.1);
            sleep(1);
            telemetry.addData("ELEMENT", "LEFT");
            telemetry.update();
        }
        while (detectionPipeline.getLocation() == StackDetectionPipeline.Location.RIGHT && that.opModeIsActive()) {

            leftFront.setPower(-0.1);
            leftBack.setPower(-0.1);
            rightFront.setPower(-0.1);
            rightBack.setPower(-0.1);
            sleep(1);
            telemetry.addData("ELEMENT", "RIGHT");
            telemetry.update();
        }
        telemetry.addData("ELEMENT", "CENTRE");
        telemetry.update();
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}
