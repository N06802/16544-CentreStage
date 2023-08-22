package org.firstinspires.ftc.teamcode.teamcodeAuto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.EasyOpCV.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.StackDetection.StackDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "TTTTEEEESSSSSTTTTT", group = "test")
public class Test extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

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

    @Override
    public void runOpMode() {
        StackDetectionPipeline detector = new StackDetectionPipeline(telemetry);

        //INITIALIZE HARDWAREMAP
        SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);

        //SET ORIGINAL POSITION
        Pose2d startPose = new Pose2d();
        robot.setPoseEstimate(startPose);
        Pose2d startBlueLeft = new Pose2d(36, 62, Math.toRadians(-90));
        robot.setPoseEstimate(startBlueLeft);

        //CREATE TRAJECTORIES
        Trajectory strafeLeft = robot.trajectoryBuilder(startPose)
                .strafeLeft(50)
                .build();


        Trajectory goForward1 = robot.trajectoryBuilder(startBlueLeft)
                .lineTo(new Vector2d(36, 24))
                .splineTo(new Vector2d(36, 7), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    while (detector.getLocation() == StackDetectionPipeline.Location.RIGHT) {
                        robot.turnLeft(0.2);
                    }
                    while (detector.getLocation() == StackDetectionPipeline.Location.LEFT) {
                        robot.turnRight(0.2);
                    }
                    robot.setMotorPowers(0, 0, 0, 0);
                })
                .build();

        Trajectory traj1 = robot.trajectoryBuilder(goForward1.end())
                .splineTo(new Vector2d(48, 7), Math.toRadians(0))
                .build();
        Trajectory traj2 = robot.trajectoryBuilder(traj1.end())
                .lineToSplineHeading(new Pose2d(42, 7, Math.toRadians(15)))
                .splineToConstantHeading(new Vector2d(30, 6), Math.toRadians(15))
                .build();

        Trajectory strafeRight = robot.trajectoryBuilder(startPose)
                .strafeRight(50)
                .build();

        /*Trajectory sequence = robot.trajectoryBuilder(startBlueLeft)
                .forward(2)
                .strafeTo(new Vector2d(12, 60))
                .lineTo(new Vector2d(12, 12))
                .strafeTo(new Vector2d(48, 12))
                .strafeTo(new Vector2d(12, 12))
                .strafeTo(new Vector2d(48, 12))
                .strafeTo(new Vector2d(12, 12))
                .strafeTo(new Vector2d(48, 12))
                .strafeTo(new Vector2d(12, 12))
                .build();*/

        Trajectory traj3 = robot.trajectoryBuilder(traj2.end())
                .lineTo(new Vector2d(12, 12))
                //.forward(48)
                .build();
        Trajectory traj4 = robot.trajectoryBuilder(traj3.end())
                .strafeTo(new Vector2d(48, 12))
                //.strafeRight(36)
                .build();
        Trajectory traj5 = robot.trajectoryBuilder(traj4.end())
                .strafeTo(new Vector2d(12, 12))
                //.strafeLeft(36)
                .build();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
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

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if (tagOfInterest == null) {
            robot.followTrajectory(goForward1);
            sleep(1000);
            robot.followTrajectory(traj1);
            robot.followTrajectory(traj2);
            /*
             * Insert your autonomous code here, presumably running some default configuration
             * since the tag was never sighted during INIT
             */
        } else {
            /*
             * Insert your autonomous code here, probably using the tag pose to decide your configuration.
             */

            // e.g.
            robot.followTrajectory(goForward1);
        }
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));

    }

}
