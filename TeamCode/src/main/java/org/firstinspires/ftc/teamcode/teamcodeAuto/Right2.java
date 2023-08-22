package org.firstinspires.ftc.teamcode.teamcodeAuto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.EasyOpCV.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.StackDetection.StackDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.FinalMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(group = "GAMEPLAY")
public class Right2 extends LinearOpMode {

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
    public void runOpMode() throws InterruptedException {

        //INITIALIZE HARDWAREMAP
        FinalMecanumDrive robot = new FinalMecanumDrive(hardwareMap);

        robot.initialize();

        //SET ORIGINAL POSITION
        Pose2d startBlueLeft = new Pose2d(36, 62, Math.toRadians(-90));
        robot.setPoseEstimate(startBlueLeft);

        //CREATE TRAJECTORIES
        Trajectory traj1 = robot.trajectoryBuilder(startBlueLeft)
                // TODO: fix so there is no strafing to the right
                .lineToSplineHeading(new Pose2d(36, 24, Math.toRadians(-90)))
               //
                // .addDisplacementMarker(2, () ->
                ////        robot.setArm(400))//x should be 36
                .splineTo(new Vector2d(42, 9), Math.toRadians(-70))
                .build();
        //Strafes left

        //strafes right
        Trajectory parkCentre = robot.trajectoryBuilder(traj1.end())
                .lineToLinearHeading(new Pose2d(36, 6, Math.toRadians(-180)))
                .build();

        Trajectory traj2 = robot.trajectoryBuilder(parkCentre.end())
                .forward(24)
                .build();

        Trajectory traj3 = robot.trajectoryBuilder(traj2.end())
                .lineToSplineHeading(new Pose2d(30, 10, Math.toRadians(-45)))
                .splineTo(new Vector2d(42, 8), Math.toRadians(-75))
                .build();

        Trajectory parkCentre2 = robot.trajectoryBuilder(traj3.end())
                .lineToLinearHeading(new Pose2d(36, 12, Math.toRadians(0)))
                .build();

        Trajectory parkLeft = robot.trajectoryBuilder(parkCentre2.end())
                .lineToConstantHeading(new Vector2d(60, 12))
                .build();

        Trajectory parkRight = robot.trajectoryBuilder(parkCentre2.end())
                .lineToConstantHeading(new Vector2d(12,12))
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

        /* Actually do something useful */
        if (tagOfInterest == null) {
            /*
             * Insert your autonomous code here, presumably running some default configuration
             * since the tag was never sighted during INIT
             */
            robot.setClaw(false);
            sleep(500);
            robot.setLift(1700);
            robot.setArm(400);
            robot.followTrajectory(traj1);
            sleep(1000);
            robot.setClaw(true);
            sleep(500);
            robot.followTrajectory(parkCentre);
            robot.setLift(0);
            robot.setArm(0);
            sleep(2500);
            robot.orientToStack(false, detector, camera, this);
            robot.followTrajectory(traj2);
            robot.forwardToStack(this);
            robot.setLift(900);
            sleep(1000);
            robot.setClaw(false);
            sleep(500);
            robot.setLift(1700);
            sleep(500);
            robot.setArm(400);
            robot.followTrajectory(traj3);
            sleep(700);
            robot.setClaw(true);
            sleep(300);
            robot.followTrajectory(parkCentre2);
            robot.setArm(100);
            robot.setLift(0);
            sleep(2500);

        } else {
            /*
             * Insert your autonomous code here, probably using the tag pose to decide your configuration.
             */

            robot.setClaw(false);
            sleep(500);
            robot.setLift(1700);
            robot.setArm(400);
            robot.followTrajectory(traj1);
            sleep(700);
            robot.setClaw(true);
            sleep(500);
            robot.followTrajectory(parkCentre);
            robot.setLift(0);
            sleep(1000);
            robot.setArm(50);
            sleep(2000);
            robot.orientToStack(false, detector, camera, this);
            robot.followTrajectory(traj2);
            robot.forwardToStack(this);
            robot.setLift(900);
            sleep(1000);
            robot.setClaw(false);
            sleep(500);
            robot.setLift(1700);
            sleep(500);
            robot.setArm(400);
            robot.followTrajectory(traj3);
            sleep(700);
            robot.setClaw(true);
            sleep(300);
            robot.followTrajectory(parkCentre2);
            robot.setArm(0);
            robot.setLift(0);
            sleep(2500);


            // e.g.

            if (tagPos == 1) {
                robot.followTrajectory(parkLeft);
            } else if (tagPos == 3) {
                robot.followTrajectory(parkRight);
            }
            robot.setLift(0);
            robot.setArm(0);
            sleep(2500);
        }
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));

    }
}
