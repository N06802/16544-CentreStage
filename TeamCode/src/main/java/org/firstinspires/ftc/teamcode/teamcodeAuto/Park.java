package org.firstinspires.ftc.teamcode.teamcodeAuto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.EasyOpCV.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.FinalMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(group = "GAMEPLAY")
public class Park extends LinearOpMode {

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

    int ID_TAG_POS_LEFT = 11; // Tag ID 4 from the 36h11 family
    int ID_TAG_POS_CENTRE = 7; // Tag ID 7 from the 36h11 family
    int ID_TAG_POS_RIGHT = 4; // Tag ID 11 from the 36h11 family
    int tagPos = 0;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {

        //INITIALIZE HARDWAREMAP
        FinalMecanumDrive robot = new FinalMecanumDrive(hardwareMap);

        Pose2d startBlueLeft = new Pose2d(36, 62, Math.toRadians(-90));
        robot.setPoseEstimate(startBlueLeft);

        int finalPosY = 36;

        //CREATE TRAJECTORIES
        Trajectory goForward = robot.trajectoryBuilder(startBlueLeft)
                .forward(50)
                .build();
        Trajectory strafeLeft = robot.trajectoryBuilder(goForward.end())
                .strafeLeft(24)
                .build();
        Trajectory strafeRight = robot.trajectoryBuilder(goForward.end())
                .strafeRight(24)
                .build();


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

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

            robot.followTrajectory(goForward);

        } else {
            /*
             * Insert your autonomous code here, probably using the tag pose to decide your configuration.
             */

            // e.g.

            robot.followTrajectory(goForward);

            if (tagPos == 3) {
                // do something
                robot.followTrajectory(strafeLeft);


            } else if (tagPos == 1) {
                robot.followTrajectory(strafeRight);
                // do something else
            }

        }
        sleep(3000);

    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));

    }

}
