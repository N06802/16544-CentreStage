package org.firstinspires.ftc.teamcode.StackDetection;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp
public class StackDetectionExampleMovement extends LinearOpMode {

    OpenCvCamera camera;



    @Override
    public void runOpMode() throws InterruptedException {



        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId",
                        "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        StackDetectionPipeline detector = new StackDetectionPipeline(telemetry);

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

        waitForStart();

        while (opModeIsActive()) {
            while (detector.getLocation() == StackDetectionPipeline.Location.LEFT) {
                telemetry.addData("ELEMENT", "LEFT");
                telemetry.update();
                setMotorPow(0.2,0.2,-0.2, -0.2);
                sleep(1);

            }
            while (detector.getLocation() == StackDetectionPipeline.Location.RIGHT) {
                telemetry.addData("ELEMENT", "RIGHT");
                telemetry.update();
                setMotorPow(-0.2,-0.2,0.2, 0.2);
                sleep(1);
            }
            while (detector.getLocation() == StackDetectionPipeline.Location.CENTRE) {
                telemetry.addData("ELEMENT", "CENTRE");
                telemetry.update();
                sleep(1);

            }
        }

    }

    private void setMotorPow(double lf, double lb, double rf, double rb){
        DcMotorEx leftFront, leftBack, rightBack, rightFront;

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setPower(lf);
        leftBack.setPower(lb);
        rightFront.setPower(rf);
        rightBack.setPower(rb);
    }
}
