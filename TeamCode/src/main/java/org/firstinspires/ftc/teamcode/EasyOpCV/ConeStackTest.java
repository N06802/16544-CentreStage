package org.firstinspires.ftc.teamcode.EasyOpCV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class ConeStackTest extends LinearOpMode {
    OpenCvCamera camera;

    @Override
    public void runOpMode() throws InterruptedException {
//Before you can obtain a camera instance from the camera factory,
        //you must decide whether or not you wish to have a live camera
        //preview displayed on the Robot Controller screen. If so, you can
        //obtain the view ID for the camera monitor container in exactly
        //the same way as is done for Vuforia.
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //Note that even if you choose to use a live viewport, you can programmatically
        //pause it to reduce CPU/battery load using
        //camera.pauseViewport(); and webcam.resumeViewport();

        //To create an OpenCvWebcam instance:
        //The first parameter specifies the WebcamName of the webcam you wish to use.
        //The second (optional) parameter specifies the view ID in which to insert the live preview.
        //a WebcamName can be obtained from the hardwareMap like so:
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "NAME_OF_CAMERA_IN_CONFIG_FILE");

        //Once you've obtained the WebcamName, you can proceed to using the camera factory:
        // With live preview
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        // Without a live preview
        //OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);

        //Now that you've obtained an OpenCvCamera instance from the camera factory,
        //the next step is to open the connection to the camera device.
        //There are two methods for doing this:

        //Asynchronously (recommended):

        //When opening asynchronously, your thread is not blocked. Instead, you provide a callback
        //in order to be notified when the opening process has been completed. Usually it is in
        //this callback that you'll want to start streaming from the camera

    camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
        @Override public void onOpened() {
        // Usually this is where you'll want to start streaming from the camera

            //After opening the camera, you can start a streaming session by
            //calling camera.startStreaming(...). The first two parameters are
            //the desired width and height of the image stream.
            //The third parameter specifies the orientation the camera is being used in.
            camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
    }
        @Override public void onError(int errorCode)
        {
            /*
             * This will be called if the camera could not be opened
             */
       }
    });

        //Synchronously (not recommended)

        // When opening synchronously, your thread is blocked until the operation is complete.
        //camera.openCameraDevice();
    }
}
