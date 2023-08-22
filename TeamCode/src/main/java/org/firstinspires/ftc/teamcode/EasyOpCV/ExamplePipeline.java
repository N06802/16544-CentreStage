package org.firstinspires.ftc.teamcode.EasyOpCV;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

// A pipeline is essentially an encapsulation of OpenCV image processing to do a certain thing.
// Most of the time, image processing requires operations to be done in series instead of in
// parallel; outputs from step A are fed into the inputs of step B, and outputs of step B are fed
// into step C, and so on; hence, the term "Pipeline."

// All pipelines are required to extend OpenCvPipeline and implement
// the public Mat processFrame(Mat input) method.

public class ExamplePipeline extends OpenCvPipeline {

    // Mat objects are used to store the value of an image. It represents
    // an n-dimensional array and is used to store image data of grayscale
    // or colour images, voxel volumes, vector fields, point clouds, tensors, histograms, etc.

    // Ideally you should create any auxilliary Mat objects your
    // pipeline will need as instance variables of your pipeline
    // class instead of as local variables in the processFrame(...)
    // function. By doing so, you eliminate the possibility of forgetting
    // to call .release() on Mats and thus iteratively leaking memory on
    // each frame (which if allowed to go unchecked will cause an OOME and crash the Robot Controller app).

    // Notice this is declared as an instance variable (and re-used), not a local variable
    Mat grey = new Mat();

    @Override
    public Mat processFrame(Mat input)
    {
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGB2GRAY);
        return grey;
    }
}
