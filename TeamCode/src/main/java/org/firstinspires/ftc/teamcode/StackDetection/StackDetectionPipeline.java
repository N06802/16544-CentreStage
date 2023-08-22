package org.firstinspires.ftc.teamcode.StackDetection;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class StackDetectionPipeline extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();

    public enum Location {
        LEFT,
        CENTRE,
        RIGHT,
        NOT_FOUND
    }

    public enum Color {
        BLUE,
        RED,
        YELLOW
    }

    private Location location;
    private Color colorChoice;

    // A Rect in OpenCV can be defined using 2 opposite vertices of a
    // triangle, which are connected by the diagonals
    // We need to create Regions Of Interest where the camera will be
    // looking for the items colors being searched for
    static final Rect LEFT_ROI = new Rect(new Point(10, 35), new Point(340, 400));
    static final Rect CENTRE_ROI = new Rect(new Point(350,35), new Point(450,400));
    static final Rect RIGHT_ROI = new Rect(new Point(460, 35), new Point(790, 400));

    // define the threshold
    static double PERCENT_COLOR_THRESHOLD = 0.025;

    public StackDetectionPipeline(Telemetry t) {
        telemetry = t;
    }

    public Mat processFrame(Mat input) {
        //Convert from RGB to HSV
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = null;
        Scalar highHSV = null;

        if(colorChoice == Color.BLUE) {
            //Create a range for the camera to detect blue
            lowHSV = new Scalar(110, 50, 50);
            highHSV = new Scalar(130, 255, 255);
        } else if(colorChoice == Color.RED){
            lowHSV = new Scalar(0, 50, 70);
            highHSV = new Scalar(20, 255, 255);
        } else if(colorChoice == Color.YELLOW) {
            //Create a range for the camera to detect yellow
            lowHSV = new Scalar(20, 100, 100);
            highHSV = new Scalar(30, 255, 255);
        }

        //Apply thresholding, which outlines the area of the image that is of the colour you want
        //In this case, OpenCV will change the colours being searched for to white, while the rest
        //will be black/greyscale
        Core.inRange(mat, //source matrix
                lowHSV, highHSV, //lower and upper boundraries
                mat //destination matrix
        );

        // create the submats
        // submat = submatrix, which is a portion of the original matrix
        Mat left = mat.submat(LEFT_ROI);
        Mat centre = mat.submat(CENTRE_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        //check percentage of the image that became white
        // We do this by adding all the pixels together, dividing by its area,
        // the dividing by 255, which is the max value for a greyscale pixel
        // here we take the first element of the sum result, because there
        // is only one channel in a greyscale image
        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double centreValue = Core.sumElems(centre).val[0] / CENTRE_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        //make sure to release the submatricies
        left.release();
        centre.release();
        right.release();

        // To help with debugging, you can use telemetry to
        // display the values used in the calculation
        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Centre percentage", Math.round(centreValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");

        boolean stoneLeft = ((leftValue > PERCENT_COLOR_THRESHOLD) && (leftValue > rightValue));
        boolean stoneRight = (rightValue > PERCENT_COLOR_THRESHOLD) && (rightValue > leftValue);
        boolean stoneCentre = ((!stoneLeft) && (!stoneRight) && (centreValue > PERCENT_COLOR_THRESHOLD));


        if ((stoneLeft && stoneRight) || (stoneLeft && stoneCentre) || (stoneCentre && stoneRight) || (!stoneCentre && !stoneRight && !stoneLeft)) {
            // not found
            location = Location.NOT_FOUND;
            telemetry.addData("ELEMENT", "Not found");
        } else if (stoneRight) {
            //right
            location = Location.RIGHT;
            telemetry.addData("ELEMENT", "Right");
        } else if (stoneCentre) {
            //centre
            location = Location.CENTRE;
            telemetry.addData("ELEMENT", "Centre");
        } else if(stoneLeft){
            // left
            location = Location.LEFT;
            telemetry.addData("ELEMENT", "left");
        }
        telemetry.update();

        //Convert from greyscale to rgb
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorStone = new Scalar(255, 0, 0);
        Scalar colorSkystone = new Scalar(0, 255, 0);

        // use a ternary operator, which is basically a mini if statement to choose
        // which color to use based on the loction of the skystone
        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT ? colorSkystone : colorStone);
        Imgproc.rectangle(mat, CENTRE_ROI, location == Location.CENTRE ? colorSkystone : colorStone);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT ? colorSkystone : colorStone);
        //return matrix so that the image can be drawn
        return mat;
    }

    public void getColor(Color color) {
        colorChoice = color;
    }

    public Location getLocation() {
        return location;
    }
}
