package org.firstinspires.ftc.teamcode._regCode.all_purpose;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Range;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SampleDetector extends OpenCvPipeline {

    private double NumToH = 180.0/355.0;
    private double SVvalMult = 2.55;
    @Override
    public Mat processFrame(Mat input) {
        return FilterBlue(input);
    }

    public Mat FilterBlue(Mat input){
        Mat blueFilter = new Mat();

        Mat hsvMat = new Mat();
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        Scalar minBlue = new Scalar(200 * NumToH, 20  * SVvalMult, 0 * SVvalMult);; // HSV blue min
        Scalar maxBlue = new Scalar(240 * NumToH, 100 * SVvalMult, 100 * SVvalMult); // HSV blue max

        Core.inRange(hsvMat, minBlue, maxBlue, blueFilter);

        //Imgproc.findContours();

        return blueFilter;
    }
}
