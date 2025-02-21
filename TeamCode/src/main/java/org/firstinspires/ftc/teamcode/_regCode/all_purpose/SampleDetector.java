package org.firstinspires.ftc.teamcode._regCode.all_purpose;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Range;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

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

        Scalar minBlue = new Scalar(200 * NumToH, 30  * SVvalMult, 30 * SVvalMult);; // HSV blue min
        Scalar maxBlue = new Scalar(240 * NumToH, 100 * SVvalMult, 100 * SVvalMult); // HSV blue max

        Core.inRange(hsvMat, minBlue, maxBlue, blueFilter);

        List<MatOfPoint> edges = new ArrayList<>();
        Mat hier = new Mat();

        //Core.

        //Imgproc.findContours(blueFilter, edges, hier, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        
        for (int i = 0; i < edges.size(); i++) {
            Scalar color = new Scalar(100 * NumToH, 100 * SVvalMult, 100 * SVvalMult);
            Imgproc.drawContours(blueFilter, edges, i, color, 2, 1, hier, 0, new Point());
        }

        return blueFilter;
    }

    public void PrintPoints(List<MatOfPoint> points){
        if (points.get(0).get(0, 0)[0] == 0);
    }
}
