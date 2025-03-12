package org.firstinspires.ftc.teamcode._regCode.all_purpose;

import com.qualcomm.robotcore.hardware.Servo;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class AIGeneratedSampleDetector extends OpenCvPipeline {
    private double rotationAngle = 0; // Stores detected angle
    private Servo sampleServo; // Servo reference

    private double NumToH = 180.0/355.0;
    private double SVvalMult = 2.55;
    private double preferredArea = 500;

    public AIGeneratedSampleDetector(Servo servo) {
        this.sampleServo = servo; // Pass servo from OpMode
    }

    RotatedRect bestFit = null;
    double largestArea = 0;

    @Override
    public Mat processFrame(Mat input) {

        Mat blueFilter = new Mat();

        Mat hsvMat = new Mat();
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        Scalar minBlue = new Scalar(200 * NumToH, 30  * SVvalMult, 30 * SVvalMult);; // HSV blue min
        Scalar maxBlue = new Scalar(240 * NumToH, 100 * SVvalMult, 100 * SVvalMult); // HSV blue max

        Core.inRange(hsvMat, minBlue, maxBlue, blueFilter);

        Mat gray = new Mat();
        Mat edges = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        Imgproc.Canny(blueFilter, edges, 50, 150);

        // Find contours
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            // Convert contour to RotatedRect
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            RotatedRect rect = Imgproc.minAreaRect(contour2f);
            double currentArea = rect.size.width * rect.size.height;

            if(currentArea >= largestArea && currentArea >= preferredArea){
                bestFit = rect;
                largestArea = currentArea;
            }
        }

        if(bestFit != null){
            // Get the rotation angle
            rotationAngle = bestFit.angle;
            if (bestFit.size.width < bestFit.size.height) {
                rotationAngle += 90; // Adjust to match vertical rectangles
            }

            // Draw the rectangle for visualization
            Point[] boxPoints = new Point[4];
            bestFit.points(boxPoints);
            for (int i = 0; i < 4; i++) {
                Imgproc.line(input, boxPoints[i], boxPoints[(i + 1) % 4], new Scalar(0, 255, 0), 2);
            }
        }

        // Normalize the angle to servo range (0 to 1)
        double servoPosition = (rotationAngle + 90) / 180;
        sampleServo.setPosition(servoPosition);

        // Cleanup
        gray.release();
        edges.release();
        hierarchy.release();

        return input; // Return frame with visualization
    }

    public double getRotationAngle() {
        return rotationAngle;
    }
}


