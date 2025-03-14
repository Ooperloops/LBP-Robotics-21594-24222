package org.firstinspires.ftc.teamcode._regCode.all_purpose;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.ArrayList;
import java.util.List;

public class SampleDetector extends OpenCvPipeline { // Fix 1: Constructor name matches class name
    private double rotationAngle = 0; // Stores detected angle
    private Servo sampleServo; // Servo reference

    // Fix 2: Adjust HSV scaling for OpenCV (Hue: 0-179, Saturation & Value: 0-255)
    private final double NumToH = 179.0 / 360.0;
    private final double SVvalMult = 2.55;
    private final double preferredArea = 500;

    public SampleDetector(Servo servo) { // Fix 1: Constructor corrected
        this.sampleServo = servo; // Pass servo from OpMode
    }

    private RotatedRect bestFit = null;
    private double largestArea = 0;

    @Override
    public Mat processFrame(Mat input) {
        // Convert image to HSV
        Mat hsvMat = new Mat();
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Fix 3: Correct HSV threshold values
        Scalar minBlue = new Scalar(100, 50, 50); // Adjusted blue detection range
        Scalar maxBlue = new Scalar(130, 255, 255);

        // Apply threshold to extract blue objects
        Mat blueFilter = new Mat();
        Core.inRange(hsvMat, minBlue, maxBlue, blueFilter);

        // Find edges using Canny edge detection
        Mat edges = new Mat();
        Imgproc.Canny(blueFilter, edges, 50, 150);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Fix 4: Reset largestArea before processing each frame
        largestArea = 0;

        for (MatOfPoint contour : contours) {
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            RotatedRect rect = Imgproc.minAreaRect(contour2f);
            double currentArea = rect.size.width * rect.size.height;

            if (currentArea > largestArea && currentArea >= preferredArea) {
                bestFit = rect;
                largestArea = currentArea;
            }
        }

        if (bestFit != null) {
            // Get the rotation angle
            rotationAngle = bestFit.angle;
            if (bestFit.size.width < bestFit.size.height) {
                rotationAngle += 90; // Adjust to match vertical rectangles
            }

            // Draw the detected rectangle on the input frame
            Point[] boxPoints = new Point[4];
            bestFit.points(boxPoints);
            for (int i = 0; i < 4; i++) {
                Imgproc.line(input, boxPoints[i], boxPoints[(i + 1) % 4], new Scalar(0, 255, 0), 2);
            }
        }

        // Normalize the angle to servo range (0 to 1) and set position
        if (sampleServo != null) {
            double servoPosition = (rotationAngle + 90) / 180;
            sampleServo.setPosition(servoPosition);
        }

        // Fix 6: Release unused Mats to prevent memory leaks
        hsvMat.release();
        blueFilter.release();
        edges.release();
        hierarchy.release();

        return input; // Return frame with visualization
    }

    public double getRotationAngle() {
        return rotationAngle;
    }
}