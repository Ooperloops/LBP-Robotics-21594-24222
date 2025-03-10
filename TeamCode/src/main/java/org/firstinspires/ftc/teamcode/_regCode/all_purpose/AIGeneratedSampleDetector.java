package org.firstinspires.ftc.teamcode._regCode.all_purpose;

import com.qualcomm.robotcore.hardware.Servo;

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

    public AIGeneratedSampleDetector(Servo servo) {
        this.sampleServo = servo; // Pass servo from OpMode
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat gray = new Mat();
        Mat edges = new Mat();
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        // Convert to grayscale and detect edges
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);
        Imgproc.Canny(gray, edges, 50, 150);

        // Find contours
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : contours) {
            // Convert contour to RotatedRect
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            RotatedRect rect = Imgproc.minAreaRect(contour2f);

            // Get the rotation angle
            rotationAngle = rect.angle;
            if (rect.size.width < rect.size.height) {
                rotationAngle += 90; // Adjust to match vertical rectangles
            }

            // Draw the rectangle for visualization
            Point[] boxPoints = new Point[4];
            rect.points(boxPoints);
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


