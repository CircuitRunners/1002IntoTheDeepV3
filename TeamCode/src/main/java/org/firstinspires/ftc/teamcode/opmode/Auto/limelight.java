package org.firstinspires.ftc.teamcode.opmode.Auto;

import com.qualcomm.robotcore.util.Range;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class limelight extends OpenCvPipeline {


    private Scalar lowerBound;
    private Scalar upperBound;
    private Point blockCenter = new Point(-1, -1);
    private double blockAngle = 0;
    private double blockY = 0;
    private List<Rect> detectedBlocks = new ArrayList<>();

    public static double wrap(double value, double min, double max) {
        double range = max - min;
        double wrappedValue = (value - min) % range;
        if (wrappedValue < 0) {
            wrappedValue += range;
        }
        return wrappedValue + min;
    }



//    public void incrementPivotPosition(double step) {
//        setPivotPosition(Range.clip(pivotPosition + step, 0.0, 1.0));
//    }

    public limelight(String color) {
        switch (color.toLowerCase()) {
            case "red":
                lowerBound = new Scalar(0, 150, 100);
                upperBound = new Scalar(10, 255, 255);
                break;
            case "blue":
                lowerBound = new Scalar(105, 180, 80);
                upperBound = new Scalar(130, 255, 255);
                break;
            case "yellow":
                lowerBound = new Scalar(22, 180, 150);
                upperBound = new Scalar(30, 255, 255);
                break;
            default:
                lowerBound = new Scalar(0, 0, 0);
                upperBound = new Scalar(0, 0, 0);
        }


    }

    @Override
    public Mat processFrame(Mat input) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Mat mask = new Mat();
        Core.inRange(hsv, lowerBound, upperBound, mask);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        detectedBlocks.clear();
        double maxArea = 0;
        RotatedRect largestBlock = null;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > 800) { // Adjust to reduce small noise detections
                RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
                detectedBlocks.add(rect.boundingRect());
                if (area > maxArea) {
                    maxArea = area;
                    largestBlock = rect;
                }
            }
        }

        for (Rect rect : detectedBlocks) {
            Imgproc.rectangle(input, rect, new Scalar(255, 0, 0), 2);
        }

        if (largestBlock != null) {
            blockCenter = largestBlock.center;
            blockY = 1200.0 / largestBlock.size.width; // Improved distance approximation
            blockAngle = largestBlock.angle;

            if (largestBlock.size.width < largestBlock.size.height) {
                blockAngle += 90; // Ensure correct angle orientation
            }
        } else {
            blockCenter = new Point(-1, -1);
            blockY = 0;
            blockAngle = 0;
        }

        hsv.release();
        mask.release();
        hierarchy.release();

        return input;
    }

    public Point getBlockCenter() { return blockCenter; }
    public double getBlockAngle() { return blockAngle; }
    public double getBlockY() { return blockY; }
}

