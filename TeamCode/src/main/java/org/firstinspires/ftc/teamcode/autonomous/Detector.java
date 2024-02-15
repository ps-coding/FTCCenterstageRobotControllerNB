package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

class Detector extends OpenCvPipeline {
    private Rect left;
    private Rect right;

    private Mat leftMat;
    private Mat rightMat;

    public double leftPct;
    public double rightPct;

    private Scalar
            RED = new Scalar(255, 0, 0),
            LOW_RED = new Scalar(100, 50, 50),
            HIGH_RED = new Scalar(130, 255, 200),
            LOW_BLUE = new Scalar(0, 50, 30),
            HIGH_BLUE = new Scalar(20, 255, 200);

    private Telemetry telemetry;
    private boolean red;

    private CameraRegionDetected region = null;

    public Detector(Telemetry telemetry, boolean red) {
        this.telemetry = telemetry;
        this.red = red;
    }

    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_BGR2HSV);

        left = new Rect(new Point(0, 0), new Size(input.width() * 0.45, input.height()));
        right = new Rect(new Point(input.width() * 0.45, 0), new Size(input.width() * 0.55, input.height()));

        leftMat = input.submat(left);
        rightMat = input.submat(right);

        if (red) {
            Core.inRange(leftMat, LOW_RED, HIGH_RED, leftMat);
            Core.inRange(rightMat, LOW_RED, HIGH_RED, rightMat);
        } else {
            Core.inRange(leftMat, LOW_BLUE, HIGH_BLUE, leftMat);
            Core.inRange(rightMat, LOW_BLUE, HIGH_BLUE, rightMat);
        }

        leftPct = Core.sumElems(leftMat).val[0] * 1.35 / left.area() / 255;
        rightPct = Core.sumElems(rightMat).val[0] * 1.65 / right.area() / 255;

        Imgproc.cvtColor(input, input, Imgproc.COLOR_HSV2BGR);

        if (leftPct + rightPct < 0.1 && rightPct < 0.05) {
            region = null;
        } else if (leftPct > rightPct) {
            region = CameraRegionDetected.LEFT;
            Imgproc.rectangle(input, left, RED, 3);
            Imgproc.rectangle(input, right, LOW_BLUE, 3);
        } else {
            region = CameraRegionDetected.RIGHT;
            Imgproc.rectangle(input, left, LOW_BLUE, 3);
            Imgproc.rectangle(input, right, RED, 3);
        }

        leftMat.release();
        rightMat.release();

        return input;
    }

    public CameraRegionDetected getRegion() {
        return region;
    }

    public enum CameraRegionDetected {
        LEFT,
        MIDDLE,
        RIGHT
    }
}
