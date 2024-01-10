package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CSRobot;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name="Tester", group = "Tester")
public class Tester extends LinearOpMode {
    OpenCvCamera cam;
    CSRobot robot = new CSRobot();
    WebcamName webcamName;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcamName = hardwareMap.get(WebcamName.class, "WobotCam");
        cam = OpenCvCameraFactory.getInstance().createWebcam(webcamName);

        Detector detector = new Detector(telemetry, false);

        cam.setPipeline(detector);
        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        waitForStart();

        switch (detector.getRegion()) {
            case LEFT:
                telemetry.addData("SPOT: ", "LEFT");
                break;
            case MIDDLE:
                telemetry.addData("SPOT: ", "MIDDLE");
                break;
            case RIGHT:
                telemetry.addData("SPOT: ", "RIGHT");
                break;
        }

        telemetry.update();

        Thread.sleep(10000);
    }
}

class Detector extends OpenCvPipeline {
    Rect left;
    Rect middle;
    Rect right;

    Mat leftMat;
    Mat middleMat;
    Mat rightMat;

    Scalar
            RED = new Scalar(255,0,0),
            LOW_RED = new Scalar(100,50,50),
            HIGH_RED = new Scalar(130,255,200),
            LOW_BLUE = new Scalar(0, 50, 30),
            HIGH_BLUE = new Scalar(20, 255, 200);

    Telemetry telemetry;
    boolean red;

    private RegionDetected region = RegionDetected.MIDDLE;

    public Detector(Telemetry telemetry, boolean red) {
        this.telemetry = telemetry;
        this.red = red;
    }

    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_BGR2HSV);

        left = new Rect(new Point(0,0), new Size(input.width() * 0.33, input.height()));
        middle = new Rect(new Point(input.width()*0.33,0), new Size(input.width() * 0.33, input.height()));
        right = new Rect(new Point(input.width()*0.66,0), new Size(input.width() * 0.34, input.height()));

        leftMat = input.submat(left);
        middleMat = input.submat(middle);
        rightMat = input.submat(right);

        if (red) {
            Core.inRange(leftMat, LOW_RED, HIGH_RED, leftMat);
            Core.inRange(middleMat, LOW_RED, HIGH_RED, middleMat);
            Core.inRange(rightMat, LOW_RED, HIGH_RED, rightMat);
        } else {
            Core.inRange(leftMat, LOW_BLUE, HIGH_BLUE, leftMat);
            Core.inRange(middleMat, LOW_BLUE, HIGH_BLUE, middleMat);
            Core.inRange(rightMat, LOW_BLUE, HIGH_BLUE, rightMat);
        }

        double leftPct = Core.sumElems(leftMat).val[0] / left.area() / 255;
        double middlePct = Core.sumElems(middleMat).val[0] / middle.area() / 255;
        double rightPct = Core.sumElems(rightMat).val[0] / right.area() / 255;

        telemetry.addData("LEFT: ", leftPct * 100);
        telemetry.addData("MIDDLE: ", middlePct * 100);
        telemetry.addData("RIGHT: ", rightPct * 100);
        telemetry.update();

        if (leftPct > middlePct && leftPct > rightPct) {
            region = RegionDetected.LEFT;
        } else if (middlePct > rightPct) {
            region = RegionDetected.MIDDLE;
        } else {
            region = RegionDetected.RIGHT;
        }

        Imgproc.rectangle(input, left, RED, 3);
        Imgproc.rectangle(input, middle, RED, 3);
        Imgproc.rectangle(input, right, RED, 3);

        leftMat.release();
        middleMat.release();
        rightMat.release();

        return input;
    }

    public RegionDetected getRegion() {
        return region;
    }

    enum RegionDetected {
        LEFT,
        MIDDLE,
        RIGHT
    }
}