package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CSRobot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="DetectorAutoPlaceFromBackRight", group = "PlaceDetector")
public class DetectorAutoPlaceFromBackRight extends LinearOpMode {
    OpenCvCamera cam;
    WebcamName webcamName;
    CSRobot robot = new CSRobot();
    DetectorAutoPlaceFromBackLeft.RegionDetected regionDetected;

    @Override
    public void runOpMode() throws InterruptedException {
        webcamName = hardwareMap.get(WebcamName.class, "WobotCam");
        cam = OpenCvCameraFactory.getInstance().createWebcam(webcamName);

        Detector detector = new Detector(telemetry, true);

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


        robot.init(hardwareMap);
        robot.boxDoor.setPosition(1.0);

        while (!isStarted()) {
            if (detector.getRegion() == Detector.CameraRegionDetected.LEFT) {
                regionDetected = DetectorAutoPlaceFromBackLeft.RegionDetected.LEFT;
            } else if (detector.getRegion() == Detector.CameraRegionDetected.RIGHT) {
                regionDetected = DetectorAutoPlaceFromBackLeft.RegionDetected.MIDDLE;
            } else {
                regionDetected = DetectorAutoPlaceFromBackLeft.RegionDetected.RIGHT;
            }

            telemetry.addData("Region: ", regionDetected.toString());
            telemetry.update();
        }

        if (detector.getRegion() == Detector.CameraRegionDetected.LEFT) {
            regionDetected = DetectorAutoPlaceFromBackLeft.RegionDetected.LEFT;
        } else if (detector.getRegion() == Detector.CameraRegionDetected.RIGHT) {
            regionDetected = DetectorAutoPlaceFromBackLeft.RegionDetected.MIDDLE;
        } else {
            regionDetected = DetectorAutoPlaceFromBackLeft.RegionDetected.RIGHT;
        }

        telemetry.addData("Region: ", regionDetected.toString());
        telemetry.update();

        // Place on spike mark
        int DISTANCE_TO_SPIKE = 16;

        robot.driveToInches(-DISTANCE_TO_SPIKE);
        if (regionDetected == DetectorAutoPlaceFromBackLeft.RegionDetected.LEFT) {
            robot.turn(90);
        } else if (regionDetected == DetectorAutoPlaceFromBackLeft.RegionDetected.RIGHT) {
            robot.turn(-90);
        }
        robot.rollWheel.setPower(-0.5);
        ElapsedTime rollWait = new ElapsedTime();
        while (rollWait.milliseconds() <= 600) {
            Thread.yield();
        }
        robot.rollWheel.setPower(0.0);

        if (regionDetected == DetectorAutoPlaceFromBackLeft.RegionDetected.LEFT) {
            robot.turn(-90);
        } else if (regionDetected == DetectorAutoPlaceFromBackLeft.RegionDetected.RIGHT) {
            robot.turn(90);
        }

        robot.driveToInches(DISTANCE_TO_SPIKE - 1);

        robot.turn(-90);

        // Place on board
        robot.driveToInches(74);

        if (regionDetected == DetectorAutoPlaceFromBackLeft.RegionDetected.LEFT) {
            robot.tinyStrafe(-7);
        } else if (regionDetected == DetectorAutoPlaceFromBackLeft.RegionDetected.MIDDLE) {
            robot.tinyStrafe(-6);
        } else {
            robot.tinyStrafe(-5);
        }

        robot.mainArm.setPosition(0.4);

        robot.driveToInches(13);

        robot.boxDoor.setPosition(0.3);

        ElapsedTime wait = new ElapsedTime();
        while (wait.milliseconds() <= 1000) {
            Thread.yield();
        }

        robot.driveToInches(-5);

        robot.mainArm.setPosition(0.0);
        ElapsedTime endGame = new ElapsedTime();
        while (endGame.milliseconds() <= 750) {
            Thread.yield();
        }
    }

    enum RegionDetected {
        LEFT,
        MIDDLE,
        RIGHT
    }
}
