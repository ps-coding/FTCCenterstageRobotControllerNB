package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CSRobot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="DetectorAutoPlaceFromBackLeft", group = "PlaceDetector")
public class DetectorAutoPlaceFromBackLeft extends LinearOpMode {
    OpenCvCamera cam;
    WebcamName webcamName;
    CSRobot robot = new CSRobot();
    Detector.RegionDetected regionDetected;

    @Override
    public void runOpMode() throws InterruptedException {
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcamName = hardwareMap.get(WebcamName.class, "WobotCam");
//        cam = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
//
//        Detector detector = new Detector(telemetry, false);
//
//        cam.setPipeline(detector);
//        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                cam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//            }
//        });


        robot.init(hardwareMap);
        robot.boxDoor.setPosition(1.0);
        waitForStart();
//
//        while (detector.getRegion() == null) {
//            Thread.yield();
//        }
//
//        regionDetected = detector.getRegion();
//
//        telemetry.addData("Region: ", regionDetected.toString());
//        telemetry.update();

        regionDetected = Detector.RegionDetected.LEFT;

        // Place on spike mark
        int DISTANCE_TO_SPIKE = 20;

        robot.driveToInches(-DISTANCE_TO_SPIKE);
        if (regionDetected == Detector.RegionDetected.LEFT) {
            robot.turn(90);
        } else if (regionDetected == Detector.RegionDetected.RIGHT) {
            robot.turn(-90);
        }
        robot.rollWheel.setPower(-1.0);
        ElapsedTime rollWait = new ElapsedTime();
        while (rollWait.milliseconds() <= 500) {
            Thread.yield();
        }
        robot.rollWheel.setPower(0.0);

        if (regionDetected == Detector.RegionDetected.LEFT) {
            robot.turn(-90);
        } else if (regionDetected == Detector.RegionDetected.RIGHT) {
            robot.turn(90);
        }

        robot.driveToInches(DISTANCE_TO_SPIKE - 3);

        robot.turn(-90);

        // Place on board
        robot.driveToInches(74);

        if (regionDetected == Detector.RegionDetected.LEFT) {
            robot.tinyStrafe(2);
        } else if (regionDetected == Detector.RegionDetected.MIDDLE) {
            robot.tinyStrafe(4);
        } else {
            robot.tinyStrafe(6);
        }

        robot.mainArm.setPosition(0.45);

        robot.driveToInches(18);

        robot.boxDoor.setPosition(0.3);

        ElapsedTime wait = new ElapsedTime();
        while (wait.milliseconds() <= 1000) {
            Thread.yield();
        }

        robot.driveToInches(-5);
    }
}
