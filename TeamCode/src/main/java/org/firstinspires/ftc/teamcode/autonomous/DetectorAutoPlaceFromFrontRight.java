package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CSRobot;

@Autonomous(name="DetectorAutoPlaceFromFrontRight", group = "PlaceDetector")
public class DetectorAutoPlaceFromFrontRight extends LinearOpMode {
    //    OpenCvCamera cam;
//    WebcamName webcamName;
    CSRobot robot = new CSRobot();
//    Detector.RegionDetected regionDetected;

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
//

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

        robot.tinyStrafe(-1);

        robot.driveToInches(16);
        robot.tinyStrafe(-5);

        robot.mainArm.setPosition(0.5);

        robot.driveToInches(19);

        robot.boxDoor.setPosition(0.3);

        ElapsedTime wait = new ElapsedTime();
        while (wait.milliseconds() <= 1000) {
            Thread.yield();
        }

        robot.driveToInches(-5);
    }
}
