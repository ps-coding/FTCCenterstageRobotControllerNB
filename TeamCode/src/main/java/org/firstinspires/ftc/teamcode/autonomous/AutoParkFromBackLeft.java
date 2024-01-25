package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CSRobot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "AutoParkFromBackLeft", group = "Park")
public class AutoParkFromBackLeft extends LinearOpMode {
    OpenCvCamera cam;
    WebcamName webcamName;
    CSRobot robot = new CSRobot();
    Detector.RegionDetected regionDetected;

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

        while (detector.getRegion() == null) {
            Thread.yield();
        }

        regionDetected = detector.getRegion();

        telemetry.addData("Region: ", regionDetected.toString());
        telemetry.update();

//        robot.boxDoor.setPosition(1.0);
//        robot.mainArm.setPosition(0.4);

        robot.tinyStrafe(1);

        robot.driveToInches(10); //

        robot.mainArm.setPosition(0.0);
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}
