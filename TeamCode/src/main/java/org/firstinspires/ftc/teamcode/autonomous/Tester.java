package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CSRobot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

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

        while (detector.getRegion() == null) {
            Thread.yield();
        }

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