package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CSRobot;

@Autonomous(name="AutoPlaceFromFrontRight", group = "PlaceBasic")
public class AutoPlaceFromFrontRight extends LinearOpMode {
    CSRobot robot = new CSRobot();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized.");
        telemetry.update();
        robot.init(hardwareMap);

        waitForStart();

        robot.boxDoor.setPosition(1.0);
        robot.mainArm.setPosition(0.4);

        Thread.sleep(2000);

        telemetry.addData("Status", "Driving...");
        telemetry.update();

        robot.tinyStrafe(-1);

        robot.driveToInches(16);
        robot.tinyStrafe(-6);

        robot.mainArm.setPosition(0.8);

        ElapsedTime wait = new ElapsedTime();
        while (wait.milliseconds() < 1100) {
            robot.extensionArm.setPower(-0.6 / (Math.max(1, (Math.exp(Math.abs(robot.extensionArm.getCurrentPosition() / 13))))));
        }
        robot.extensionArm.setPower(0.0);

        robot.driveToInches(15);
        try {Thread.sleep(1000);} catch (InterruptedException e) {}
        robot.boxDoor.setPosition(0.0);
        try {Thread.sleep(2000);} catch (InterruptedException e) {}
        robot.mainArm.setPosition(0.0);
        try {
            Thread.sleep(900);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        robot.extensionArm.setPower(-0.1);
        try {
            Thread.sleep(750);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        robot.extensionArm.setPower(0.0);
    }
}
