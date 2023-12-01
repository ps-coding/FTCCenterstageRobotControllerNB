package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CSRobot;

@Autonomous(name = "AutoParkFromFrontRight", group = "Park")
public class AutoParkFromFrontRight extends LinearOpMode {
    CSRobot robot = new CSRobot();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized.");
        telemetry.update();
        robot.autoInit(hardwareMap);

        waitForStart();

        robot.claw.setPosition(1.0);
        robot.secondaryArm.setPosition(0.4);

        telemetry.addData("Status", "Driving...");
        telemetry.update();

        robot.tinyStrafe(-1);

        robot.driveToInches(46);

        robot.secondaryArm.setPosition(0.0);
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}
