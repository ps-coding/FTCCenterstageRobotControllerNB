package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.CSRobot;

@Autonomous(name = "AutoClawRaise", group = "Experimental")
public class AutoClawRaise extends LinearOpMode {
    CSRobot robot = new CSRobot();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized.");
        telemetry.update();
        robot.init(hardwareMap);

        waitForStart();

        robot.claw.setPosition(1.0);
        robot.secondaryArm.setPosition(0.4);

        telemetry.addData("Status", "Driving...");
        telemetry.update();

        robot.secondaryArm.setPosition(0.8);
        robot.rootArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rootArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (true) {
            robot.rootArm.setPower((-60 - robot.rootArm.getCurrentPosition()) / 60 / Math.max(1, (Math.exp(Math.abs(robot.rootArm.getCurrentPosition() / 25)))));
            Thread.yield();
        }
    }
}
