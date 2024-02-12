package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CSRobot;

@Autonomous(name="AutoPlaceFromFrontLeft", group = "PlaceBasic")
public class AutoPlaceFromFrontLeft extends LinearOpMode {
    CSRobot robot = new CSRobot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.boxDoor.setPosition(1.0);
        waitForStart();

        robot.tinyStrafe(1);

        robot.driveToInches(16);
        robot.tinyStrafe(7);

        robot.mainArm.setPosition(0.45);

        robot.driveToInches(21);

        robot.boxDoor.setPosition(0.3);

        ElapsedTime wait = new ElapsedTime();
        while (wait.milliseconds() <= 1000) {
            Thread.yield();
        }

        robot.driveToInches(-5);
    }
}
