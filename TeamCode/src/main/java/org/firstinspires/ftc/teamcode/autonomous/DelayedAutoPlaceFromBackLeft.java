package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CSRobot;

@Autonomous(name="DelayedAutoPlaceFromBackLeft", group = "PlaceDelayed")
public class DelayedAutoPlaceFromBackLeft extends LinearOpMode {
    CSRobot robot = new CSRobot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        robot.boxDoor.setPosition(1.0);
        waitForStart();

        ElapsedTime delay = new ElapsedTime();
        while (delay.milliseconds() <= 10000) {
            Thread.yield();
        }

        robot.tinyStrafe(1);

        robot.driveToInches(74);
        robot.tinyStrafe(5);

        robot.mainArm.setPosition(0.5);

        robot.driveToInches(18);

        robot.boxDoor.setPosition(0.3);

        ElapsedTime wait = new ElapsedTime();
        while (wait.milliseconds() <= 1000) {
            Thread.yield();
        }

        robot.driveToInches(-5);
    }
}
