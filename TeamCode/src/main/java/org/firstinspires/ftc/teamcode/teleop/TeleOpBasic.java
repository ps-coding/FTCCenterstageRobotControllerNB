package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.CSRobot;

@TeleOp(name = "TeleOpBasic", group = "TeleOp")
public class TeleOpBasic extends OpMode {
    CSRobot robot = new CSRobot();

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        robot.gamePadPower(gamepad1, gamepad2);

        telemetry.addData("Door Open? ", Math.round(robot.boxDoor.getPosition()) == 0 ? "Open" : "Closed");
        telemetry.addData("Angle: ", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        telemetry.update();
    }
}
