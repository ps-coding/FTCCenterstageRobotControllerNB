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

        telemetry.addData("frPower: ", robot.frDrivePower + "\n");
        telemetry.addData("flPower: ", robot.flDrivePower + "\n");
        telemetry.addData("brPower: ", robot.brDrivePower + "\n");
        telemetry.addData("blPower: ", robot.blDrivePower + "\n");

        telemetry.addData("angle: ", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        telemetry.addData("extensionArmPower: ", robot.extensionArmPower + "\n");
        telemetry.addData("extensionArm: ", robot.extensionArm.getCurrentPosition());

        telemetry.addData("mainArm: ", robot.mainArm.getPosition() + "\n");
//prasham is not good at APUSH
//        telemetry.addData("boxDoor: ", robot.boxDoor.getPosition() + "\n");

        telemetry.addData("flyFlew: ", robot.flyFlew);

        telemetry.addData("rollWheel: ", robot.rollWheel.getCurrentPosition());

//        telemetry.addData("hangRoller: ", robot.hangRoller.getCurrentPosition());

        telemetry.update();
    }
}
