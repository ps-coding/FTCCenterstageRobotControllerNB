package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class CSRobot {
    private DcMotor flDrive;
    private DcMotor frDrive;
    private DcMotor blDrive;
    private DcMotor brDrive;

    public DcMotor extensionArm;


    public Servo mainArm;
    private boolean mainArmUp;
    private ElapsedTime mainArmDebounce = new ElapsedTime();


    public Servo boxDoor;
    private boolean doorOpen = false;
    private ElapsedTime doorDebounce = new ElapsedTime();

    public DcMotor rollWheel;

    public Servo flyShoot;
    public boolean flyFlew = false;

    public DcMotor hangRoller;

    public IMU imu;

    public double flDrivePower;
    public double frDrivePower;
    public double brDrivePower;
    public double blDrivePower;
    public double extensionArmPower;

    public double multiplier;

    public void init(final HardwareMap hardwareMap) {
        // Initialize hardware map
        flDrive = hardwareMap.get(DcMotor.class, "flDrive");
        frDrive = hardwareMap.get(DcMotor.class, "frDrive");
        blDrive = hardwareMap.get(DcMotor.class, "blDrive");
        brDrive = hardwareMap.get(DcMotor.class, "brDrive");

        extensionArm = hardwareMap.get(DcMotor.class, "extensionArm");
        mainArm = hardwareMap.get(Servo.class, "mainArm");
        boxDoor = hardwareMap.get(Servo.class, "boxDoor");
        rollWheel = hardwareMap.get(DcMotor.class, "rollWheel");
        flyShoot = hardwareMap.get(Servo.class, "flyShoot");
        hangRoller = hardwareMap.get(DcMotor.class, "hangRoller");

        // Set up drive motors
        frDrive.setDirection(DcMotor.Direction.REVERSE);
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up extension arm motor
        extensionArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        extensionArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up main servo motors
        mainArm.setPosition(0.0);
        boxDoor.setPosition(0.3);

        // Set up roll wheel motor
        rollWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rollWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rollWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Fly shoot servo motor
        flyShoot.setPosition(1.0);

        // Hang roller motor
        hangRoller.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangRoller.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hangRoller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up the IMU (gyro/angle sensor)
        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                )
        );
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(imuParameters);
    }

    public void gamePadPower(Gamepad gp1, Gamepad gp2) {
        // Core functionality
        mainDrive(gp1);

        // Plugins
        extensionArmControl(gp2);
        mainArmToggle(gp2);
        boxDoorToggle(gp2);
        rollControl(gp2);
        fly(gp2);
        hang(gp2);
    }

    public void mainDrive(Gamepad gp1) {
        multiplier = Math.max(0.3, 1 - gp1.right_trigger);

        final double drive = (-gp1.left_stick_y);
        final double turn = (gp1.left_stick_x);
        final double strafe = (gp1.right_stick_x);

        flDrivePower = (drive + strafe + turn);
        frDrivePower = (drive - strafe - turn);
        blDrivePower = (drive - strafe + turn);
        brDrivePower = (drive + strafe - turn);

        final double SLOWDOWN = 4.0;

        flDrive.setPower(flDrivePower * multiplier / SLOWDOWN);
        frDrive.setPower(frDrivePower * multiplier / SLOWDOWN);
        blDrive.setPower(blDrivePower * multiplier / SLOWDOWN);
        brDrive.setPower(brDrivePower * multiplier / SLOWDOWN);
    }

    public void extensionArmControl(Gamepad gp2) {
        extensionArmPower = Math.abs(gp2.left_stick_y);
        extensionArm.setPower(extensionArmPower);
    }

    public void mainArmToggle(Gamepad gp2) {
        if (gp2.x && mainArmDebounce.milliseconds() > 300) {
            mainArmDebounce.reset();

            mainArmUp = !mainArmUp;

            if (mainArmUp) {
                mainArm.setPosition(0.5);
            } else {
                mainArm.setPosition(0.0);
            }
        }
    }

    public void boxDoorToggle(Gamepad gp2) {
        if (gp2.a && doorDebounce.milliseconds() > 300) {
            doorDebounce.reset();

            doorOpen = !doorOpen;

            if (doorOpen) {
                boxDoor.setPosition(1.0);
            } else {
                boxDoor.setPosition(0.3);
            }
        }
    }

    public void rollControl(Gamepad gp2) {
        rollWheel.setPower(gp2.right_stick_y);
    }

    public void fly(Gamepad gp2) {
        if (gp2.y && !flyFlew) {
            flyFlew = true;

            flyShoot.setPosition(0.0);
        }
    }

    public void hang(Gamepad gp2) {
        double power = gp2.left_trigger - gp2.right_trigger;
        hangRoller.setPower(power);
    }

    public void turnLeft(double target) {
        this.imu.resetYaw();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentPosition = this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double error = target - currentPosition;

        double kp = 0.5;

        final int DELAY = 50;

        while (Math.abs(error) > 1) {
            currentPosition = this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            error = target - currentPosition;

            double proportional = error * kp;

            double turn = proportional / (180 * kp);

            flDrivePower = -turn;
            frDrivePower = turn;
            blDrivePower = -turn;
            brDrivePower = turn;

            flDrive.setPower(flDrivePower / 5);
            frDrive.setPower(frDrivePower / 5);
            blDrive.setPower(blDrivePower / 5);
            brDrive.setPower(brDrivePower / 5);

            try {Thread.sleep(DELAY);} catch (InterruptedException e) {}
        }
    }

    public void turnRight(double target) {
        this.imu.resetYaw();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentPosition = -this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double error = target - currentPosition;

        double kp = 0.5;

        final int DELAY = 50;

        while (Math.abs(error) > 1) {
            currentPosition = -this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            error = target - currentPosition;

            double proportional = error * kp;

            double turn = proportional / (180 * kp);
// big black bedbreaking bombaclad boy breaker  Bolli
            flDrivePower = turn;
            frDrivePower = -turn;
            blDrivePower = turn;
            brDrivePower = -turn;

            flDrive.setPower(flDrivePower / 5);
            frDrive.setPower(frDrivePower / 5);
            blDrive.setPower(blDrivePower / 5);
            brDrive.setPower(brDrivePower / 5);

            try {Thread.sleep(DELAY);} catch (InterruptedException e) {}
        }

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        try {Thread.sleep(500);} catch (InterruptedException e) {}
    }

    public void driveToInches(final double inches) {
        driveTo((int) (inches * (100 / 11.75) * 1.5));
    }

    public void driveTo(final int pos) {
        if (pos == 0) return;

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (Math.abs(pos - flDrive.getCurrentPosition()) > 50) {
            int flDistance = pos - flDrive.getCurrentPosition();
            int frDistance = pos - frDrive.getCurrentPosition();
            int blDistance = pos - blDrive.getCurrentPosition();
            int brDistance = pos - brDrive.getCurrentPosition();

            flDrivePower = (double)flDistance / (double)Math.abs(pos);
            blDrivePower = (double)blDistance / (double)Math.abs(pos);
            frDrivePower = (double)frDistance / (double)Math.abs(pos);
            brDrivePower = (double)brDistance / (double)Math.abs(pos);

            flDrive.setPower(flDrivePower / 5);
            frDrive.setPower(frDrivePower / 5);
            blDrive.setPower(blDrivePower / 5);
            brDrive.setPower(brDrivePower / 5);

            Thread.yield();
        }

        drive(0.0);

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        try {Thread.sleep(500);} catch (InterruptedException e) {}
    }

    public void tinyStrafe(int pow) {
        strafe(48 * pow);
    }

    public void strafe(int pos) {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int flBrPos = pos;
        int frBlPos = -pos;

        while (Math.abs(flBrPos - brDrive.getCurrentPosition()) > 10) {
            int flDistance = flBrPos - flDrive.getCurrentPosition();
            int frDistance = frBlPos - frDrive.getCurrentPosition();
            int blDistance = frBlPos - blDrive.getCurrentPosition();
            int brDistance = flBrPos - brDrive.getCurrentPosition();

            flDrivePower = (double)flDistance / (double)Math.abs(flBrPos);
            blDrivePower = (double)blDistance / (double)Math.abs(frBlPos);
            frDrivePower = (double)frDistance / (double)Math.abs(frBlPos);
            brDrivePower = (double)brDistance / (double)Math.abs(flBrPos);

            flDrive.setPower(flDrivePower / 5);
            frDrive.setPower(frDrivePower / 5);
            blDrive.setPower(blDrivePower / 5);
            brDrive.setPower(brDrivePower / 5);

            Thread.yield();
        }

        drive(0.0);

        try {Thread.sleep(500);} catch (InterruptedException e) {}
    }

    private void setDriveMode(final DcMotor.RunMode mode) {
        flDrive.setMode(mode);
        frDrive.setMode(mode);
        blDrive.setMode(mode);
        brDrive.setMode(mode);
    }

    private void drive(final double pow) {
        flDrive.setPower(pow);
        blDrive.setPower(pow);
        frDrive.setPower(pow);
        brDrive.setPower(pow);
    }
}
