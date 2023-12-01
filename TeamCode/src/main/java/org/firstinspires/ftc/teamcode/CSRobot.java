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

    public DcMotor rootArm;
    public Servo secondaryArm;
    public Servo flyShoot;
    private boolean flyFlew = false;

    private ElapsedTime secondaryArmDebounce = new ElapsedTime();
    private SecondaryArmMode secondaryArmState = SecondaryArmMode.DOWN;
    public Servo claw;
    private ElapsedTime clawDebounce = new ElapsedTime();
    private boolean clawOpen = false;

    private ElapsedTime rootArmTime = new ElapsedTime();

    private ElapsedTime reset = new ElapsedTime();

    public IMU imu;

    public double flDrivePower;
    public double frDrivePower;
    public double brDrivePower;
    public double blDrivePower;
    public double rootArmPower;

    public double multiplier;

    public void init(final HardwareMap hardwareMap) {
        // Initialize hardware map
        flDrive = hardwareMap.get(DcMotor.class, "flDrive");
        frDrive = hardwareMap.get(DcMotor.class, "frDrive");
        blDrive = hardwareMap.get(DcMotor.class, "blDrive");
        brDrive = hardwareMap.get(DcMotor.class, "brDrive");

        frDrive.setDirection(DcMotor.Direction.REVERSE);

        rootArm = hardwareMap.get(DcMotor.class, "rootArm");
        secondaryArm = hardwareMap.get(Servo.class, "secondaryArm");
        claw = hardwareMap.get(Servo.class, "claw");
        flyShoot = hardwareMap.get(Servo.class, "flyShoot");

        // Set up drive motors
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up root arm motor
        rootArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rootArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rootArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up servo motors
        claw.setPosition(1.0);
        secondaryArm.setPosition(0.0);
        flyShoot.setPosition(1.0);

        while (secondaryArm.getPosition() > 0.05) {
            Thread.yield();
        }

        try { Thread.sleep(3000); } catch (InterruptedException error) { }

        rootArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rootArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

    public void autoInit(final HardwareMap hardwareMap) {
        // Initialize hardware map
        flDrive = hardwareMap.get(DcMotor.class, "flDrive");
        frDrive = hardwareMap.get(DcMotor.class, "frDrive");
        blDrive = hardwareMap.get(DcMotor.class, "blDrive");
        brDrive = hardwareMap.get(DcMotor.class, "brDrive");

        frDrive.setDirection(DcMotor.Direction.REVERSE);

        rootArm = hardwareMap.get(DcMotor.class, "rootArm");
        secondaryArm = hardwareMap.get(Servo.class, "secondaryArm");
        claw = hardwareMap.get(Servo.class, "claw");
        flyShoot = hardwareMap.get(Servo.class, "flyShoot");

        // Set up drive motors
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up root arm motor
        rootArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rootArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rootArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up servo motors
        claw.setPosition(1.0);
        flyShoot.setPosition(1.0);

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
        rootArmDrive(gp2);
        secondaryArmDrive(gp2);
        rootArmToggle(gp2);
        toggleClaw(gp2);
        fly(gp2);
    }

    public void mainDrive(Gamepad gp1) {
        multiplier = Math.max(0.3, 1 - gp1.right_trigger);

        final double drive = (-gp1.left_stick_y);
        final double strafe = (gp1.left_stick_x);
        final double turn = (gp1.right_stick_x);

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

    public void rootArmDrive(Gamepad gp2) {
        rootArmPower = gp2.left_stick_y;

        if (rootArmPower <= 0) {
            rootArm.setPower(rootArmPower / (
                    Math.max(1, (Math.exp(Math.abs(rootArm.getCurrentPosition() / 35))))
            ));
        } else {
            rootArm.setPower(rootArmPower / 6);
        }
    }

    public void secondaryArmDrive(Gamepad gp2) {
        if (gp2.right_bumper && reset.milliseconds() > 300) {
            reset.reset();

            secondaryArm.setPosition(0.0);
            secondaryArmState = SecondaryArmMode.DOWN;
        }

        if (gp2.x && secondaryArmDebounce.milliseconds() > 300) {
            secondaryArmDebounce.reset();

            if (secondaryArmState == SecondaryArmMode.UP) {
                secondaryArm.setPosition(0.0);
                try {
                    Thread.sleep(900);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                rootArm.setPower(-0.1);
                try {
                    Thread.sleep(750);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                rootArm.setPower(0.0);
                secondaryArmState = SecondaryArmMode.DOWN;
            } else if (secondaryArmState == SecondaryArmMode.DOWN) {
                secondaryArm.setPosition(0.4);
                secondaryArmState = SecondaryArmMode.LOADED;
            } else if (secondaryArmState == SecondaryArmMode.LOADED) {
                secondaryArm.setPosition(1.0);
                secondaryArmState = SecondaryArmMode.MOMENTUM;
            } else {
                secondaryArm.setPosition(0.8);
                secondaryArmState = SecondaryArmMode.UP;
            }
        }
    }

    public void rootArmToggle(Gamepad gp2) {
        if (gp2.left_bumper && rootArmTime.milliseconds() > 300) {
            rootArmTime.reset();

            ElapsedTime wait = new ElapsedTime();
            while (wait.milliseconds() < 900) {
                rootArm.setPower(-0.5 / (Math.max(1, (Math.exp(Math.abs(rootArm.getCurrentPosition() / 13))))));
            }
            rootArm.setPower(0.0);
        }
    }

    public void toggleClaw(Gamepad gp2) {
        if (gp2.a && clawDebounce.milliseconds() > 300) {
            clawDebounce.reset();

            clawOpen = !clawOpen;

            if (clawOpen) {
                claw.setPosition(0.0);
            } else {
                claw.setPosition(1.0);
            }
        }
    }

    public void fly(Gamepad gp2) {
        if (gp2.y && !flyFlew) {
            flyFlew = true;

            flyShoot.setPosition(0.0);
        }
    }

    public void turnLeft(double target) {
        this.imu.resetYaw();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double currentPosition = this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double error = target - currentPosition;
        double lastError;

        double kp = 0.5;
        double kd = 0.1;

        final int DELAY = 50;

        while (Math.abs(error) > 1) {
            currentPosition = this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            lastError = error;
            error = target - currentPosition;

            double proportional = error * kp;
            double derivative = ((error - lastError) / DELAY) * kd;

            double turn = (proportional + derivative) / (180 * kp);

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
        double lastError;

        double kp = 0.5;
        double kd = 0.1;

        final int DELAY = 50;

        while (Math.abs(error) > 1) {
            currentPosition = -this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            lastError = error;
            error = target - currentPosition;

            double proportional = error * kp;
            double derivative = ((error - lastError) / DELAY) * kd;

            double turn = (proportional + derivative) / (180 * kp);

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

    enum SecondaryArmMode {
        DOWN,
        LOADED,
        MOMENTUM,
        UP
    }
}
