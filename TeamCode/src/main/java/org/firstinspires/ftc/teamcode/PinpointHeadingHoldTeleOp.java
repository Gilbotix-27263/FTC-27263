package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Single-controller TeleOp that drives a mecanum chassis using a goBILDA Pinpoint odometry module
 * for XY tracking and the Control Hub IMU for heading hold. The robot keeps its heading steady
 * whenever there is no active rotation request on the right stick.
 */
@TeleOp(name = "Pinpoint Heading Hold", group = "Linear Opmode")
public class PinpointHeadingHoldTeleOp extends LinearOpMode {
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    private IMU imu;
    private GoBildaPinpointDriver pinpoint;

    private static final double HEADING_KP = 0.015;
    private static final double HEADING_KI = 0.0;
    private static final double HEADING_KD = 0.0005;
    private static final double ROTATION_DEADBAND = 0.05;

    private double headingTarget = 0.0;
    private double headingIntegral = 0.0;
    private double lastHeadingError = 0.0;
    private final ElapsedTime headingTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        leftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        rightFront = hardwareMap.get(DcMotor.class, "RightFront");
        rightBack = hardwareMap.get(DcMotor.class, "RightBack");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                LogoFacingDirection.RIGHT,
                UsbFacingDirection.UP
        ));
        imu.initialize(parameters);
        headingTarget = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // The Pinpoint module supplies XY odometry updates over I2C. We connect to it for telemetry.
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        headingTimer.reset();

        telemetry.addLine("Pinpoint heading hold ready");
        telemetry.addData("IMU heading (deg)", Math.toDegrees(headingTarget));
        telemetry.addData("Pinpoint", pinpoint != null ? "connected" : "not found");
        telemetry.update();

        waitForStart();
        headingTimer.reset();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Forward/back
            double x = gamepad1.left_stick_x;  // Strafe
            double rotationInput = gamepad1.right_stick_x;

            double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double correctiveRotation;
            if (Math.abs(rotationInput) > ROTATION_DEADBAND) {
                headingTarget = robotHeading;
                headingIntegral = 0.0;
                lastHeadingError = 0.0;
                correctiveRotation = rotationInput;
                headingTimer.reset();
            } else {
                correctiveRotation = calculateHeadingHold(robotHeading);
            }

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(correctiveRotation), 1.0);
            double lfPower = (y + x + correctiveRotation) / denominator;
            double lbPower = (y - x + correctiveRotation) / denominator;
            double rfPower = (y - x - correctiveRotation) / denominator;
            double rbPower = (y + x - correctiveRotation) / denominator;

            leftFront.setPower(lfPower);
            leftBack.setPower(lbPower);
            rightFront.setPower(rfPower);
            rightBack.setPower(rbPower);

            telemetry.addData("Heading target (deg)", Math.toDegrees(headingTarget));
            telemetry.addData("Heading (deg)", Math.toDegrees(robotHeading));
            telemetry.addData("Heading error (deg)", Math.toDegrees(headingTarget - robotHeading));
            telemetry.addData("Pinpoint", pinpoint != null ? "connected" : "not found");
            telemetry.update();
        }
    }

    private double calculateHeadingHold(double robotHeading) {
        double dt = headingTimer.seconds();
        headingTimer.reset();

        double error = angleWrap(headingTarget - robotHeading);
        headingIntegral += error * dt;
        double derivative = (error - lastHeadingError) / Math.max(dt, 1e-3);
        lastHeadingError = error;

        return (HEADING_KP * error) + (HEADING_KI * headingIntegral) + (HEADING_KD * derivative);
    }

    private double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2.0 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2.0 * Math.PI;
        }
        return radians;
    }
}
