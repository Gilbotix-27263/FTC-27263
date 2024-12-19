package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "AutonomousMovement")
public class AutonomousMovement extends LinearOpMode {
    private DcMotor motor1; // Front Left
    private DcMotor motor2; // Front Right
    private DcMotor motor3; // Back Left
    private DcMotor motor4; // Back Right
    private IMU imu;

    private static final double DRIVE_SPEED = 0.5;
    private static final double TURN_SPEED = 0.3;
    private static final int COUNTS_PER_REV = 537; // Example: GoBILDA Yellow Jacket 312RPM
    private static final double WHEEL_DIAMETER = 4.0; // Wheel diameter in inches
    private static final double COUNTS_PER_INCH = (COUNTS_PER_REV) / (Math.PI * WHEEL_DIAMETER);

    @Override
    public void runOpMode() {
        // Initialize motors
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");

        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor3.setDirection(DcMotor.Direction.FORWARD);
        motor4.setDirection(DcMotor.Direction.REVERSE);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        // Reset encoders
        resetEncoders();

        telemetry.addData("Status", "Ready to start");
        telemetry.update();
        waitForStart();

        // Autonomous Sequence
        spinToAngle(0);                // Step 1: Spin to 0 degrees
        driveStraight(1000, DRIVE_SPEED); // Step 2: Drive forward 1000 encoder counts
        resetEncoders();               // Reset encoders after driving forward
        strafe(500, DRIVE_SPEED, true);  // Step 3: Strafe right 500 encoder counts
        resetEncoders();               // Reset encoders after strafing
        spinToAngle(180);              // Step 4: Spin to 180 degrees
        driveStraight(1000, DRIVE_SPEED); // Step 5: Drive backward 1000 encoder counts
        resetEncoders();               // Reset encoders after driving backward
        strafe(500, DRIVE_SPEED, true);  // Step 6: Strafe left 500 encoder counts
        resetEncoders();               // Reset encoders after strafing
        spinToAngle(0);

        telemetry.addData("Status", "Sequence Complete");
        telemetry.update();
    }

    private void spinToAngle(double targetAngle) {
        while (opModeIsActive()) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double currentYaw = orientation.getYaw(AngleUnit.DEGREES);
            double error = normalizeAngle(targetAngle - currentYaw);

            if (Math.abs(error) <= 0.09) { // Deadband of 1 degree
                stopMotors();
                break;
            }

            double power = error * 0.08; // Proportional control
            power = Range.clip(power, -TURN_SPEED, TURN_SPEED);

            motor1.setPower(power);
            motor2.setPower(-power);
            motor3.setPower(power);
            motor4.setPower(-power);

            telemetry.addData("Target Angle", "%.2f", targetAngle);
            telemetry.addData("Current Angle", "%.2f", currentYaw);
            telemetry.addData("Error", "%.2f", error);
            telemetry.addData("Power", "%.2f", power);
            telemetry.update();
        }
    }

    private void driveStraight(int encoderCounts, double speed) {
        resetEncoders();

        motor1.setTargetPosition(-encoderCounts);
        motor2.setTargetPosition(encoderCounts);
        motor3.setTargetPosition(encoderCounts);
        motor4.setTargetPosition(-encoderCounts);

        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor1.setPower(speed);
        motor2.setPower(speed);
        motor3.setPower(speed);
        motor4.setPower(speed);

        while (opModeIsActive() && motorsAreBusy()) {
            telemetry.addData("Path", "Driving straight");
            telemetry.update();
        }
        stopMotors();
    }

    private void strafe(int encoderCounts, double speed, boolean right) {
        resetEncoders();

        int direction = right ? 1 : -1;

        motor1.setTargetPosition(-encoderCounts * direction);
        motor2.setTargetPosition(-encoderCounts * direction);
        motor3.setTargetPosition(encoderCounts * direction);
        motor4.setTargetPosition(encoderCounts * direction);

        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor1.setPower(speed);
        motor2.setPower(speed);
        motor3.setPower(speed);
        motor4.setPower(speed);

        while (opModeIsActive() && motorsAreBusy()) {
            telemetry.addData("Path", "Strafing");
            telemetry.update();
        }
        stopMotors();
    }

    private void resetEncoders() {
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private boolean motorsAreBusy() {
        return motor1.isBusy() && motor2.isBusy() && motor3.isBusy() && motor4.isBusy();
    }

    private void stopMotors() {
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}
