package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp
public class HoldPos extends LinearOpMode {
    // Motors
    private DcMotor motor1; // Front Left
    private DcMotor motor2; // Front Right
    private DcMotor motor3; // Back Left
    private DcMotor motor4; // Back Right

    // IMU
    private IMU imu;

    // Toggle variables
    private boolean isAutonomousActive = false; // Tracks the autonomous mode state
    private long lastToggleTime = 0; // Tracks the time of the last toggle
    private double targetYaw = 90.0; // Initial target yaw

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");

        // Define hub orientation
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        // Set hub orientation
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Initialize IMU with orientation
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Wait for the game to start
        telemetry.addData("Status", "Waiting for start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Toggle autonomous mode with the A button (2-second delay)
            if (gamepad1.a && System.currentTimeMillis() - lastToggleTime > 2000) {
                isAutonomousActive = !isAutonomousActive;
                lastToggleTime = System.currentTimeMillis();
                telemetry.addData("Autonomous Mode", isAutonomousActive ? "ON" : "OFF");
                telemetry.update();
            }

            // If autonomous mode is active, move to the target yaw
            if (isAutonomousActive) {
                // Check for B button to adjust target yaw
                if (gamepad1.b && System.currentTimeMillis() - lastToggleTime > 500) {
                    targetYaw = normalizeAngle(targetYaw + 10.0); // Add 10 degrees and normalize
                    lastToggleTime = System.currentTimeMillis();
                }
                moveToPosition(targetYaw); // Move to updated target yaw
            } else {
                stopMotors(); // Stop all motors if autonomous mode is off
                showOrientationTelemetry(); // Show pitch, roll, and yaw
            }
        }
    }

    /**
     * Moves the robot to a specific yaw position (Z-axis).
     *
     * @param targetZ Target yaw in degrees
     */
    private void moveToPosition(double targetZ) {
        // Get current orientation
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double currentZ = orientation.getYaw(AngleUnit.DEGREES);

        // Normalize the error to handle the yaw wrapping
        double error = normalizeAngle(targetZ - currentZ);

        // Apply proportional control to correct the yaw
        if (Math.abs(error) > 0.0 && opModeIsActive()) { // Deadband of 0 degree
            double proportionalGain = 0.05; // Lower gain for smoother control
            double power = error * proportionalGain;
            power = Math.max(-0.8, Math.min(0.8, power)); // Limit power to [-0.3, 0.3]

            // Apply power to all motors in the same direction
            motor1.setPower(power);
            motor2.setPower(power);
            motor3.setPower(power);
            motor4.setPower(power);

            // Show telemetry for holding position
            telemetry.addData("Target Z", "%.2f°", targetZ);
            telemetry.addData("Current Z", "%.2f°", currentZ);
            telemetry.addData("Error", "%.2f°", error);
            telemetry.addData("Power", "%.2f", power);
            telemetry.update();
        } else {
            stopMotors(); // Stop motors when error is within deadband
        }
    }


    /**
     * Show telemetry for pitch, roll, and yaw when not holding position.
     */
    private void showOrientationTelemetry() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        telemetry.addData("X Axis (Pitch)", "%.2f°", orientation.getPitch(AngleUnit.DEGREES));
        telemetry.addData("Y Axis (Roll)", "%.2f°", orientation.getRoll(AngleUnit.DEGREES));
        telemetry.addData("Z Axis (Yaw)", "%.2f°", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.update();
    }

    /**
     * Normalize an angle to the range [-180, 180].
     *
     * @param angle The angle to normalize
     * @return The normalized angle
     */
    private double normalizeAngle(double angle) {
        while (angle > 180) {
            angle -= 360;
        }
        while (angle < -180) {
            angle += 360;
        }
        return angle;
    }

    /**
     * Stops all motors.
     */
    private void stopMotors() {
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
    }
}
