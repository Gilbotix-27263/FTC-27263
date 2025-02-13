package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Calibration with Smooth Transition", group = "Calibration")
@Disabled
public class ServoCalibration extends LinearOpMode {
    private Servo servoMovingIntake;

    // Default positions for calibration
    private double calib0Degrees = 0.0;
    private double calib90Degrees = 1.0;
    private double currentPosition = 0.0; // Tracks the servo's current position

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the servo
        servoMovingIntake = hardwareMap.get(Servo.class, "movingIntake");

        // Set initial servo position
        currentPosition = calib0Degrees;
        servoMovingIntake.setPosition(currentPosition);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Adjust the position for 0 degrees using D-pad Up/Down
            if (gamepad1.dpad_up) {
                calib0Degrees += 0.01; // Increment position
            } else if (gamepad1.dpad_down) {
                calib0Degrees -= 0.01; // Decrement position
            }

            // Adjust the position for 90 degrees using D-pad Left/Right
            if (gamepad1.dpad_left) {
                calib90Degrees -= 0.01; // Decrement position
            } else if (gamepad1.dpad_right) {
                calib90Degrees += 0.01; // Increment position
            }

            // Clip the values to ensure they're within the valid servo range
            calib0Degrees = Math.max(0.0, Math.min(1.0, calib0Degrees));
            calib90Degrees = Math.max(0.0, Math.min(1.0, calib90Degrees));

            // Move servo to 0 degrees when "A" is pressed
            if (gamepad1.a) {
                smoothMove(calib0Degrees);
                telemetry.addData("Moving to 0 Degrees", "Target: %.2f", calib0Degrees);
            }

            // Move servo to 90 degrees when "B" is pressed
            if (gamepad1.b) {
                smoothMove(calib90Degrees);
                telemetry.addData("Moving to 90 Degrees", "Target: %.2f", calib90Degrees);
            }

            // Telemetry for feedback
            telemetry.addData("Calibration - 0 Degrees", "%.2f", calib0Degrees);
            telemetry.addData("Calibration - 90 Degrees", "%.2f", calib90Degrees);
            telemetry.addData("Current Servo Position", "%.2f", currentPosition);
            telemetry.update();
        }
    }

    /**
     * Smoothly moves the servo to the target position in small increments.
     *
     * @param targetPosition The desired servo position to move to.
     */
    private void smoothMove(double targetPosition) {
        double step = 0.01; // Step size for smooth transitions
        if (currentPosition < targetPosition) {
            while (currentPosition < targetPosition) {
                currentPosition += step;
                currentPosition = Math.min(currentPosition, targetPosition); // Ensure it doesn't overshoot
                servoMovingIntake.setPosition(currentPosition);
                telemetry.addData("Servo Position (Increasing)", "%.2f", currentPosition);
                telemetry.update();
                sleep(20); // Small delay for smooth motion
            }
        } else if (currentPosition > targetPosition) {
            while (currentPosition > targetPosition) {
                currentPosition -= step;
                currentPosition = Math.max(currentPosition, targetPosition); // Ensure it doesn't undershoot
                servoMovingIntake.setPosition(currentPosition);
                telemetry.addData("Servo Position (Decreasing)", "%.2f", currentPosition);
                telemetry.update();
                sleep(20); // Small delay for smooth motion
            }
        }
        currentPosition = targetPosition; // Set to the final position
    }
}
