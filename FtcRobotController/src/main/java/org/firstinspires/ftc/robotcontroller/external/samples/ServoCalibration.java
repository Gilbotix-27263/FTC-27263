package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Calibration", group = "Calibration")
public class ServoCalibration extends LinearOpMode {
    private Servo servoMovingIntake;

    // Default positions for calibration (can be adjusted in real-time)
    private double calib0Degrees = 0.0;
    private double calib90Degrees = 1.0;
    private double currentPosition = 0.0; // Tracks the current servo position

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the servo
        servoMovingIntake = hardwareMap.get(Servo.class, "movingIntake");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Adjust the position for 0 degrees using gamepad D-pad
            if (gamepad1.dpad_up) {
                calib0Degrees += 0.01; // Increment position
            } else if (gamepad1.dpad_down) {
                calib0Degrees -= 0.01; // Decrement position
            }

            // Adjust the position for 90 degrees using gamepad D-pad left/right
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
                telemetry.addData("Servo Command", "Moving to 0 Degrees (%.2f)", calib0Degrees);
            }

            // Move servo to 90 degrees when "B" is pressed
            if (gamepad1.b) {
                smoothMove(calib90Degrees);
                telemetry.addData("Servo Command", "Moving to 90 Degrees (%.2f)", calib90Degrees);
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
        while (Math.abs(currentPosition - targetPosition) > step) {
            if (currentPosition < targetPosition) {
                currentPosition += step;
            } else if (currentPosition > targetPosition) {
                currentPosition -= step;
            }

            currentPosition = Math.max(0.0, Math.min(1.0, currentPosition)); // Ensure valid range
            servoMovingIntake.setPosition(currentPosition);
            sleep(20); // Add a small delay for smooth motion
        }

        // Set the position exactly at the target to ensure accuracy
        currentPosition = targetPosition;
        servoMovingIntake.setPosition(currentPosition);
    }
}
