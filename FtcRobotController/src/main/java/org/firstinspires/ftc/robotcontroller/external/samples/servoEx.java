package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class servoEx extends LinearOpMode {
    private Servo arm; // Motor controller connected as a servo
    private static final double INCREMENT = 0.05; // Small increment for smoother movement
    private static final long DELAY_MS = 50; // Delay in milliseconds to slow updates

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the servo
        arm = hardwareMap.get(Servo.class, "arm");

        // Set initial position to 0.00 (fully retracted or start position)
        arm.setPosition(0.50);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        double currentPosition = 0.00; // Track the current position

        // Wait for the game to start
        waitForStart();

        while (opModeIsActive()) {
            // Check input and update position
            if (gamepad1.right_trigger > 0.1) {
                currentPosition += INCREMENT; // Increment position
            } else if (gamepad1.left_trigger > 0.1) {
                currentPosition -= INCREMENT; // Decrement position
            } else {
                // Ensure no position change when triggers are not pressed
                currentPosition = arm.getPosition(); // Maintain current position
            }

            // Clamp the position to the valid range (0.0 to 1.0)
            currentPosition = Math.max(0.0, Math.min(1.0, currentPosition));

            // Update the servo position
            arm.setPosition(currentPosition);

            // Telemetry
            telemetry.addData("Servo Position", "%.2f", currentPosition);
            telemetry.update();

            // Delay to slow down updates

        }
    }
}