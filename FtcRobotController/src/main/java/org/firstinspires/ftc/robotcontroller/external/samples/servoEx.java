package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class servoEx extends LinearOpMode {
    private Servo servo1; // Motor controller connected as a servo

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the servo
        servo1 = hardwareMap.get(Servo.class, "servo1");

        // Set initial position to 0.00 (fully retracted or start position)
        servo1.setPosition(0.00);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        double currentPosition = 0.00; // Track the current position

        double x = 0;

        // Wait for the game to start
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.x && x == 0) {
                x = 1.0;
                if (currentPosition == 0.0) {
                    currentPosition = 1.0;
                } else if (currentPosition == 1.0) {
                    currentPosition = 0.0;
                } else if (gamepad1.x != true) {
                    x = 0.0;
                }
                // Clamp the position to the valid range (0.0 to 1.0)
                currentPosition = Math.max(0.0, Math.min(1.0, currentPosition));

                // Update the servo position
                servo1.setPosition(currentPosition);

                // Display the servo position in telemetry
                telemetry.addData("Servo Position", "%.2f", currentPosition);
                telemetry.update();
            }
        }
    }
}