package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class servoEx extends LinearOpMode {
    private Servo arm; // Servo instance for the arm

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the servo
        arm = hardwareMap.get(Servo.class, "arm");

        // Set the initial position of the servo
        arm.setPosition(0.5); // Midpoint (Range is 0.0 to 1.0)

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Servo Name", "arm");
        telemetry.update();

        // Wait for the game to start
        waitForStart();

        while (opModeIsActive()) {
            // Use gamepad triggers to control the servo position
            double position = arm.getPosition(); // Get the current position
            if (gamepad1.right_trigger > 0.1) {
                position += 0.01; // Increase position
            }
            if (gamepad1.left_trigger > 0.1) {
                position -= 0.01; // Decrease position
            }

            // Clamp the position to the valid range (0.0 to 1.0)
            position = Math.max(0.0, Math.min(1.0, position));

            // Set the new position to the servo
            arm.setPosition(position);

            // Telemetry to display the servo position
            telemetry.addData("Servo Name", "arm");
            telemetry.addData("Servo Position", "%.2f", position);
            telemetry.update();
        }
    }
}