package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class servoEx extends LinearOpMode {
    private Servo arm; // Motor controller connected as a servo

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the servo
        arm = hardwareMap.get(Servo.class, "arm");

        // Set initial position to 0.00 (fully retracted or start position)
        arm.setPosition(0.00);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start
        waitForStart();

        while (opModeIsActive()) {
            // Control motor using gamepad triggers
            if (gamepad1.right_trigger > 0.1) {
                // Forward rotation (slower speed)
                arm.setPosition(arm.getPosition() + gamepad1.right_trigger * 0.01); // Increment by a small amount
            } else if (gamepad1.left_trigger > 0.1) {
                // Reverse rotation (slower speed)
                arm.setPosition(arm.getPosition() - gamepad1.left_trigger * 0.01); // Decrement by a small amount
            }

            // Clamp the position to the valid range (0.0 to 1.0)
            arm.setPosition(Math.max(0.0, Math.min(1.0, arm.getPosition())));

            // Display servo position
            telemetry.addData("Servo Position", "%.2f", arm.getPosition());
            telemetry.update();
        }
    }
}