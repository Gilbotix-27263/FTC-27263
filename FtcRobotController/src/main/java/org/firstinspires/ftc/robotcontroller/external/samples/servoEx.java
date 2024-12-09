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

        // Set initial position (neutral)
        arm.setPosition(0.5); // Neutral position to stop the motor

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start
        waitForStart();

        while (opModeIsActive()) {
            // Control motor using gamepad triggers
            if (gamepad1.right_trigger > 0.1) {
                // Forward rotation
                arm.setPosition(0.5 + gamepad1.right_trigger * 0.5); // Forward speed
            } else if (gamepad1.left_trigger > 0.1) {
                // Reverse rotation
                arm.setPosition(0.5 - gamepad1.left_trigger * 0.5); // Reverse speed
            } else {
                // Stop the motor
                arm.setPosition(0.5); // Neutral position
            }

            // Display servo position
            telemetry.addData("Servo Position", "%.2f", arm.getPosition());
            telemetry.update();
        }
    }
}
