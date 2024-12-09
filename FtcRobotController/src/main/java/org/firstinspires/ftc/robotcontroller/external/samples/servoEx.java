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

        // Set the servo to stop by default
        arm.setPosition(0.5); // 0.5 is neutral for continuous rotation servos

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Servo Name", "arm");
        telemetry.update();

        // Wait for the game to start
        waitForStart();

        while (opModeIsActive()) {
            double power = 0.5; // Default to neutral (stopped)

            // Use gamepad triggers to control the servo speed
            if (gamepad1.right_trigger > 0.1) {
                power = 0.5 + gamepad1.right_trigger * 0.5; // Rotate forward
            } else if (gamepad1.left_trigger > 0.1) {
                power = 0.5 - gamepad1.left_trigger * 0.5; // Rotate backward
            }

            // Set the servo power
            arm.setPosition(power);

            // Telemetry to display the servo state
            telemetry.addData("Servo Name", "arm");
            telemetry.addData("Servo Power", "%.2f", power);
            telemetry.update();
        }
    }
}