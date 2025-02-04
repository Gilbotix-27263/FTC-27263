package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "NeoPixel_TeamColor", group = "Main")
public class NeoPixel_TeamColor extends LinearOpMode {

    private Servo pwmServo;
    private double selectedColor = 0.5; // Default to Purple (middle)

    @Override
    public void runOpMode() {
        pwmServo = hardwareMap.get(Servo.class, "neoPixelServo"); // Match FTC Config

        // Set default color to Purple before INIT is pressed
        pwmServo.setPosition(0.5);

        telemetry.addData("Status", "Initialized - Defaulting to Purple");
        telemetry.addData("Select Team", "Press A for Red, B for Blue");
        telemetry.update();

        // Team selection loop (continuously update color until Start is pressed)
        while (!isStarted()) {
            if (gamepad1.a) {
                selectedColor = 0.0; // Red Team
            } else if (gamepad1.b) {
                selectedColor = 1.0; // Blue Team
            }

            pwmServo.setPosition(selectedColor); // Continuously update the PWM signal

            telemetry.addData("Selected Team", selectedColor == 0.0 ? "Red" : "Blue");
            telemetry.update();
        }

        waitForStart(); // Now we wait for the match to start

        // Once the match starts, lock the last color by keeping the same position
        pwmServo.setPosition(selectedColor);

        telemetry.addData("Final Color", selectedColor == 0.0 ? "Red" : "Blue");
        telemetry.update();

        while (opModeIsActive()) {
            // Keep running, but do not change the LED color anymore
        }
    }
}
