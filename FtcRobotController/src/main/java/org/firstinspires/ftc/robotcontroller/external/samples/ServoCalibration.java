package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Calibration", group = "Calibration")
public class ServoCalibration extends LinearOpMode {
    private Servo servoMovingIntake;

    // Default positions for calibration (adjust as needed)
    private static final double DEFAULT_0_DEGREES = 0.0;
    private static final double DEFAULT_90_DEGREES = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the servo
        servoMovingIntake = hardwareMap.get(Servo.class, "movingIntake");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Move servo to 0 degrees
            if (gamepad1.a) {
                servoMovingIntake.setPosition(DEFAULT_0_DEGREES);
                telemetry.addData("Servo Position", "0 Degrees (%.2f)", DEFAULT_0_DEGREES);
            }

            // Move servo to 90 degrees
            if (gamepad1.b) {
                servoMovingIntake.setPosition(DEFAULT_90_DEGREES);
                telemetry.addData("Servo Position", "90 Degrees (%.2f)", DEFAULT_90_DEGREES);
            }

            // Telemetry for feedback
            telemetry.update();
        }
    }
}
