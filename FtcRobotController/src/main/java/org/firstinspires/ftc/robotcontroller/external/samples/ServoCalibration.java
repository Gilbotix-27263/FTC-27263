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
                servoMovingIntake.setPosition(calib0Degrees);
                telemetry.addData("Servo Command", "0 Degrees (%.2f)", calib0Degrees);
            }

            // Move servo to 90 degrees when "B" is pressed
            if (gamepad1.b) {
                servoMovingIntake.setPosition(calib90Degrees);
                telemetry.addData("Servo Command", "90 Degrees (%.2f)", calib90Degrees);
            }

            // Telemetry for feedback
            telemetry.addData("Calibration - 0 Degrees", "%.2f", calib0Degrees);
            telemetry.addData("Calibration - 90 Degrees", "%.2f", calib90Degrees);
            telemetry.update();
        }
    }
}
