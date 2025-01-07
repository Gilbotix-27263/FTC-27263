package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(group = "Main")
public class ArmEx extends LinearOpMode {
    private DcMotor arm; // DC motor for the arm
    private static final double MAX_POWER = 0.8; // Maximum motor power for movement
    private static final double HOLDING_POWER = 0.5; // Power for holding position (adjust experimentally)

    private boolean isHolding = false; // Tracks whether the motor is holding position

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the motor
        arm = hardwareMap.get(DcMotor.class, "arm");

        // Set motor to RUN_WITHOUT_ENCODER mode
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start
        waitForStart();

        while (opModeIsActive()) {
            // Control arm movement
            if (gamepad1.right_trigger > 0.1) {
                // Move arm up
                arm.setPower(MAX_POWER);
                isHolding = false; // Not holding position
            } else if (gamepad1.left_trigger > 0.1) {
                // Move arm down
                arm.setPower(-MAX_POWER);
                isHolding = false; // Not holding position
            } else {
                // Hold the position with HOLDING_POWER
                if (!isHolding) {
                    arm.setPower(HOLDING_POWER); // Apply holding power to counteract gravity
                    isHolding = true;
                }
            }

            // Telemetry
            telemetry.addData("Motor Power", arm.getPower());
            telemetry.addData("Holding Position", isHolding ? "Yes" : "No");
            telemetry.update();
        }
    }
}