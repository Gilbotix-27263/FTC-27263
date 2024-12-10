package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class servoEx extends LinearOpMode {
    private DcMotor arm; // DC motor with encoder for the arm
    private static final double MAX_POWER = 0.8; // Maximum motor power for movement
    private static final double HOLDING_POWER = 0.5; // Power for holding position (adjust experimentally)
    private static final int TICKS_PER_REV = 537; // Encoder ticks per revolution (adjust for your motor)
    private static final int ARM_MAX_POS = 1000; // Maximum arm position (adjust as needed)
    private static final int ARM_MIN_POS = 0;    // Minimum arm position

    private int targetPosition = 0; // Target position to hold when idle

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the motor
        arm = hardwareMap.get(DcMotor.class, "arm");

        // Reset and set the motor to use the encoder
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start
        waitForStart();

        while (opModeIsActive()) {
            // Get the current encoder position
            int currentPosition = arm.getCurrentPosition();

            // Control arm movement
            if (gamepad1.right_trigger > 0.1 && currentPosition < ARM_MAX_POS) {
                // Move arm up
                targetPosition += TICKS_PER_REV / 10; // Increment target position
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setTargetPosition(targetPosition);
                arm.setPower(MAX_POWER);
            } else if (gamepad1.left_trigger > 0.1 && currentPosition > ARM_MIN_POS) {
                // Move arm down
                targetPosition -= TICKS_PER_REV / 10; // Decrement target position
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setTargetPosition(targetPosition);
                arm.setPower(MAX_POWER);
            } else {
                // Maintain position when idle with strong holding power
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setTargetPosition(targetPosition);
                arm.setPower(HOLDING_POWER); // Apply enough power to counteract weight
            }

            // Clamp target position to valid range
            targetPosition = Math.max(ARM_MIN_POS, Math.min(ARM_MAX_POS, targetPosition));

            // Telemetry
            telemetry.addData("Arm Position", currentPosition);
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Motor Power", arm.getPower());
            telemetry.update();
        }
    }
}