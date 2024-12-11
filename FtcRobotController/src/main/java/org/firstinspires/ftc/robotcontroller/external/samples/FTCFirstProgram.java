package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class FTCFirstProgram extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor1 = null; // Front Left
    private DcMotor motor2 = null; // Front Right
    private DcMotor motor3 = null; // Back Left
    private DcMotor motor4 = null; // Back Right
    private DcMotor arm; // Arm motor without encoder

    private double speedMultiplier = 1.0; // Default to full speed
    private boolean isSlowMode = false;
    private ElapsedTime toggleTimer = new ElapsedTime(); // Timer for LB toggle cooldown

    private static final double TICKS_PER_REV = 537.7; // Encoder ticks per revolution (adjust for your motor)
    private static final double MAX_POWER = 0.8; // Maximum arm power for movement
    private static final double HOLDING_POWER = 0.5; // Power to hold arm position
    private static final double MIN_HOLDING_POWER = 0.2; // Minimum holding power to reduce strain
    private static final double TEMPERATURE_THRESHOLD = 70.0; // Simulated overheating threshold

    private double armTemperature = 25.0; // Simulated motor temperature (in degrees Celsius)
    private boolean isCooling = false; // Flag for cooling down the arm motor

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");
        arm = hardwareMap.get(DcMotor.class, "arm");

        // Set drive motors to use encoders
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set arm motor to RUN_WITHOUT_ENCODER
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set motor directions to default
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor3.setDirection(DcMotor.Direction.FORWARD);
        motor4.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start
        waitForStart();
        runtime.reset();
        toggleTimer.reset();

        while (opModeIsActive()) {
            // Toggle speed with LB button
            if (gamepad1.left_bumper && toggleTimer.seconds() > 2.0) {
                isSlowMode = !isSlowMode;
                speedMultiplier = isSlowMode ? 0.3 : 1.0;
                toggleTimer.reset();
            }

            // Simulate temperature increase during movement
            if (gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1) {
                armTemperature += 0.1; // Increase temperature during active use
            } else {
                armTemperature -= 0.05; // Cool down naturally when idle
            }

            armTemperature = Math.max(25.0, armTemperature); // Prevent temperature from going below ambient

            // Check for overheating
            if (armTemperature >= TEMPERATURE_THRESHOLD) {
                isCooling = true;
                arm.setPower(0); // Turn off power to cool down
            } else if (armTemperature < TEMPERATURE_THRESHOLD - 5.0) {
                isCooling = false; // Resume normal operation after cooling
            }

            // Control the arm motor
            if (!isCooling) {
                if (gamepad1.right_trigger > 0.1) {
                    arm.setPower(MAX_POWER); // Move arm up
                } else if (gamepad1.left_trigger > 0.1) {
                    arm.setPower(-MAX_POWER); // Move arm down
                } else {
                    // Apply dynamic holding power based on load and temperature
                    double dynamicHoldingPower = Math.max(MIN_HOLDING_POWER, HOLDING_POWER - (armTemperature / 100.0));
                    arm.setPower(dynamicHoldingPower);
                }
            }

            // Telemetry
            telemetry.addData("Speed Mode", isSlowMode ? "Slow" : "Fast");
            telemetry.addData("Speed Multiplier", "%.2f", speedMultiplier);
            telemetry.addData("Arm Motor", "Power: %.2f", arm.getPower());
            telemetry.addData("Arm Temperature", "%.1f°C", armTemperature);
            telemetry.addData("Cooling", isCooling ? "Yes" : "No");
            telemetry.update();
        }
    }
}