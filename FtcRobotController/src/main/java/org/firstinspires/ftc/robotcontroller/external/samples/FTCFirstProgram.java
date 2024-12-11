package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class FTCFirstProgram extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor1 = null; // Front Left
    private DcMotor motor2 = null; // Front Right
    private DcMotor motor3 = null; // Back Left
    private DcMotor motor4 = null; // Back Right
    private DcMotor arm; // Arm motor without encoder
    private double speedMultiplier = 1.0; // Default to Null speed
    private boolean isSlowMode = false;
    private ElapsedTime toggleTimer = new ElapsedTime(); // Timer for LB toggle cooldown

    private static final double TICKS_PER_REV = 537.7; // Encoder ticks per revolution (adjust for your motor)
    private static final double MAX_POWER = 0.8; // Maximum arm power for movement
    private static final double HOLDING_POWER = 0.5; // Power to hold arm position

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

            // Read inputs and scale them by the speed multiplier
            double drive = gamepad1.left_stick_y * speedMultiplier; // Forward/Backward
            double turn = -gamepad1.right_stick_x * speedMultiplier; // Turning
            double strafe = -gamepad1.left_stick_x * speedMultiplier; // Strafing (Sideways)

            // Use provided function for motor powers
            double frontLeftPower = drive + turn + strafe;
            double frontRightPower = -drive - turn + strafe;
            double backLeftPower = -drive + turn - strafe;
            double backRightPower = drive - turn - strafe;

            // Normalize the powers if any exceed 1.0
            double maxPower = Math.max(1.0, Math.abs(frontLeftPower));
            maxPower = Math.max(maxPower, Math.abs(frontRightPower));
            maxPower = Math.max(maxPower, Math.abs(backLeftPower));
            maxPower = Math.max(maxPower, Math.abs(backRightPower));

            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;

            // Set motor powers
            motor1.setPower(frontLeftPower);
            motor2.setPower(frontRightPower);
            motor3.setPower(backLeftPower);
            motor4.setPower(backRightPower);

            // Control the arm motor
            if (gamepad1.right_trigger > 0.1) {
                arm.setPower(MAX_POWER); // Move arm up
            } else if (gamepad1.left_trigger > 0.1) {
                arm.setPower(-MAX_POWER); // Move arm down
            } else {
                arm.setPower(HOLDING_POWER); // Hold position when idle
            }


            // Telemetry
            telemetry.addData("Speed Mode", isSlowMode ? "Slow" : "Fast");
            telemetry.addData("Speed Multiplier", "%.2f", speedMultiplier);
            telemetry.addData("Motor 1 (FL)", "PW: %.2f, POS: %.0f°", frontLeftPower, toDegrees(motor1.getCurrentPosition()));
            telemetry.addData("Motor 2 (FR)", "PW: %.2f, POS: %.0f°", frontRightPower, toDegrees(motor2.getCurrentPosition()));
            telemetry.addData("Motor 3 (BL)", "PW: %.2f, POS: %.0f°", backLeftPower, toDegrees(motor3.getCurrentPosition()));
            telemetry.addData("Motor 4 (BR)", "PW: %.2f, POS: %.0f°", backRightPower, toDegrees(motor4.getCurrentPosition()));
            telemetry.addData("Arm Motor", "Power: %.2f", arm.getPower());
            telemetry.update();
        }
    }

    /**
     * Converts encoder ticks to degrees based on motor specifications.
     */
    private double toDegrees(int ticks) {
        return (360.0 * ticks) / TICKS_PER_REV;
    }
}