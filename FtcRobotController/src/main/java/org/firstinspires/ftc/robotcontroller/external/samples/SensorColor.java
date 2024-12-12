package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class SensorColor extends LinearOpMode {
    private ColorSensor colorSensor; // REV Color Sensor V3
    private DcMotor arm; // DC motor for the arm

    private static final double MAX_POWER = 0.8; // Maximum motor power for movement
    private static final double HOLDING_POWER = 0.5; // Power for holding position
    private boolean isHolding = false; // Tracks whether the motor is holding position

    private boolean isOperating = false; // Tracks if the arm is in an active operation
    private long actionStartTime = 0; // Tracks the start time of an operation

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the color sensor and arm motor
        colorSensor = hardwareMap.get(ColorSensor.class, "CSensor");
        arm = hardwareMap.get(DcMotor.class, "arm");

        // Set arm motor to RUN_WITHOUT_ENCODER mode
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Read color values
            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();

            // Determine color and operate the arm
            if (isBlueDetected(red, green, blue) && !isOperating) {
                startArmOperation(MAX_POWER); // Extend the arm when blue is detected
            } else if (isYellowDetected(red, green, blue) && !isOperating) {
                startArmOperation(-MAX_POWER); // Retract the arm when yellow is detected
            }

            // Manage arm operation timing
            handleArmOperation();

            // Telemetry
            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);
            telemetry.addData("Detected Color", getColorName(red, green, blue));
            telemetry.addData("Arm Power", arm.getPower());
            telemetry.addData("Holding Position", isHolding ? "Yes" : "No");
            telemetry.update();
        }
    }

    /**
     * Checks if blue is detected based on RGB values.
     *
     * @param red   The red value
     * @param green The green value
     * @param blue  The blue value
     * @return True if blue is detected, otherwise false
     */
    private boolean isBlueDetected(int red, int green, int blue) {
        return blue > red && blue > green && blue > 100; // Adjust thresholds as needed
    }

    /**
     * Checks if yellow is detected based on RGB values.
     *
     * @param red   The red value
     * @param green The green value
     * @param blue  The blue value
     * @return True if yellow is detected, otherwise false
     */
    private boolean isYellowDetected(int red, int green, int blue) {
        return red > blue && green > blue && green > red && green > 100;
        // Red and green are high, blue is low, and the difference between red and green is small
    }

    /**
     * Checks if red is detected based on RGB values.
     *
     * @param red   The red value
     * @param green The green value
     * @param blue  The blue value
     * @return True if red is detected, otherwise false
     */
    private boolean isRedDetected(int red, int green, int blue) {
        return red > green && red > blue && (red - green) >= 50 && red > 100;
        // Red is dominant, and the difference between red and green is significant
    }


    /**
     * Starts an arm operation (extend or retract).
     *
     * @param power The motor power for the operation (+ for extend, - for retract)
     */
    private void startArmOperation(double power) {
        arm.setPower(power); // Start the arm operation
        actionStartTime = System.currentTimeMillis();
        isOperating = true;
        isHolding = false; // Reset holding state
    }

    /**
     * Manages the arm operation timing.
     */
    private void handleArmOperation() {
        if (isOperating) {
            long elapsedTime = System.currentTimeMillis() - actionStartTime;

            if (elapsedTime >= 2000) { // Stop the motor after 2 seconds
                arm.setPower(0);
                isOperating = false;
            }
        } else {
            holdArm(); // Hold position when no active operation
        }
    }

    /**
     * Holds the arm position using HOLDING_POWER.
     */
    private void holdArm() {
        if (!isHolding) {
            arm.setPower(HOLDING_POWER); // Apply holding power
            isHolding = true;
        }
    }

    /**
     * Determines a basic color name based on RGB values.
     *
     * @param red   The red value
     * @param green The green value
     * @param blue  The blue value
     * @return The detected color name
     */
    private String getColorName(int red, int green, int blue) {
        if (isBlueDetected(red, green, blue)) {
            return "Blue";
        } else if (isYellowDetected(red, green, blue)) {
            return "Yellow";
        } else if (isRedDetected(red, green, blue)) {
            return "Red";
        } else {
            return "Unknown";
        }
    }

}
