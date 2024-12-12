package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp
public class SensorColor extends LinearOpMode {
    private ColorSensor colorSensor; // REV Color Sensor V3

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the color sensor
        colorSensor = hardwareMap.get(ColorSensor.class, "CSensor");

        // Wait for the game to start
        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();
        waitForStart();

        // Main loop
        while (opModeIsActive()) {
            // Read color values
            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();
            int alpha = colorSensor.alpha(); // Overall brightness

            // Display color values
            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);
            telemetry.addData("Alpha (Brightness)", alpha);

            // If you want to calculate and display RGB-based color data (optional)
            telemetry.addData("Detected Color", getColorName(red, green, blue));

            telemetry.update();
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
        if (red > green && red > blue) {
            return "Red";
        } else if (green > red && green > blue) {
            return "Green";
        } else if (blue > red && blue > green) {
            return "Blue";
        } else {
            return "Unknown";
        }
    }
}
