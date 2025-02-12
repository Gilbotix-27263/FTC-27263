package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;

public class LEDIndicator {
    private Servo neoPixelServo;
    private double selectedColor = 0.5;
    private boolean isLocked = false;

    public LEDIndicator(HardwareMap hardwareMap) {
        neoPixelServo = hardwareMap.get(Servo.class, "neoPixelServo");
    }

    public void initialize() {
        neoPixelServo.setPosition(selectedColor);
    }

    public void control(Gamepad gamepad1, boolean opModeActive) {
        if (!isLocked) {
            if (gamepad1.a) {
                selectedColor = 0.0; // Red
            } else if (gamepad1.b) {
                selectedColor = 1.0; // Blue
            }
            neoPixelServo.setPosition(selectedColor);
        }

        // Lock the color after the op mode starts
        if (opModeActive) {
            isLocked = true;
        }
    }
}
