package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSystem {
    private CRServo servoIntakeLeft, servoIntakeRight;
    private Servo servoMovingIntake;

    public IntakeSystem(HardwareMap hardwareMap) {
        servoIntakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        servoIntakeRight = hardwareMap.get(CRServo.class, "intakeRight");
        servoMovingIntake = hardwareMap.get(Servo.class, "movingIntake");
    }

    public void initialize() {
        servoMovingIntake.setPosition(0.1333);
    }

    public void control(Gamepad gamepad2) {
        if (gamepad2.a) {
            servoIntakeLeft.setPower(1.0);
            servoIntakeRight.setPower(-1.0);
        } else if (gamepad2.b) {
            servoIntakeLeft.setPower(-1.0);
            servoIntakeRight.setPower(1.0);
        } else {
            servoIntakeLeft.setPower(0.0);
            servoIntakeRight.setPower(0.0);
        }

        double movingIntakePosition = gamepad2.left_bumper ? 0.1333 : (gamepad2.right_bumper ? 0.8333 : 0.5);
        servoMovingIntake.setPosition(movingIntakePosition);
    }
}
