package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ArmSystem {
    private DcMotor armUD, armEx;
    private TouchSensor armExZeroSensor;
    private static final double MAX_ARMUD_POWER = 0.4;
    private static final double MAX_ARMEX_POWER = 0.8;
    private static final int ARM_UD_HIGH = -2000;
    private static final int ARM_UD_MID = 0;
    private static final int ARM_UD_LOW = -500;
    private static final int ARM_EX_HIGH = -2000;
    private static final int ARM_EX_MID = 0;
    private static final int ARM_EX_LOW = -1000;
    private static final double ZERO_DELAY = 1.0;
    private boolean armExZeroed = false;
    private ElapsedTime zeroDelayTimer = new ElapsedTime();

    public ArmSystem(HardwareMap hardwareMap) {
        armUD = hardwareMap.get(DcMotor.class, "armUD");
        armEx = hardwareMap.get(DcMotor.class, "arm");
        armExZeroSensor = hardwareMap.get(TouchSensor.class, "armExZeroSensor");
    }

    public void initialize() {
        armUD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armUD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armUD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        zeroDelayTimer.reset();
        zeroArmExtension();
    }

    private void zeroArmExtension() {
        while (!armExZeroSensor.isPressed()) {
            armEx.setPower(0.2);
        }
        armEx.setPower(0.0);
        armEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armExZeroed = true;
    }

    public void control(Gamepad gamepad2) {
        double armUDPower = gamepad2.left_stick_y * MAX_ARMUD_POWER;
        double armExPower = gamepad2.right_trigger - gamepad2.left_trigger;

        if (armExZeroSensor.isPressed() && zeroDelayTimer.seconds() > ZERO_DELAY) {
            armEx.setPower(0.0);
            armEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            zeroDelayTimer.reset();
            armExZeroed = true;
        }

        int armExCurrentPosition = armEx.getCurrentPosition();
        if (armExCurrentPosition >= -100 && armExPower > 0) {
            armEx.setPower(0.0);
        } else if (armExCurrentPosition <= -2250 && armExPower < 0) {
            armEx.setPower(0.0);
        } else {
            armEx.setPower(armExPower * MAX_ARMEX_POWER);
        }

        int armUDCurrentPosition = armUD.getCurrentPosition();
        if ((armUDCurrentPosition <= -2200 && armUDPower < 0) || (armUDCurrentPosition >= 0 && armUDPower > 0)) {
            armUD.setPower(0.0);
        } else {
            armUD.setPower(armUDPower);
        }

        if (gamepad2.right_stick_y < -0.5) {
            armUD.setTargetPosition(ARM_UD_HIGH);
            armEx.setTargetPosition(ARM_EX_HIGH);
        } else if (gamepad2.right_stick_y > 0.5) {
            armUD.setTargetPosition(ARM_UD_LOW);
            armEx.setTargetPosition(ARM_EX_LOW);
        } else {
            armUD.setTargetPosition(ARM_UD_MID);
            armEx.setTargetPosition(ARM_EX_MID);
        }
        armUD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armUD.setPower(0.5);
        armEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armEx.setPower(0.5);
    }
}
