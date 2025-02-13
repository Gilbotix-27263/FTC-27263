package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DriveTrain {
    private DcMotor motor1, motor2, motor3, motor4;
    private double speedMultiplier = 1.0;
    private boolean isSlowMode = false;
    private ElapsedTime toggleTimer = new ElapsedTime();

    public DriveTrain(HardwareMap hardwareMap) {
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void initialize() {
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor3.setDirection(DcMotor.Direction.FORWARD);
        motor4.setDirection(DcMotor.Direction.REVERSE);
        toggleTimer.reset();
    }

    public void control(Gamepad gamepad1) {
        if (gamepad1.left_bumper && toggleTimer.seconds() > 0.5) {
            isSlowMode = !isSlowMode;
            speedMultiplier = isSlowMode ? 0.3 : 1.0;
            toggleTimer.reset();
        }

        double drive = gamepad1.left_stick_y * speedMultiplier;
        double turn = -gamepad1.right_stick_x * speedMultiplier;
        double strafe = -gamepad1.left_stick_x * speedMultiplier;

        double frontLeftPower = drive + turn + strafe;
        double frontRightPower = -drive - turn + strafe;
        double backLeftPower = -drive + turn - strafe;
        double backRightPower = drive - turn - strafe;

        // Set power to drive motors
        motor1.setPower(frontLeftPower);
        motor2.setPower(frontRightPower);
        motor3.setPower(backLeftPower);
        motor4.setPower(backRightPower);
    }
}
