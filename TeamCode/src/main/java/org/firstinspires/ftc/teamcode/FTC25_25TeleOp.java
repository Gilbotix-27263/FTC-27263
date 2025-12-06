package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "FTC25-25 TeleOp", group = "Linear Opmode")
public class FTC25_25TeleOp extends LinearOpMode {
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor intake;
    private DcMotor leftLauncher;
    private DcMotor rightLauncher;

    private static final double DRIVE_POWER_SCALE = 1.0;
    private static final double INTAKE_POWER = 1.0;
    private static final double LAUNCHER_POWER = 1.0;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "LeftFront");
        leftBack = hardwareMap.get(DcMotor.class, "LeftBack");
        rightFront = hardwareMap.get(DcMotor.class, "RightFront");
        rightBack = hardwareMap.get(DcMotor.class, "RightBack");
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftLauncher = hardwareMap.get(DcMotor.class, "LLauncher");
        rightLauncher = hardwareMap.get(DcMotor.class, "RLauncher");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Positive power spins the intake counter-clockwise and launchers in opposite directions
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLauncher.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addLine("FTC25-25 TeleOp ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y * DRIVE_POWER_SCALE; // Forward/backward
            double x = gamepad1.left_stick_x * DRIVE_POWER_SCALE; // Strafing
            double rx = gamepad1.right_stick_x * DRIVE_POWER_SCALE; // Rotation

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double lfPower = (y + x + rx) / denominator;
            double lbPower = (y - x + rx) / denominator;
            double rfPower = (y - x - rx) / denominator;
            double rbPower = (y + x - rx) / denominator;

            leftFront.setPower(lfPower);
            leftBack.setPower(lbPower);
            rightFront.setPower(rfPower);
            rightBack.setPower(rbPower);

            if (gamepad1.right_trigger > 0.05) {
                leftLauncher.setPower(LAUNCHER_POWER);
                rightLauncher.setPower(LAUNCHER_POWER);
            } else {
                leftLauncher.setPower(0);
                rightLauncher.setPower(0);
            }

            if (gamepad1.a) {
                intake.setPower(INTAKE_POWER);
            } else {
                intake.setPower(0);
            }

            telemetry.addData("Drive", "LF: %.2f LB: %.2f RF: %.2f RB: %.2f", lfPower, lbPower, rfPower, rbPower);
            telemetry.addData("Launcher", gamepad1.right_trigger > 0.05 ? "ACTIVE" : "OFF");
            telemetry.addData("Intake", gamepad1.a ? "ACTIVE" : "OFF");
            telemetry.update();
        }
    }
}
