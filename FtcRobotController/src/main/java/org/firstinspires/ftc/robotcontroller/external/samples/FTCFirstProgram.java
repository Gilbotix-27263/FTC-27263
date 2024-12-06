package org.firstinspires.ftc.robotcontroller.external.samples;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class FTCFirstProgram extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor1 = null; // Front Left
    private DcMotor motor2 = null; // Front Right
    private DcMotor motor3 = null; // Back Left
    private DcMotor motor4 = null; // Back Right

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");

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

        while (opModeIsActive()) {
            // Read inputs
            double drive = -gamepad1.left_stick_y; // Forward/Backward
            double turn = gamepad1.right_stick_x; // Turning
            double sideDrive = gamepad1.left_stick_x; // Strafing (Sideways)

            // Control motors separately
            if (Math.abs(drive) > 0.1) {
                handleDrive(drive);
            } else if (Math.abs(sideDrive) > 0.1) {
                handleStrafe(sideDrive);
            } else if (Math.abs(turn) > 0.1) {
                handleTurn(turn);
            } else {
                // Stop all motors if no input is detected
                stopAllMotors();
            }

            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }

    private void handleDrive(double drive) {
        // All motors move forward or backward equally
        double power = Range.clip(drive, -1.0, 1.0);
        motor1.setPower(power);
        motor2.setPower(-power);
        motor3.setPower(-power);
        motor4.setPower(power);

        telemetry.addData("Drive", "Power (%.2f)", power);
    }

    private void handleStrafe(double sideDrive) {
        // Left wheels move opposite to right wheels for strafing
        double power = Range.clip(sideDrive, -1.0, 1.0);
        motor1.setPower(power);  // Front Left
        motor2.setPower(power); // Front Right
        motor3.setPower(-power); // Back Left
        motor4.setPower(-power);  // Back Right

        telemetry.addData("Strafe", "Power (%.2f)", power);
    }

    private void handleTurn(double turn) {
        // Left and right wheels move in opposite directions for turning
        double power = Range.clip(turn, -1.0, 1.0);
        motor1.setPower(power);  // Front Left
        motor2.setPower(-power); // Front Right
        motor3.setPower(power);  // Back Left
        motor4.setPower(-power); // Back Right

        telemetry.addData("Turn", "Power (%.2f)", power);
    }

    private void stopAllMotors() {
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);

        telemetry.addData("Motors", "Stopped");
    }
}