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

            // Control motors for all movements
            setMotorPowers(drive, turn, sideDrive);

            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }

    private void setMotorPowers(double drive, double turn, double sideDrive) {
        // Calculate power for each motor based on input
        double frontLeftPower = drive + turn + sideDrive;
        double frontRightPower = drive - turn - sideDrive;
        double backLeftPower = drive + turn - sideDrive;
        double backRightPower = drive - turn + sideDrive;

        // Adjust for strafing: Left wheels move one direction, right wheels move opposite
        frontLeftPower = drive + turn + sideDrive; // Front left: drive + turn + strafe
        frontRightPower = drive - turn - sideDrive; // Front right: drive - turn - strafe
        backLeftPower = drive + turn - sideDrive; // Back left: drive + turn - strafe
        backRightPower = drive - turn + sideDrive; // Back right: drive - turn + strafe

        // Clip power values to ensure they're within the valid range
        frontLeftPower = Range.clip(frontLeftPower, -1.0, 1.0);
        frontRightPower = Range.clip(frontRightPower, -1.0, 1.0);
        backLeftPower = Range.clip(backLeftPower, -1.0, 1.0);
        backRightPower = Range.clip(backRightPower, -1.0, 1.0);

        // Set motor powers
        motor1.setPower(frontLeftPower);
        motor2.setPower(frontRightPower);
        motor3.setPower(backLeftPower);
        motor4.setPower(backRightPower);

        // Telemetry for debugging
        telemetry.addData("Motor Powers", "FL (%.2f), FR (%.2f), BL (%.2f), BR (%.2f)",
                frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }
}