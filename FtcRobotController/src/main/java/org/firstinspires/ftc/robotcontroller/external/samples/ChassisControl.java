package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Chassis Control", group = "Samples")
public class ChassisControl extends LinearOpMode {

    private DcMotor motor1;
    private DcMotor motor2;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");

        // Set motor directions
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Forward and backward control
            double drive = gamepad1.left_stick_y;

            // Turning control
            double turn = -gamepad1.right_stick_x;

            // Motor power calculations
            double leftPower = drive + turn;
            double rightPower = drive - turn;

            // Set motor power
            motor1.setPower(leftPower);
            motor2.setPower(rightPower);

            // Telemetry for debugging
            telemetry.addData("Motor 1 Power", motor1.getPower());
            telemetry.addData("Motor 2 Power", motor2.getPower());
            telemetry.update();
        }
    }
}
