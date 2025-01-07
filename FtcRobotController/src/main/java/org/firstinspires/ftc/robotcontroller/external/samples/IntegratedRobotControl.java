package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Integrated Robot Control", group = "Samples")
public class IntegratedRobotControl extends LinearOpMode {
    // Drive motors
    private DcMotor motor1, motor2, motor3, motor4;

    // Arm and intake motors/servos
    private DcMotor armUD, armEx;
    private Servo servoIntake, servoMovingIntake;

    // Constants
    private static final double MAX_ARM_POWER = 0.8;
    private static final double HOLDING_POWER = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");
        armUD = hardwareMap.get(DcMotor.class, "armUD");
        armEx = hardwareMap.get(DcMotor.class, "arm");
        servoIntake = hardwareMap.get(Servo.class, "intake");
        servoMovingIntake = hardwareMap.get(Servo.class, "movingIntake");

        // Configure motors
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor3.setDirection(DcMotor.Direction.FORWARD);
        motor4.setDirection(DcMotor.Direction.REVERSE);

        armUD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armUD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Telemetry setup
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for start signal
        waitForStart();

        while (opModeIsActive()) {
            // --- Controller 1: Driving ---
            double drive = gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;
            double strafe = -gamepad1.left_stick_x;

            double frontLeftPower = drive + turn + strafe;
            double frontRightPower = drive - turn - strafe;
            double backLeftPower = drive + turn - strafe;
            double backRightPower = drive - turn + strafe;

            motor1.setPower(frontLeftPower);
            motor2.setPower(frontRightPower);
            motor3.setPower(backLeftPower);
            motor4.setPower(backRightPower);

            // --- Controller 2: Arm and Intake ---
            // ArmUD movement with left joystick y-axis
            double armUDPower = -gamepad2.left_stick_y * MAX_ARM_POWER; // Invert for natural direction
            armUD.setPower(armUDPower);

            // ArmEx movement (manual control)
            if (gamepad2.right_trigger > 0.1) {
                armEx.setPower(MAX_ARM_POWER); // Move arm up
            } else if (gamepad2.left_trigger > 0.1) {
                armEx.setPower(-MAX_ARM_POWER); // Move arm down
            } else {
                armEx.setPower(HOLDING_POWER); // Hold position
            }

            // Servo control
            double intakePosition = gamepad2.a ? 0.0 : (gamepad2.b ? 1.0 : 0.5);
            double movingIntakePosition = gamepad2.left_bumper ? 0.0 : (gamepad2.right_bumper ? 1.0 : 0.5);

            servoIntake.setPosition(Range.clip(intakePosition, 0.0, 1.0));
            servoMovingIntake.setPosition(Range.clip(movingIntakePosition, 0.0, 1.0));

            // Telemetry
            telemetry.addData("Drive Motors", "FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f",
                    frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.addData("ArmUD Motor", "Power: %.2f, Position: %d", armUD.getPower(), armUD.getCurrentPosition());
            telemetry.addData("ArmEx Motor", "Power: %.2f", armEx.getPower());
            telemetry.addData("Intake Servo", "Position: %.2f", servoIntake.getPosition());
            telemetry.addData("Moving Intake Servo", "Position: %.2f", servoMovingIntake.getPosition());
            telemetry.update();
        }
    }
}
