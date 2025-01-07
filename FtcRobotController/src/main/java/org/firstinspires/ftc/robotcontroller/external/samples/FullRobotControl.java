package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(group = "Main")
public class FullRobotControl extends LinearOpMode {
    // Drive motors
    private DcMotor motor1, motor2, motor3, motor4;

    // Arm and intake motors/servos
    private DcMotor armUD, armEx;
    private CRServo servoIntake; // CRServo for continuous rotation
    private Servo servoMovingIntake;

    // Speed control
    private double speedMultiplier = 1.0;
    private boolean isSlowMode = false;
    private ElapsedTime toggleTimer = new ElapsedTime();

    // Constants
    private static final double MAX_ARM_POWER = 0.8;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");
        armUD = hardwareMap.get(DcMotor.class, "armUD");
        armEx = hardwareMap.get(DcMotor.class, "arm");
        servoIntake = hardwareMap.get(CRServo.class, "intake"); // CRServo initialization
        servoMovingIntake = hardwareMap.get(Servo.class, "movingIntake");

        // Configure motors
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor3.setDirection(DcMotor.Direction.FORWARD);
        motor4.setDirection(DcMotor.Direction.REVERSE);

        armUD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armUD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int armUDTargetPosition = armUD.getCurrentPosition();
        int armExTargetPosition = armEx.getCurrentPosition();

        toggleTimer.reset();

        // Set the servoMovingIntake to its 0 position
        double zeroPosition = 0.0; // Define the zero position value
        servoMovingIntake.setPosition(zeroPosition);

        // Telemetry setup
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for start signal
        waitForStart();

        while (opModeIsActive()) {
            // --- Speed Control ---
            if (gamepad1.left_bumper && toggleTimer.seconds() > 0.5) { // Debounce for toggle
                isSlowMode = !isSlowMode;
                speedMultiplier = isSlowMode ? 0.3 : 1.0; // Adjust speed multiplier
                toggleTimer.reset();
            }

            // --- Controller 1: Driving ---
            double drive = gamepad1.left_stick_y * speedMultiplier;
            double turn = gamepad1.right_stick_x * speedMultiplier;
            double strafe = gamepad1.left_stick_x * speedMultiplier;

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
            if (Math.abs(armUDPower) > 0.1) {
                armUD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armUD.setPower(armUDPower);
                armUDTargetPosition = armUD.getCurrentPosition();
            } else {
                armUD.setTargetPosition(armUDTargetPosition);
                armUD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armUD.setPower(0.5); // Adjust holding power as needed
            }

            // ArmEx movement (manual control)
            double armExPower = gamepad2.right_trigger - gamepad2.left_trigger;
            if (Math.abs(armExPower) > 0.1) {
                armEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armEx.setPower(armExPower * MAX_ARM_POWER);
                armExTargetPosition = armEx.getCurrentPosition();
            } else {
                armEx.setTargetPosition(armExTargetPosition);
                armEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armEx.setPower(0.5); // Adjust holding power as needed
            }

            // CRServo control for intake
            if (gamepad2.a) {
                servoIntake.setPower(1.0); // Rotate forward
            } else if (gamepad2.b) {
                servoIntake.setPower(-1.0); // Rotate backward
            } else {
                servoIntake.setPower(0.0); // Stop rotation
            }

            // Servo control for moving intake
            double movingIntakePosition = gamepad2.left_bumper ? 0.0 : (gamepad2.right_bumper ? 1.0 : 0.5);
            servoMovingIntake.setPosition(Range.clip(movingIntakePosition, 0.0, 1.0));

            // Telemetry
            telemetry.addData("Speed Mode", isSlowMode ? "Slow" : "Fast");
            telemetry.addData("Drive Motors", "FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f",
                    frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.addData("ArmUD Motor", "Power: %.2f, Position: %d", armUD.getPower(), armUD.getCurrentPosition());
            telemetry.addData("ArmEx Motor", "Power: %.2f, Position: %d", armEx.getPower(), armEx.getCurrentPosition());
            telemetry.addData("Intake CRServo", "Power: %.2f", servoIntake.getPower());
            telemetry.addData("Moving Intake Servo", "Position: %.2f", servoMovingIntake.getPosition());
            telemetry.update();
        }
    }
}
