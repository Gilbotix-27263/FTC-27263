package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(group = "Main")
public class FullRobotControl extends LinearOpMode {

    // Declare drive motors
    private DcMotor motor1, motor2, motor3, motor4;

    // Declare arm and intake components
    private DcMotor armUD, armEx;
    private CRServo servoIntakeLeft, servoIntakeRight; // CRServos for intake mechanism
    private Servo servoMovingIntake; // Servo for the moving intake component

    // REV Touch Sensor to detect the zero position of the arm extension
    private TouchSensor armExZeroSensor;

    // Speed control variables
    private double speedMultiplier = 1.0;
    private boolean isSlowMode = false;
    private ElapsedTime toggleTimer = new ElapsedTime();

    // Maximum power for arm motors
    private static final double MAX_ARM_POWER = 0.8;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize hardware components
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");
        armUD = hardwareMap.get(DcMotor.class, "armUD");
        armEx = hardwareMap.get(DcMotor.class, "arm");
        servoIntakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        servoIntakeRight = hardwareMap.get(CRServo.class, "intakeRight");
        servoMovingIntake = hardwareMap.get(Servo.class, "movingIntake");
        armExZeroSensor = hardwareMap.get(TouchSensor.class, "armExZeroSensor");

        // Configure motor directions for driving
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor3.setDirection(DcMotor.Direction.FORWARD);
        motor4.setDirection(DcMotor.Direction.REVERSE);

        // Reset and configure encoders for arm motors
        armUD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armUD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize toggle timer for speed control
        toggleTimer.reset();

        // Calibrate the arm extension motor to its zero position using the touch sensor
        telemetry.addData("Calibration", "Calibrating armEx to zero position...");
        telemetry.update();

        // Move armEx backward until the touch sensor is triggered
        while (!isStopRequested() && !armExZeroSensor.isPressed()) {
            armEx.setPower(0.2); // Move armEx slowly toward zero position
        }

        // Stop the motor and reset its encoder
        armEx.setPower(0.0);
        armEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Calibration", "Completed.");
        telemetry.update();

        // Set the moving intake servo to its initial position
        servoMovingIntake.setPosition(0.0);

        // Signal that the robot is ready
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the start signal
        waitForStart();

        while (opModeIsActive()) {

            // Handle speed toggle based on gamepad input
            if (gamepad1.left_bumper && toggleTimer.seconds() > 0.5) {
                isSlowMode = !isSlowMode;
                speedMultiplier = isSlowMode ? 0.3 : 1.0; // Toggle speed multiplier
                toggleTimer.reset();
            }

            // Calculate driving power for all wheels based on joystick input
            double drive = gamepad1.left_stick_y * speedMultiplier;
            double turn = gamepad1.right_stick_x * speedMultiplier;
            double strafe = gamepad1.left_stick_x * speedMultiplier;

            double frontLeftPower = drive + turn + strafe;
            double frontRightPower = -drive - turn + strafe;
            double backLeftPower = -drive + turn - strafe;
            double backRightPower = drive - turn - strafe;

            // Set power to drive motors
            motor1.setPower(frontLeftPower);
            motor2.setPower(frontRightPower);
            motor3.setPower(backLeftPower);
            motor4.setPower(backRightPower);

            // Control the up/down movement of the arm using the left joystick (gamepad2)
            double armUDPower = -gamepad2.left_stick_y * MAX_ARM_POWER;
            if (Math.abs(armUDPower) > 0.1) {
                armUD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armUD.setPower(armUDPower);
            } else {
                armUD.setPower(0.0);
            }

            // Control the arm extension using triggers (gamepad2)
            double armExPower = gamepad2.right_trigger - gamepad2.left_trigger;
            if (Math.abs(armExPower) > 0.1) {
                armEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armEx.setPower(armExPower * MAX_ARM_POWER);
            } else {
                armEx.setPower(0.0);
            }

            // Control the intake mechanism using buttons (gamepad2)
            if (gamepad2.a) {
                servoIntakeLeft.setPower(1.0); // Intake forward
                servoIntakeRight.setPower(-1.0); // Opposite direction
            } else if (gamepad2.b) {
                servoIntakeLeft.setPower(-1.0); // Intake backward
                servoIntakeRight.setPower(1.0); // Opposite direction
            } else {
                servoIntakeLeft.setPower(0.0); // Stop intake
                servoIntakeRight.setPower(0.0);
            }

            // Control the moving intake servo using bumpers (gamepad2)
            double movingIntakePosition = gamepad2.left_bumper ? 0.93 : (gamepad2.right_bumper ? 0.55 : 0.93);
            servoMovingIntake.setPosition(Range.clip(movingIntakePosition, 0.0, 1.0));

            // Display telemetry data for debugging
            telemetry.addData("Speed Mode", isSlowMode ? "Slow" : "Fast");
            telemetry.addData("Drive Motors", "FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f",
                    frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.addData("ArmUD Motor", "Power: %.2f, Position: %d", armUD.getPower(), armUD.getCurrentPosition());
            telemetry.addData("ArmEx Motor", "Power: %.2f, Position: %d", armEx.getPower(), armEx.getCurrentPosition());
            telemetry.addData("Intake Left CRServo", "Power: %.2f", servoIntakeLeft.getPower());
            telemetry.addData("Intake Right CRServo", "Power: %.2f", servoIntakeRight.getPower());
            telemetry.addData("Moving Intake Servo", "Position: %.2f", servoMovingIntake.getPosition());
            telemetry.addData("ArmEx Zero Sensor", "Pressed: %b", armExZeroSensor.isPressed());
            telemetry.update();
        }
    }
}
