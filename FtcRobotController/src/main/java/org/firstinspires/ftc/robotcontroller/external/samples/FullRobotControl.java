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

    // Declare drive motors (Controller 1)
    private DcMotor motor1, motor2, motor3, motor4;

    // Declare arm and intake components (Controller 2)
    private DcMotor armUD, armEx;
    private CRServo servoIntakeLeft, servoIntakeRight; // CRServos for intake mechanism
    private Servo servoMovingIntake; // Servo for the moving intake component

    // REV Touch Sensor to detect the zero position of the arm extension
    private TouchSensor armExZeroSensor;

    // Speed control variables
    private double speedMultiplier = 1.0;
    private boolean isSlowMode = false;
    private ElapsedTime toggleTimer = new ElapsedTime();
    private ElapsedTime zeroDelayTimer = new ElapsedTime(); // Timer for zeroing delay

    // Maximum power for arm motors
    private static final double MAX_ARMUD_POWER = 0.4;
    private static final double MAX_ARMEX_POWER = 1;

    private static final int ARMUD_MAX_POSITION = -2100;
    private static final double ZERO_DELAY = 1.0; // Minimum delay in seconds between zeroing actions

    // Arm state flag
    private boolean armExZeroed = false;

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
        armUD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Enable brake mode for armUD

        armEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize toggle timer for speed control
        toggleTimer.reset();
        zeroDelayTimer.reset(); // Initialize zeroing delay timer

        // Set the moving intake servo to its initial position
        servoMovingIntake.setPosition(0.1333);

        // Zero the armEx at the start of the program
        telemetry.addData("Status", "Zeroing ArmEx...");
        telemetry.update();

        zeroDelayTimer.reset();
        while (!armExZeroSensor.isPressed() && !isStopRequested()) {
            armEx.setPower(0.2); // Move the arm down slowly
        }

        if (!armExZeroed && armExZeroSensor.isPressed()) {
            armEx.setPower(0.0); // Stop the motor once zeroed
            armEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            zeroDelayTimer.reset(); // Reset the timer after zeroing
            armExZeroed = true; // Set zeroed flag
        }

        telemetry.addData("Status", "ArmEx Zeroed");
        telemetry.update();

        // Signal that the robot is ready
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Track the target positions for armUD and armEx
        int armUDTargetPosition = armUD.getCurrentPosition();
        int armExTargetPosition = armEx.getCurrentPosition();

        // Wait for the start signal
        waitForStart();

        while (opModeIsActive()) {

            // Driving controls (Controller 1)
            if (gamepad1.left_bumper && toggleTimer.seconds() > 0.5) {
                isSlowMode = !isSlowMode;
                speedMultiplier = isSlowMode ? 0.3 : 1.0; // Toggle speed multiplier
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

            // Arm and intake controls (Controller 2)

            // Reset armEx encoder whenever the zero sensor is pressed, with delay and flag check
            if (armExZeroSensor.isPressed() && zeroDelayTimer.seconds() > ZERO_DELAY && !armExZeroed) {
                armEx.setPower(0.0); // Ensure the motor stops immediately
                armEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                zeroDelayTimer.reset(); // Reset delay timer after resetting encoder
                armExZeroed = true; // Prevent further movement
            } else if (!armExZeroSensor.isPressed()) {
                armExZeroed = false; // Reset the flag if the sensor is released
            }

            // Prevent moving the armEx further back after it is zeroed
            if (armExZeroed && gamepad2.left_trigger > 0.1) {
                armEx.setPower(0.0); // Block backward movement
            } else {
                // Control the arm extension using triggers (gamepad2)
                double armExPower = gamepad2.right_trigger - gamepad2.left_trigger;
                int armExCurrentPosition = armEx.getCurrentPosition();

                if (Math.abs(armExPower) > 0.1) {
                    // Prevent movement beyond limits
                    if ((armExCurrentPosition <= -2150 && armExPower < 0) || (armExCurrentPosition >= 0 && armExPower > 0)) {
                        armEx.setPower(0.0); // Stop movement if out of range
                    } else {
                        armEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        armEx.setPower(armExPower * MAX_ARMEX_POWER);
                        armExTargetPosition = armEx.getCurrentPosition();
                    }
                } else {
                    armEx.setTargetPosition(armExTargetPosition);
                    armEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armEx.setPower(0.5); // Holding power
                }
            }

            double armUDPower = gamepad2.left_stick_y * MAX_ARMUD_POWER;
            int armUDCurrentPosition = armUD.getCurrentPosition();

            if (Math.abs(armUDPower) > 0.1) {
                // Prevent movement beyond limits
                if ((armUDCurrentPosition >= ARMUD_MAX_POSITION && armUDPower < 0) || (armUDCurrentPosition <= 0 && armUDPower > 0)) {
                    armUD.setPower(0.0); // Stop movement if out of range
                } else {
                    armUD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    armUD.setPower(armUDPower);
                }
            } else {
                armUD.setTargetPosition(armUDCurrentPosition);
                armUD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armUD.setPower(0.5); // Holding power
            }

            // Intake mechanism controls (gamepad2)
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

            // Moving intake servo controls (gamepad2)
            double movingIntakePosition = gamepad2.left_bumper ? 0.1333 : (gamepad2.right_bumper ? 0.8333 : 0.5);
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
