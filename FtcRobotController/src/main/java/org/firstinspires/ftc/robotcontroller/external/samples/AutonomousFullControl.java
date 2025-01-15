package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(group = "Main")
public class AutonomousFullControl extends LinearOpMode {
    private DcMotor motor1, motor2, motor3, motor4; // Drive motors
    private DcMotor armUD, armEx; // Arm motors
    private CRServo servoIntakeLeft, servoIntakeRight; // Intake CRServos
    private Servo servoMovingIntake; // Moving intake servo
    private TouchSensor armExZeroSensor; // Arm extension zero position sensor
    private IMU imu; // IMU for orientation
    private DistanceSensor distanceSensor; // Distance sensor

    private static final double DRIVE_SPEED = 0.5;
    private static final double TURN_SPEED = 0.3;
    private static final int COUNTS_PER_REV = 537; // Encoder counts per revolution
    private static final double WHEEL_DIAMETER = 4.0; // Wheel diameter in inches
    private static final double COUNTS_PER_INCH = (COUNTS_PER_REV) / (Math.PI * WHEEL_DIAMETER);
    private static final double MAX_ARM_POWER = 0.4;

    @Override
    public void runOpMode() {
        // Initialize hardware
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
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        // Set motor directions for proper movement
        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor3.setDirection(DcMotor.Direction.FORWARD);
        motor4.setDirection(DcMotor.Direction.REVERSE);

        resetEncoders();
        servoMovingIntake.setPosition(0.1333);

        telemetry.addData("Status", "Ready to start");
        telemetry.update();
        waitForStart();

        // Autonomous sequence starting at 5A
        moveToGridCoordinate("5A", "6B"); // Move to game element near 6B/6C
        collectGameObject();               // Collect the game element

        startExtendArm();                  // Start extending arm early
        moveToGridCoordinate("6B", "6A"); // Move to the basket at 6A
        scoreGameObject();                 // Score the game element

        moveToGridCoordinate("6A", "5A"); // Return to starting position at 5A

        telemetry.addData("Status", "Sequence Complete");
        telemetry.update();
    }

    private void startExtendArm() {
        telemetry.addData("Action", "Extending arm early...");
        telemetry.update();

        new Thread(() -> {
            extendArm(400);
            moveArmToPosition(2000); // Move armUD to position 2000 simultaneously
        }).start(); // Extend arm asynchronously
    }

    private void moveToGridCoordinate(String start, String end) {
        int[] startPos = convertToGridPosition(start);
        int[] endPos = convertToGridPosition(end);
        moveToGridPosition(endPos[0], endPos[1]);
    }

    private int[] convertToGridPosition(String coordinate) {
        int y = Integer.parseInt(coordinate.substring(0, 1)) - 1; // Row to Y
        int x = coordinate.charAt(1) - 'A'; // Column to X
        return new int[] {x, y};
    }

    private void moveToGridPosition(int x, int y) {
        int deltaX = x * (int) (24 * COUNTS_PER_INCH); // X in inches
        int deltaY = y * (int) (24 * COUNTS_PER_INCH); // Y in inches

        // Move in Y direction first (rows)
        driveStraight(deltaY, DRIVE_SPEED);
        spinToAngle(0); // Adjust to move in X direction

        // Move in X direction (columns)
        driveStraight(deltaX, DRIVE_SPEED);
    }

    private void collectGameObject() {
        telemetry.addData("Action", "Collecting game object...");
        telemetry.update();

        moveArmToPosition(200);        // Lower the arm to collect position
        controlIntake(true, 2000);    // Intake forward for 2 seconds
        moveArmToPosition(0);         // Raise the arm back

        telemetry.addData("Action", "Game object collected");
        telemetry.update();
    }

    private void scoreGameObject() {
        telemetry.addData("Action", "Scoring game object...");
        telemetry.update();

        controlIntake(false, 2000);   // Reverse intake to release the game piece
        retractArm();                 // Retract arm back

        telemetry.addData("Action", "Game object scored");
        telemetry.update();
    }

    private void moveArmToPosition(int targetPosition) {
        armUD.setTargetPosition(targetPosition);
        armUD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armUD.setPower(MAX_ARM_POWER);

        while (opModeIsActive() && armUD.isBusy()) {
            telemetry.addData("Arm Position", armUD.getCurrentPosition());
            telemetry.update();
        }
    }

    private void extendArm(int encoderCounts) {
        if (armExZeroSensor.isPressed()) {
            armEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        armEx.setTargetPosition(encoderCounts);
        armEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armEx.setPower(MAX_ARM_POWER);

        while (opModeIsActive() && armEx.isBusy()) {
            telemetry.addData("ArmEx Position", armEx.getCurrentPosition());
            telemetry.update();
        }
    }

    private void retractArm() {
        extendArm(0);
    }

    private void controlIntake(boolean forward, long duration) {
        double power = forward ? 1.0 : -1.0;
        servoIntakeLeft.setPower(power);
        servoIntakeRight.setPower(-power);

        sleep(duration);

        servoIntakeLeft.setPower(0);
        servoIntakeRight.setPower(0);
    }

    private void resetEncoders() {
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private boolean motorsAreBusy() {
        return motor1.isBusy() && motor2.isBusy() && motor3.isBusy() && motor4.isBusy();
    }

    private void stopMotors() {
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    private void driveStraight(int encoderCounts, double speed) {
        resetEncoders();

        motor1.setTargetPosition(encoderCounts);
        motor2.setTargetPosition(encoderCounts);
        motor3.setTargetPosition(encoderCounts);
        motor4.setTargetPosition(encoderCounts);

        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor1.setPower(speed);
        motor2.setPower(-speed);
        motor3.setPower(-speed);
        motor4.setPower(speed);

        while (opModeIsActive() && motorsAreBusy()) {
            telemetry.addData("Path", "Driving straight");
            telemetry.update();
        }
        stopMotors();
    }

    private void spinToAngle(double targetAngle) {
        while (opModeIsActive()) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double currentYaw = orientation.getYaw(AngleUnit.DEGREES);
            double error = normalizeAngle(targetAngle - currentYaw);

            if (Math.abs(error) < 1.0) { // Deadband for stopping
                stopMotors();
                break;
            }

            double power = error * 0.02; // Proportional control
            power = Range.clip(power, -TURN_SPEED, TURN_SPEED);

            motor1.setPower(-power);
            motor2.setPower(power);
            motor3.setPower(-power);
            motor4.setPower(power);

            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Current Angle", currentYaw);
            telemetry.addData("Error", error);
            telemetry.update();
        }
    }
}
