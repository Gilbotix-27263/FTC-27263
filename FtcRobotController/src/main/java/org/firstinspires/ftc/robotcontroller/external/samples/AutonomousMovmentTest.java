package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(group = "Main")
public class AutonomousMovmentTest extends LinearOpMode {

    private int Tile= 24;
    private DcMotor motor1; // Front Left
    private DcMotor motor2; // Front Right
    private DcMotor motor3; // Back Left
    private DcMotor motor4; // Back Right
    private IMU imu;

    private static final double TURN_SPEED = 0.3;
    private static final int COUNTS_PER_REV = 537; // Example: GoBILDA Yellow Jacket 312RPM
    private static final double WHEEL_DIAMETER = 4.0; // Wheel diameter in inches
    private static final double COUNTS_PER_INCH = (COUNTS_PER_REV) / (Math.PI * WHEEL_DIAMETER);



    @Override
    public void runOpMode() {
        // Initialize motors
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");

        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor3.setDirection(DcMotor.Direction.FORWARD);
        motor4.setDirection(DcMotor.Direction.REVERSE);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        // Reset encoders


        telemetry.addData("Status", "Ready to start");
        telemetry.update();
        waitForStart();






        telemetry.addData("Status", "Sequence Complete");
        telemetry.update();

        while(opModeIsActive()){
            sequence();

        }


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
    private void sequence(){

        Side(-5 * Tile);
        resetEncoders();
        Drive(4 * Tile);
        resetEncoders();
        spinToAngle(90);
        resetEncoders();
        Drive(5 * Tile);

    }
    private void Drive(int inch){
        resetEncoders();

        motor1.setTargetPosition(-((int) COUNTS_PER_INCH * inch) );
        motor2.setTargetPosition(((int) COUNTS_PER_INCH * inch));
        motor3.setTargetPosition(((int) COUNTS_PER_INCH * inch));
        motor4.setTargetPosition(-((int) COUNTS_PER_INCH * inch));

        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor1.setPower(1);
        motor2.setPower(1);
        motor3.setPower(1);
        motor4.setPower(1);

        while (opModeIsActive() && motorsAreBusy()) {
            telemetry.addData("Path", "Straight");
            telemetry.update();
        }
        stopMotors();
}

    private void Side(int inch){
        resetEncoders();
        motor1.setTargetPosition(-((int) COUNTS_PER_INCH * inch) );
        motor2.setTargetPosition(-((int) COUNTS_PER_INCH * inch));
        motor3.setTargetPosition(((int) COUNTS_PER_INCH * inch));
        motor4.setTargetPosition(((int) COUNTS_PER_INCH * inch));

        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor1.setPower(1);
        motor2.setPower(1);
        motor3.setPower(1);
        motor4.setPower(1);
        while (opModeIsActive() && motorsAreBusy()) {
            telemetry.addData("Path", "Strafing");
            telemetry.update();
        }
        stopMotors();
    }


    private void stopMotors() {
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
    }
    private void spinToAngle(double targetAngle) {
        while (opModeIsActive()) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double currentYaw = orientation.getYaw(AngleUnit.DEGREES);
            double error = normalizeAngle(targetAngle - currentYaw);

            if (Math.abs(error) <= 0.09) { // Deadband of 1 degree
                stopMotors();
                break;
            }

            double power = error * 0.08; // Proportional control
            power = Range.clip(power, -TURN_SPEED, TURN_SPEED);

            motor1.setPower(power);
            motor2.setPower(-power);
            motor3.setPower(power);
            motor4.setPower(-power);

            telemetry.addData("Target Angle", "%.2f", targetAngle);
            telemetry.addData("Current Angle", "%.2f", currentYaw);
            telemetry.addData("Error", "%.2f", error);
            telemetry.addData("Power", "%.2f", power);
            telemetry.update();
        }
    }
    private boolean motorsAreBusy() {
        return motor1.isBusy() || motor2.isBusy() || motor3.isBusy() || motor4.isBusy();
    }
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}
