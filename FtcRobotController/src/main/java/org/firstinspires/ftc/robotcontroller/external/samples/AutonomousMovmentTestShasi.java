package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(group = "Main")
public class AutonomousMovmentTestShasi extends LinearOpMode {

    private int Tile= 24;
    private DcMotor motor1; // Front Left
    private DcMotor motor3; // Back Left

    private IMU imu;
    private int UD_TICKS_PER_REV = 	1425;

    private int EX_TICKS_PER_REV = 	540;
    private int EX_FULL = 58 * (EX_TICKS_PER_REV/10);
    private int EX_TICKS_PER_INCH = EX_TICKS_PER_REV * (1/8);
    // Declare arm and intake components
    private DcMotor armUD, armEx;
    private CRServo servoIntakeLeft, servoIntakeRight; // CRServos for intake mechanism
    private Servo servoMovingIntake; // Servo for the moving intake component

    // REV Touch Sensor to detect the zero position of the arm extension
    private TouchSensor armExZeroSensor;
    private static final double TURN_SPEED = 0.3;
    private static final int COUNTS_PER_REV = 537; // Example: GoBILDA Yellow Jacket 312RPM
    private static final double WHEEL_DIAMETER = 4.0; // Wheel diameter in inches
    private static final double COUNTS_PER_INCH = (COUNTS_PER_REV) / (Math.PI * WHEEL_DIAMETER);



    @Override
    public void runOpMode() {
        // Initialize motors
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");





        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor3.setDirection(DcMotor.Direction.FORWARD);


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

        if (waitforopmode() == true){
            sequence();
        }


    }

    private boolean waitforopmode(){
        while (true){
            if (opModeIsActive()){
                return  true;
            }
            else{
                continue;
            }
        }
    }
    private void resetEncoders() {
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
        private void sequence(){


        Drive(55);
        resetEncoders();
        spinToAngle(90);


    }


    private void Drive(int inch){
        resetEncoders();


        motor1.setTargetPosition(((int) COUNTS_PER_INCH * inch) );
        motor3.setTargetPosition(((int) COUNTS_PER_INCH * inch));

        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor1.setPower(1);

        motor3.setPower(1);

        while (opModeIsActive() && motorsAreBusy()) {
            telemetry.addData("Path", "Straight");
            telemetry.update();
        }
        stopMotors();
}











    private void stopMotors() {
        motor1.setPower(0);

        motor3.setPower(0);

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
            motor3.setPower(-power);

            telemetry.addData("Target Angle", "%.2f", targetAngle);
            telemetry.addData("Current Angle", "%.2f", currentYaw);
            telemetry.addData("Error", "%.2f", error);
            telemetry.addData("Power", "%.2f", power);
            telemetry.update();
        }
    }
    private boolean motorsAreBusy() {
        return motor1.isBusy() ||  motor3.isBusy();
    }
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}
