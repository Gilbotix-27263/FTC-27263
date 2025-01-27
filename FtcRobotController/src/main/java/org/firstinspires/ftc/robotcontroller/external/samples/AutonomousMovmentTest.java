package org.firstinspires.ftc.robotcontroller.external.samples;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


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
    private int UD_TICKS_PER_REV = 	1425;

    private int EX_TICKS_PER_REV = 	540;
    private int EX_FULL = 58 * (EX_TICKS_PER_REV/10);
    private int EX_TICKS_PER_INCH = EX_TICKS_PER_REV * (1/8);
    // Declare arm and intake components
    private DcMotor armUD;
    private CRServo servoIntakeLeft, servoIntakeRight; // CRServos for intake mechanism
    private Servo servoMovingIntake; // Servo for the moving intake component

    private  int currentgridx = 0;
    private int   getCurrentgridy = 0;

    // REV Touch Sensor to detect the zero position of the arm extension
    private static final double TURN_SPEED = 0.3;
    private static final int COUNTS_PER_REV = 537; // Example: GoBILDA Yellow Jacket 312RPM
    private static final double WHEEL_DIAMETER = 4.0; // Wheel diameter in inches
    private static final double COUNTS_PER_INCH = (COUNTS_PER_REV) / (Math.PI * WHEEL_DIAMETER);

    private DistanceSensor sensorDistance;

    private DcMotor armEx;
    private TouchSensor armExZeroSensor;
































    @Override
    public void runOpMode() {
        // Initialize motors
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");
        armUD = hardwareMap.get(DcMotor.class, "armUD");
        servoIntakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        servoIntakeRight = hardwareMap.get(CRServo.class, "intakeRight");
        servoMovingIntake = hardwareMap.get(Servo.class, "movingIntake");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
        armEx = hardwareMap.get(DcMotor.class, "arm");
        armExZeroSensor = hardwareMap.get(TouchSensor.class, "armExZeroSensor");



        armUD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armUD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        armEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);





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

        while (opModeIsActive()) {
            servoMovingIntake.setPosition(0.83);
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double currentaYaw = orientation.getYaw(AngleUnit.DEGREES);
            if (sensorDistance.getDistance(DistanceUnit.INCH)<=10){
                stopMotors();
                Side(2);
                resetEncoders();
                armEx.setPower(-1);
                if (armExZeroSensor.isPressed()){
                    armEx.setPower(0);
                }
                servoMovingIntake.setPosition(0.5);
                armEx.setPower(1);
                if (armEx.getCurrentPosition()<=-2500){
                    armEx.setPower(0);
                }
                ServoIntakes(true);


            }
            else {
                motor1.setPower(-0.8);
                motor2.setPower(0.8);
                motor3.setPower(0.8);
                motor4.setPower(-0.8);
            }






            telemetry.addData("deviceName", sensorDistance.getDeviceName() );
            telemetry.addData("range", String.format("%.01f mm", sensorDistance.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f m", sensorDistance.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f in", sensorDistance.getDistance(DistanceUnit.INCH)));

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
    private void armEX(int d){
        armEx.setTargetPosition(-d* EX_TICKS_PER_INCH);
        armEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive()){
            if (armExZeroSensor.isPressed()){
                armEx.setPower(0);
                armEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (armEx.getCurrentPosition()<=-5500){
                armEx.setPower(0);
            }
        }
    }



















    private void ArmUpDown(int rotations){

        armUD.setTargetPosition(-rotations* UD_TICKS_PER_REV);
        armUD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armUD.setPower(1);


        while (opModeIsActive()){
                if (armUD.getCurrentPosition()>1500){
                    armUD.setTargetPosition(1500);

            }

            telemetry.addData("Armud",armUD.getCurrentPosition());
            telemetry.update();
        }


    }

    private void ServoIntakes(boolean In){

        int inout = In ? 1:-1;

        servoIntakeRight.setPower(inout);
        servoIntakeLeft.setPower(-inout);


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
