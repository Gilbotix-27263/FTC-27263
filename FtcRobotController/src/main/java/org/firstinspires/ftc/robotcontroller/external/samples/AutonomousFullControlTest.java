package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous(group = "Main")
public class AutonomousFullControlTest extends LinearOpMode {
    // Declare hardware components
    private DcMotor motor1, motor2, motor3, motor4; // Drive motors
    private DcMotor armUD, armEx; // Arm motors for up/down and extension
    private CRServo servoIntakeLeft, servoIntakeRight; // Intake CRServos for picking up objects
    private Servo servoMovingIntake; // Servo for moving the intake mechanism
    private TouchSensor armExZeroSensor; // Sensor to detect zero position of arm extension
    private IMU imu; // IMU for orientation control
    private DistanceSensor distanceSensor; // Distance sensor for object detection (if needed)

    // Constants for robot movement and control
    double MOTOR_ENCODER_TICKS = 537;
    double WHEEL_RADIUS = 2;
    double TILE = 24;

    double ENCODER_TICKS_PER_INCH = MOTOR_ENCODER_TICKS / (2*Math.PI*WHEEL_RADIUS);


    private static final double MAX_ARM_POWER = 0.4; // Maximum power for arm motors

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

        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor3.setDirection(DcMotor.Direction.FORWARD);
        motor4.setDirection(DcMotor.Direction.REVERSE);

        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);




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
    private void Drive(int distance){
        resetEncoders();
        motor1.setTargetPosition(-distance*( (int) ENCODER_TICKS_PER_INCH));
        motor2.setTargetPosition(distance*( (int) ENCODER_TICKS_PER_INCH));
        motor3.setTargetPosition(distance*( (int) ENCODER_TICKS_PER_INCH));
        motor4.setTargetPosition(-distance*( (int) ENCODER_TICKS_PER_INCH));

        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);



    }
    private void Strafe(int distance){
        resetEncoders();
        motor1.setTargetPosition(-distance*( (int) ENCODER_TICKS_PER_INCH));
        motor2.setTargetPosition(-distance*( (int) ENCODER_TICKS_PER_INCH));
        motor3.setTargetPosition(distance*( (int) ENCODER_TICKS_PER_INCH));
        motor4.setTargetPosition(distance*( (int) ENCODER_TICKS_PER_INCH));

        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);



    }
}
