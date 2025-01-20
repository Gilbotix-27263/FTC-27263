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
public class AutonomousTest extends LinearOpMode {
    private DcMotor motor1; // Front Left
    private DcMotor motor2; // Front Right
    private DcMotor motor3; // Back Left
    private DcMotor motor4; // Back Right
    private IMU imu;


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
        while (opModeIsActive()){
            Drive(5);
        }
    }


    private void Drive(int inch){
        motor1.setTargetPosition(-((int) COUNTS_PER_INCH * inch) );
        motor2.setTargetPosition(((int) COUNTS_PER_INCH * inch));
        motor3.setTargetPosition(((int) COUNTS_PER_INCH * inch));
        motor4.setTargetPosition(-((int) COUNTS_PER_INCH * inch));

        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
}
