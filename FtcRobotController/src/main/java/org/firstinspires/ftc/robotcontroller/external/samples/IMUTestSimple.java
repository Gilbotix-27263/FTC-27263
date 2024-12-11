package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class IMUTestSimple extends LinearOpMode {
    private BNO055IMU imu;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing IMU...");
        telemetry.update();

        imu = hardwareMap.get(BNO055IMU.class, "imui");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu.initialize(parameters);

        if (imu.isGyroCalibrated()) {
            telemetry.addData("IMU Status", "Calibrated");
        } else {
            telemetry.addData("IMU Status", "Not Calibrated");
        }
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("IMU Yaw", "%.2f", imu.getAngularOrientation().firstAngle);
            telemetry.update();
        }
    }
}