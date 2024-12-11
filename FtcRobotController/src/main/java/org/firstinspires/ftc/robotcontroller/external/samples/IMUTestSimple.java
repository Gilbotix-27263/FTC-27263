package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class BHI260APIMUTest extends LinearOpMode {
    private BHI260IMU imu; // BHI260AP IMU

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing IMU...");
        telemetry.update();

        // Initialize the IMU
        imu = hardwareMap.get(BHI260IMU.class, "imu"); // Match the name in your hardware configuration

        // Wait for the game to start
        telemetry.addData("IMU Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Retrieve angular velocity (rate of change of orientation)
            AngularVelocity angularVelocity = imu.getAngularVelocity();

            // Retrieve orientation angles (yaw, pitch, roll)
            Orientation orientation = imu.getRobotOrientation(BHI260IMU.AxisReference.INTRINSIC, BHI260IMU.AxisOrder.ZYX);

            // Telemetry for angular velocity
            telemetry.addData("Angular Velocity - Yaw (deg/s)", "%.2f", angularVelocity.zRotationRate);
            telemetry.addData("Angular Velocity - Pitch (deg/s)", "%.2f", angularVelocity.xRotationRate);
            telemetry.addData("Angular Velocity - Roll (deg/s)", "%.2f", angularVelocity.yRotationRate);

            // Telemetry for orientation
            telemetry.addData("Orientation - Yaw (deg)", "%.2f", orientation.firstAngle);
            telemetry.addData("Orientation - Pitch (deg)", "%.2f", orientation.secondAngle);
            telemetry.addData("Orientation - Roll (deg)", "%.2f", orientation.thirdAngle);

            telemetry.update();
        }
    }
}
