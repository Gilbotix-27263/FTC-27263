package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp
public class IMUTestSimple extends LinearOpMode {
    // IMU sensor instance
    IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the IMU using the hardware map
        imu = hardwareMap.get(IMU.class, "imu");

        // Define the hub orientation: logo and USB directions
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        // Set hub orientation
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Initialize IMU with the defined orientation
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Wait for the game to start
        telemetry.addData("Status", "Waiting for start");
        telemetry.update();

        // Main loop
        while (!isStopRequested()) {
            // Display hub orientation
            telemetry.addData("Hub Orientation", "Logo=%s, USB=%s", logoDirection, usbDirection);

            // Reset yaw if the Y button is pressed
            if (gamepad1.y) {
                telemetry.addData("Yaw", "Resetting...");
                imu.resetYaw();
            } else {
                telemetry.addData("Yaw", "Press Y to reset");
            }

            // Add an empty line between the reset yaw and XYZ telemetry
            telemetry.addLine();

            // Get current yaw, pitch, roll, and angular velocity
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

            // Combine position and velocity for X, Y, Z
            telemetry.addData("X Axis", "Pitch: %.2f° | Velocity: %.2f°/s",
                    orientation.getPitch(AngleUnit.DEGREES), angularVelocity.xRotationRate);
            telemetry.addData("Y Axis", "Roll: %.2f° | Velocity: %.2f°/s",
                    orientation.getRoll(AngleUnit.DEGREES), angularVelocity.yRotationRate);
            telemetry.addData("Z Axis", "Yaw: %.2f° | Velocity: %.2f°/s",
                    orientation.getYaw(AngleUnit.DEGREES), angularVelocity.zRotationRate);

            telemetry.update();
        }
    }
}
