package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "AprilTag and VisionPortal Test", group = "Samples")
public class WebcamTest extends LinearOpMode {

    private VisionPortal visionPortal;      // VisionPortal object
    private AprilTagProcessor aprilTagProcessor;  // AprilTag processor

    @Override
    public void runOpMode() {
        // Initialize AprilTag Processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)           // Draw the Tag ID
                .setDrawTagOutline(true)      // Draw the Tag outline
                .setDrawAxes(true)            // Draw axis lines
                .setDrawCubeProjection(true)  // Draw cube projection
                .build();

        // Initialize VisionPortal with the webcam and the AprilTagProcessor
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor) // Add the AprilTag Processor
                .build();

        telemetry.addData("Status", "Initialized and Ready to Start");
        telemetry.update();

        // Wait for the game to start
        waitForStart();

        while (opModeIsActive()) {
            // Fetch detections from the AprilTagProcessor
            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

            // Display how many tags are detected
            telemetry.addData("Number of Tags Detected", detections.size());

            for (AprilTagDetection tag : detections) {
                // Display AprilTag details
                telemetry.addData("Tag ID", tag.id);
                telemetry.addData("X Position (meters)", "%.2f", tag.ftcPose.x);
                telemetry.addData("Y Position (meters)", "%.2f", tag.ftcPose.y);
                telemetry.addData("Z Position (meters)", "%.2f", tag.ftcPose.z);
                telemetry.addData("Yaw (degrees)", "%.2f", tag.ftcPose.yaw);
                telemetry.addData("Pitch (degrees)", "%.2f", tag.ftcPose.pitch);
                telemetry.addData("Roll (degrees)", "%.2f", tag.ftcPose.roll);
                telemetry.addLine();
            }

            telemetry.update();
        }

        // Stop VisionPortal when done
        visionPortal.close();
    }
}