package org.firstinspires.ftc.robotcontroller.external.samples;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "RoadRunnerTest")
public class RoadRunnerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(
                drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                        .forward(24)
                        .strafeRight(12)
                        .build()
        );

        telemetry.addData("Status", "Trajectory complete");
        telemetry.update();
    }
}
