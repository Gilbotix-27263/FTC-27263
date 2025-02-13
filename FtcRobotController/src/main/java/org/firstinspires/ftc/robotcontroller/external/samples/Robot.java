package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org. firstinspires. ftc. robotcore. external. Telemetry;

public class Robot {
    private DriveTrain driveTrain;
    private ArmSystem armControl;
    private IntakeSystem intakeSystem;
    private LEDIndicator ledIndicator;
    private LinearOpMode opMode;


    public Robot(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode) {
        driveTrain = new DriveTrain(hardwareMap);
        armControl = new ArmSystem(hardwareMap);
        intakeSystem = new IntakeSystem(hardwareMap);
        ledIndicator = new LEDIndicator(hardwareMap);
        this.opMode = opMode;
    }

    public void initialize() {
        driveTrain.initialize();
        armControl.initialize();
        intakeSystem.initialize();
        ledIndicator.initialize();
    }

    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        driveTrain.control(gamepad1);
        armControl.control(gamepad2);
        intakeSystem.control(gamepad2);
        ledIndicator.control(gamepad1, opMode.opModeIsActive());
    }
}
