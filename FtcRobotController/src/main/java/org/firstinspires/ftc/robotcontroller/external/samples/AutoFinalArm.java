package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous(group = "Test Stuff IDK", name = "Autonomous Arm Test")
public class AutoFinalArm extends LinearOpMode {
    private TouchSensor armZeroSensor;
    private DcMotor armUD, armEx;

    @Override
    public void runOpMode() throws InterruptedException {
        armZeroSensor = hardwareMap.get(TouchSensor.class, "armExZeroSensor");
        armUD = hardwareMap.get(DcMotor.class, "armUD");
        armEx = hardwareMap.get(DcMotor.class, "arm");

        waitForStart();

        while(opModeIsActive())
        {

        }
    }

    private void moveUD()
    {

    }

    private void moxeEx()
    {
        
    }
}
