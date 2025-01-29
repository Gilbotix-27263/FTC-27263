package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous(group = "Test Stuff IDK", name = "Autonomous Arm Test")
public class AutoFinalArm extends LinearOpMode {
    private TouchSensor armZeroSensor;
    private DcMotor armUD, armEx;
    private CRServo servoIntakeLeft, servoIntakeRight;

    @Override
    public void runOpMode() throws InterruptedException {
        //Me When Copy-Paste LOL
        armZeroSensor = hardwareMap.get(TouchSensor.class, "armExZeroSensor");
        armUD = hardwareMap.get(DcMotor.class, "armUD");
        armEx = hardwareMap.get(DcMotor.class, "arm");
        servoIntakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        servoIntakeRight = hardwareMap.get(CRServo.class, "intakeRight");

        armUD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armUD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armUD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Enable brake mode for armUD

        armEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while(opModeIsActive())
        {

        }
    }

    private void moveUD()
    {
        telemetry.addData("Action", "MoveUD");
    }

    private void moveEx()
    {
        telemetry.addData("Action", "MoveEx");
    }

    private void pickUp()
    {
        telemetry.addData("Action", "Pick Up");
    }

    private void release()
    {
        telemetry.addData("Action", "Release");
    }
}
