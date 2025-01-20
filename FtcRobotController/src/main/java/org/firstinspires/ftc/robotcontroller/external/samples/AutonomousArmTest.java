package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class AutonomousArmTest extends LinearOpMode {


    private int UD_TICKS_PER_REV = 	1425;
    // Declare arm and intake components
    private DcMotor armUD, armEx;
    private CRServo servoIntakeLeft, servoIntakeRight; // CRServos for intake mechanism
    private Servo servoMovingIntake; // Servo for the moving intake component

    // REV Touch Sensor to detect the zero position of the arm extension
    private TouchSensor armExZeroSensor;

    public void runOpMode() throws InterruptedException {

        // Initialize hardware components
        armUD = hardwareMap.get(DcMotor.class, "armUD");
        armEx = hardwareMap.get(DcMotor.class, "arm");
        servoIntakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        servoIntakeRight = hardwareMap.get(CRServo.class, "intakeRight");
        servoMovingIntake = hardwareMap.get(Servo.class, "movingIntake");
        armExZeroSensor = hardwareMap.get(TouchSensor.class, "armExZeroSensor");

        armUD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armUD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armUD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        while (opModeIsActive()){
            Sequence();
        }
    }
    private void Sequence(){
        ArmUpDown(4);
    }
    private void ArmUpDown(int rotations){
        armUD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armUD.setTargetPosition(rotations * UD_TICKS_PER_REV);
        armUD.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (opModeIsActive() && (armUD.isBusy())){
            telemetry.addData("Armud",armUD.getCurrentPosition());
            telemetry.update();
        }


    }



}
