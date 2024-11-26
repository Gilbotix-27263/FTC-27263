package org.firstinspires.ftc.robotcontroller.external.samples;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp

public class FTCFirstProgram extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor1 = null;
    private DcMotor motor2 = null;
    private DcMotor motor3 = null;
    private DcMotor motor4 = null;
    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");

        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.REVERSE);
        motor3.setDirection(DcMotor.Direction.FORWARD);
        motor4.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                double drive = -gamepad1.left_stick_y;
                double turn = gamepad1.right_stick_x;
                double sideDrive = gamepad1.left_stick_x;
                setPowerToMotors(drive, turn, sideDrive);
            }
        }
    }
    private void setDirection(double sideDrive, double turn, double drive) {
        if (sideDrive > 0.2) {
            motor1.setDirection(DcMotor.Direction.FORWARD);
            motor2.setDirection(DcMotor.Direction.REVERSE);
            motor3.setDirection(DcMotor.Direction.REVERSE);
            motor4.setDirection(DcMotor.Direction.REVERSE);
        }
        else if (turn < 0) {
        }
        else if (turn > 0) {
        }
        else{

        }
    }
    private void setPowerToMotors(double drive, double turn, double sideDrive){
        double motor1Pw;
        double motor2Pw;
        double motor3Pw;
        double motor4Pw;


        motor1Pw = Range.clip(drive + turn + sideDrive, -1.0, 1.0) ;
        motor2Pw = Range.clip(drive + turn + sideDrive, -1.0, 1.0) ;
        motor3Pw = Range.clip(drive + turn + sideDrive, -1.0, 1.0) ;
        motor4Pw = Range.clip(drive + turn + sideDrive, -1.0, 1.0) ;

        motor1.setPower(motor1Pw);
        motor2.setPower(motor2Pw);
        motor3.setPower(motor3Pw);
        motor4.setPower(motor4Pw);

        telemetry.addData("Status", "runtime: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f)", motor1Pw);
        telemetry.addData("Motors", "left (%.2f)", motor2Pw);
        telemetry.addData("Motors", "left (%.2f)", motor3Pw);
        telemetry.addData("Motors", "left (%.2f)", motor4Pw);
        telemetry.update();
    }


}
