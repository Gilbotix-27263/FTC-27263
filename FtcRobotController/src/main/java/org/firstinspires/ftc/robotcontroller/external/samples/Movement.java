package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Movement extends LinearOpMode{

    private DcMotor motor1 = null;
    private DcMotor motor2 = null;
    private DcMotor motor3 = null;
    private DcMotor motor4 = null;

    /*@Override
    public void runOpMode()
    {
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");

        waitForStart();

        if(opModeIsActive())
        {
            while(opModeIsActive())
            {
                if (gamepad1.left_stick_y > 0.1) {
                    motor1.setDirection(DcMotor.Direction.FORWARD);
                    motor2.setDirection(DcMotor.Direction.FORWARD);
                    motor3.setDirection(DcMotor.Direction.REVERSE);
                    motor4.setDirection(DcMotor.Direction.REVERSE);

                    motor1.setPower(Math.abs(gamepad1.left_stick_y));
                    motor2.setPower(Math.abs(gamepad1.left_stick_y));
                    motor3.setPower(Math.abs(gamepad1.left_stick_y));
                    motor4.setPower(Math.abs(gamepad1.left_stick_y));
                }
                telemetry.update();
            }
        }
    }*/

    @Override
    public void runOpMode() {
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                if (gamepad1.left_stick_y > 0.1) {
                    motor1.setDirection(DcMotor.Direction.FORWARD);
                    motor2.setDirection(DcMotor.Direction.FORWARD);
                    motor3.setDirection(DcMotor.Direction.REVERSE);
                    motor4.setDirection(DcMotor.Direction.REVERSE);

                    motor1.setPower(Math.abs(gamepad1.left_stick_y));
                    motor2.setPower(Math.abs(gamepad1.left_stick_y));
                    motor3.setPower(Math.abs(gamepad1.left_stick_y));
                    motor4.setPower(Math.abs(gamepad1.left_stick_y));
                }
                telemetry.update();
            }
        }
    }
}
