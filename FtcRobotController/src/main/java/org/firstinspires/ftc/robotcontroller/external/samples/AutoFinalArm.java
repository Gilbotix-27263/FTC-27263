//Please Delete The "addData" In The Final Version. These Are Just For Testing

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

        int e = 1;
        double s = 0.5;

        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.a)
            {
                moveArm(true, e, s);
            }else
            {
                armUD.setPower(0);
            }

            //Change
            if(gamepad1.b)
            {
                moveArm(false, e, s);
            }else
            {
                armEx.setPower(0);
            }

            if(gamepad1.x)
            {
                pickUp();
            }else if(!gamepad1.y)
            {
                servoIntakeLeft.setPower(0);
                servoIntakeRight.setPower(0);
            }

            if(gamepad1.y)
            {
                release();
            }else if(!gamepad1.x)
            {
                servoIntakeLeft.setPower(0);
                servoIntakeRight.setPower(0);
            }

            if(gamepad1.dpad_right)
            {
                e++;
            }else if(gamepad1.dpad_left)
            {
                e--;
            }

            if(gamepad1.dpad_up)
            {
                s += 0.1;
            }else if(gamepad1.dpad_down)
            {
                s -= 0.1;
            }
        }
    }

    private void moveArm(boolean Ud, int encoderCounts, double speed) {
        telemetry.addData("Action", "MoveArm | " + Ud + ", " + encoderCounts + ", " + speed);

        if(Ud)
        {
            armUD.setTargetPosition(encoderCounts);
            armUD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armUD.setPower(speed);
        }else
        {
            if(!armZeroSensor.isPressed())
            {
                armEx.setTargetPosition(encoderCounts);
                armEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armEx.setPower(speed);
            }else
            {
                return;
            }
        }
    }

    private void pickUp()
    {
        telemetry.addData("Action", "Pick Up");

        servoIntakeLeft.setPower(1.0);
        servoIntakeRight.setPower(-1.0);
    }

    private void release()
    {
        telemetry.addData("Action", "Release");

        servoIntakeLeft.setPower(-1.0);
        servoIntakeRight.setPower(1.0);
    }

    private void stopAll()
    {
        armUD.setPower(0);
        armEx.setPower(0);
        servoIntakeLeft.setPower(0);
        servoIntakeRight.setPower(0);
    }
}
