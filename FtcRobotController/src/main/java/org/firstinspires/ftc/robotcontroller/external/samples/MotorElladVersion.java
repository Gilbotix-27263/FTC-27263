package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Motor Code(Ellad's Version)")
public class MotorElladVersion extends LinearOpMode {

    private DcMotor motor1 = null;
    private DcMotor motor2 = null;
    private DcMotor motor3 = null;
    private DcMotor motor4 = null;
    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    public void runOpMode() {
        // Put initialization blocks here.
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.

                //Forward--Backwards
                if(gamepad1.left_stick_y > 0)
                {
                    powerMotors(DcMotor.Direction.FORWARD, DcMotor.Direction.FORWARD, DcMotor.Direction.REVERSE, DcMotor.Direction.REVERSE, Math.abs(gamepad1.left_stick_y), Math.abs(gamepad1.left_stick_y), Math.abs(gamepad1.left_stick_y), Math.abs(gamepad1.left_stick_y));
                }
                if(gamepad1.left_stick_y < 0)
                {
                    powerMotors(DcMotor.Direction.REVERSE, DcMotor.Direction.REVERSE, DcMotor.Direction.FORWARD, DcMotor.Direction.FORWARD, Math.abs(gamepad1.left_stick_y), Math.abs(gamepad1.left_stick_y), Math.abs(gamepad1.left_stick_y), Math.abs(gamepad1.left_stick_y));
                }

                //Left--Right
                if(gamepad1.left_stick_x > 0)
                {
                    powerMotors(DcMotor.Direction.FORWARD, DcMotor.Direction.FORWARD, DcMotor.Direction.FORWARD, DcMotor.Direction.FORWARD, Math.abs(gamepad1.left_stick_x), Math.abs(gamepad1.left_stick_x), Math.abs(gamepad1.left_stick_x), Math.abs(gamepad1.left_stick_x));
                }
                if(gamepad1.left_stick_x < 0)
                {
                    powerMotors(DcMotor.Direction.REVERSE, DcMotor.Direction.REVERSE, DcMotor.Direction.REVERSE, DcMotor.Direction.REVERSE, Math.abs(gamepad1.left_stick_x), Math.abs(gamepad1.left_stick_x), Math.abs(gamepad1.left_stick_x), Math.abs(gamepad1.left_stick_x));
                }

                //Turn Left--Turn Right
                if(gamepad1.dpad_right)
                {
                    powerMotors(DcMotor.Direction.FORWARD, DcMotor.Direction.REVERSE, DcMotor.Direction.FORWARD, DcMotor.Direction.REVERSE, 1, 1, 1, 1);
                }
                if(gamepad1.dpad_left)
                {
                    powerMotors(DcMotor.Direction.REVERSE, DcMotor.Direction.FORWARD, DcMotor.Direction.REVERSE, DcMotor.Direction.FORWARD, 1, 1, 1, 1);
                }
                telemetry.update();
            }
        }
    }
    //A More Optimized Way To Power Motors(I Think)
    public void powerMotors(DcMotor.Direction dir1, DcMotor.Direction dir2, DcMotor.Direction dir3, DcMotor.Direction dir4, float pow1, float pow2, float pow3, float pow4)
    {
        motor1.setDirection(dir1);
        motor2.setDirection(dir2);
        motor3.setDirection(dir3);
        motor4.setDirection(dir4);

        motor1.setPower(pow1);
        motor2.setPower(pow2);
        motor3.setPower(pow3);
        motor4.setPower(pow4);
    }
}
