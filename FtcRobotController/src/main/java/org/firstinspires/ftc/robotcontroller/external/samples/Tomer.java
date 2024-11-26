package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "Tomer (k)")
public class Tomer extends LinearOpMode {

    private DcMotor motor1;

    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    public void runOpMode() {
        motor1 = hardwareMap.get(DcMotor.class, "motor1");

        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                if (gamepad1.left_stick_y > 0.1) {
                    //moveMotors(false, Math.abs(gamepad1.left_stick_x));
                    //System.out.println("Forward");
                    motor1.setDirection(DcMotor.Direction.FORWARD);
                    motor1.setPower(Math.abs(gamepad1.left_stick_y));
                }
                if (gamepad1.left_stick_y < -0.1) {
                   // moveMotors(true, Math.abs(gamepad1.left_stick_x));
                    // System.out.println("Backwards");
                    motor1.setDirection(DcMotor.Direction.REVERSE);
                    motor1.setPower(Math.abs(gamepad1.left_stick_y));
                }
                telemetry.update();
            }
        }
    }

    /**
     * Describe this function...
     */
    private void moveMotors(boolean my_1isReverse, float my_1Power) {
        if (!my_1isReverse) {
            motor1.setDirection(DcMotor.Direction.FORWARD);
        } else {
            motor1.setDirection(DcMotor.Direction.REVERSE);
        }
        motor1.setPower(my_1Power);
    }
}