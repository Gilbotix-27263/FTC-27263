package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Movement", group = "Second Robot")
public class SecondRobotMovement extends LinearOpMode {
    DcMotor motor1, motor2;

    double multiplier = 0.3;
    private ElapsedTime toggleTimer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");

        motor1.setDirection(DcMotor.Direction.FORWARD);
        motor2.setDirection(DcMotor.Direction.FORWARD);

        toggleTimer.reset();

        waitForStart();

        boolean slowMode = true;

        while(opModeIsActive())
        {
            if (gamepad1.left_bumper && toggleTimer.seconds() > 0.5) {
                slowMode = !slowMode;
                multiplier = slowMode ? 0.3 : 1.0;
                toggleTimer.reset();
            }

            motor1.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x) * multiplier);
            motor2.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x) * multiplier);

            telemetry.addData("Speed Mode", slowMode ? "Slow" : "Fast");
            telemetry.addData("Motor1 Power", motor1.getPower());
            telemetry.addData("Motor2 Power", motor2.getPower());
            telemetry.update();
        }
    }
}
