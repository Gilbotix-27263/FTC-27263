package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Distance Sensor Shit IDK", group = "Second Robot")
public class SecondRobotStuff extends LinearOpMode {
    DcMotor motor1, motor2;
    DistanceSensor dis;
    @Override
    public void runOpMode() throws InterruptedException {
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        dis = hardwareMap.get(DistanceSensor.class, "sensor_distance");

        waitForStart();

        while(opModeIsActive())
        {
            if(dis.getDistance(DistanceUnit.CM) > 20)
            {
                motor1.setDirection(DcMotor.Direction.REVERSE);
                motor2.setDirection(DcMotor.Direction.FORWARD);
            }else
            {
                motor1.setDirection(DcMotor.Direction.REVERSE);
                motor2.setDirection(DcMotor.Direction.REVERSE);
            }

            motor1.setPower(0.5
            );
            motor2.setPower(0.5);

            telemetry.addData("Distance", dis.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
