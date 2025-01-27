package org.firstinspires.ftc.robotcontroller.external.samples;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
public class Armtest extends LinearOpMode {
    private DcMotor armUD;

    public void runOpMode(){
        armUD = hardwareMap.get(DcMotor.class,"armUD");



        while (opModeIsActive()){
            armUD.setPower(1);

        }
    }
}
