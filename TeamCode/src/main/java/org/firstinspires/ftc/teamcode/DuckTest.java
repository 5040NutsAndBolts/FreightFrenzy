package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@Autonomous(name="Duck SPIN", group="auto")
public class DuckTest extends LinearOpMode
{


    public static int duckspeed=0;
    DcMotorEx motor;


    @Override
    public void runOpMode() throws InterruptedException
    {

        motor=hardwareMap.get(DcMotorEx.class,"motor1");
        motor.setVelocityPIDFCoefficients(65,.4, 10,0);
        waitForStart();
        while(opModeIsActive())
        {

            motor.setVelocity(DashConfig.duckVel);
            telemetry.addData("Speed",motor.getVelocity());
            telemetry.update();

        }


    }
}
