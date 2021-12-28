package org.firstinspires.ftc.teamcode.demobots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;

@TeleOp(name="Limit switch Test",group="")
public class LimitSwitchTest extends LinearOpMode
{

    DigitalChannel limitSwitch;

    @Override
    public void runOpMode() throws InterruptedException
    {
        limitSwitch = hardwareMap.digitalChannel.get("Switch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        waitForStart();
        while(opModeIsActive())
        {

            telemetry.addData("isPressed",limitSwitch.getState());
            telemetry.update();

        }
    }
}
