package org.firstinspires.ftc.teamcode.demobots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogOutput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Pintank",group="teleop")
public class Pintank extends LinearOpMode
{

    AnalogInput irSensor;
    DigitalChannel irBeacon;

    @Override
    public void runOpMode() throws InterruptedException
    {

        irBeacon=hardwareMap.digitalChannel.get("beacon");

        waitForStart();
        boolean toggle = false;
        ElapsedTime e = new ElapsedTime();
        e.startTime();
        while(opModeIsActive())
        {

            irBeacon.setMode(DigitalChannel.Mode.OUTPUT);
            irBeacon.setState(toggle);
            if(e.seconds()>.2)
            {

                toggle=!toggle;
                e.reset();

            }


        }

    }
}
