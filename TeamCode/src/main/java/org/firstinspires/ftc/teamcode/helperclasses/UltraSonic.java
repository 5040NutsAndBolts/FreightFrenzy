package org.firstinspires.ftc.teamcode.helperclasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

@TeleOp (name="UltraSonic", group = "Teleop")
public class UltraSonic extends LinearOpMode {
    public AnalogInput distanceSensor;

    @Override
    public void runOpMode() throws InterruptedException
    {
        distanceSensor = hardwareMap.get(AnalogInput.class, "Distance Sensor");

        waitForStart();

        while(opModeIsActive())
        {
           telemetry.addData("voltage", distanceSensor.getVoltage());
           telemetry.addData("inches", distanceSensor.getVoltage() / 6.4 * 1000 * 2);
           telemetry.update();
        }

    }
}
