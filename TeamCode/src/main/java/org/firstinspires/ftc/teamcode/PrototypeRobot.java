package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="",group="Teleop")
public class PrototypeRobot extends LinearOpMode
{

    //The linear slide motors
    DcMotor leftDepositSlide;
    DcMotor rightDepositSlide;

    @Override
    public void runOpMode() throws InterruptedException
    {

        //Slide intake motor controller set up
        //Naming proper name used on driver hub
        leftDepositSlide = hardwareMap.get(DcMotor.class, "Left Slide");
        rightDepositSlide = hardwareMap.get(DcMotor.class, "Right Slide");

        waitForStart();

        while(opModeIsActive())
        {

            //controller thing thing goes here :)
            //Slide intake motor controller set up
            leftDepositSlide.setPower(gamepad2.left_stick_y);
            rightDepositSlide.setPower(gamepad2.left_stick_y);
        }

    }


}
