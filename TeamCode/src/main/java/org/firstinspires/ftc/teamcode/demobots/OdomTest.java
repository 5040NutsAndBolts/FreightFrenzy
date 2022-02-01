package org.firstinspires.ftc.teamcode.demobots;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;

@Disabled
@TeleOp(name = "OdomTest",group="teleop")
public class OdomTest extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        Hardware.currentOpMode=this;
        Hardware robot = new Hardware(hardwareMap);
        waitForStart();
        robot.softBrake();
        robot.resetOdometry(0,0,0);
        while(opModeIsActive())
        {
            robot.updatePositionRoadRunner();
            robot.drive(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);
            telemetry.addData("left",robot.leftEncoderPos);
            telemetry.addData("right",robot.rightEncoderPos);
            telemetry.addData("center",robot.centerEncoderPos);
            telemetry.update();
        }
    }
}
