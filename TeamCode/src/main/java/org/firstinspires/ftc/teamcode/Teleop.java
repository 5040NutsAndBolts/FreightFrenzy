package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

@TeleOp(name="Teleop",group="Teleop")
public class Teleop extends LinearOpMode
{

    boolean x2Pressed = false;
    boolean a2Pressed = false;
    boolean a1Pressed = false;
    boolean rightRampUp = true;
    boolean leftRampUp = true;
    boolean intakeUp = true;


    @Override
    public void runOpMode() throws InterruptedException
    {

        Hardware robot = new Hardware(hardwareMap);

        waitForStart();
        //setting position for the ramp servo start
        robot.depositNeutral();
        robot.rightRampUp();
        robot.leftRampUp();

        Hardware.currentOpMode=this;
        robot.startIntakeThread();

        while (opModeIsActive())
        {

            //Slide outtake motor controller set up (linear slides)
            //robot.depositSlide.setPower(gamepad2.left_stick_y);
            if(gamepad2.right_bumper)
                robot.depositLevel++;

            if(gamepad2.left_bumper)
                robot.depositLevel--;

            //Move freight into correct deposit side
            if(gamepad2.dpad_left)
                robot.depositLeft();
            else if(gamepad2.dpad_right)
                robot.depositRight();
            else if(gamepad2.dpad_up)
                robot.depositNeutral();

            //Toggle left ramp when x is pressed
            if (!x2Pressed && gamepad2.x)
            {
                x2Pressed = true;
                leftRampUp = !leftRampUp;
            } else if (!gamepad2.x)
                x2Pressed = false;

            if(leftRampUp)
                robot.leftRampUp();
            else
                robot.leftRampDown();

            //Toggle right ramp when b is pressed
            if (!a2Pressed&&gamepad2.a) {

                a2Pressed = true;
                rightRampUp=!rightRampUp;

            }
            else if (!gamepad2.a)
                a2Pressed = false;

            if(rightRampUp)
                robot.rightRampUp();
            else
                robot.rightRampDown();

            //Set intake power
            if(gamepad1.right_trigger>0)
                robot.setIntakePower(-gamepad1.right_trigger);
            else
                robot.setIntakePower(gamepad1.left_trigger);

            //Set drivetrain power
            robot.drive(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);

            //Set duck spinner power
            robot.setLeftDuckSpinnerPower(gamepad2.left_trigger);
            robot.setRightDuckSpinnerPower(gamepad2.right_trigger);

            //Toggle Intake Arm Up & Down when gamepad1.a is pressed
            if (!a1Pressed && gamepad1.a)
            {
                a1Pressed = true;
                intakeUp = !intakeUp;
            } else if (!gamepad1.a)
                a1Pressed = false;

            if(intakeUp)
                robot.intakeArmUp();
            else
            {
                if(robot.intakeArmPosition()>30)
                    robot.closeIntake();
                robot.intakeArmDown();
            }

            if(robot.intakeArmPosition()<30)
                robot.openIntake();
            //color sensor code for the intake to
            if (robot.colorsensor.alpha() > 8) {

                robot.intakeArmUp();
                //stop and reset encoder to make it when it sees that the
                //servo would go up to position zero
            }


            telemetry.addData("intake arm position",robot.intakeArmPosition());
            telemetry.addData("deposit position", robot.depositPosition());
            telemetry.update();

        }
    }
}