package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helperclasses.HelperMethods;

@TeleOp(name="Teleop",group="Teleop")
public class Teleop extends LinearOpMode
{

    boolean x2Pressed = false;
    double horizontalPos=.5;
    double verticalPos=.5;
    boolean a2Pressed = false;
    boolean a1Pressed = false;
    boolean rightRampUp = true;
    boolean leftRampUp = true;
    boolean intakeUp = true;
    boolean bumperPressed=false;
    boolean b1Pressed=false;
    boolean slowMode=false;
    boolean b2pressed=false;
    boolean leftDepositeRamp=true;
    boolean linkedDeposit = true;

    ElapsedTime e;

    @Override
    public void runOpMode() throws InterruptedException
    {


        e=new ElapsedTime();
        e.startTime();
        double lastTime=0;
        Hardware.currentOpMode=this;
        Hardware robot = new Hardware(hardwareMap);

        telemetry.addLine("init done");
        telemetry.update();
        waitForStart();
        //setting position for the ramp servo start
        robot.depositNeutral();
        robot.rightRampUp();
        robot.leftRampUp();

        Hardware.currentOpMode=this;
        //robot.startIntakeThread();
        //robot.activateDeposit();

        while (opModeIsActive())
        {

            robot.updatePositionRoadRunner();

            robot.intake();
            robot.deposit();

            //Slide outtake motor controller set up (linear slides)
            if(gamepad2.right_bumper&&!bumperPressed&&robot.depositLevel<2)
            {
                robot.depositLevel++;
                bumperPressed=true;
            }
            else if(gamepad2.left_bumper&&!bumperPressed&&robot.depositLevel>0)
            {
                robot.depositLevel--;
                bumperPressed=true;
            }
            else if(!gamepad2.left_bumper&&!gamepad2.right_bumper)
                bumperPressed=false;

            //override to control deposit with stick
            if(gamepad2.y)
            {
                robot.depositOverride=true;
                if(!robot.depositSlide.getMode().equals(DcMotor.RunMode.RUN_WITHOUT_ENCODER))
                    robot.depositSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.depositSlide.setPower(Math.abs(gamepad2.right_stick_y-.05)>.05?-gamepad2.right_stick_y:.12);
            }
            else
            {
                if(!robot.depositSlide.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION))
                    robot.depositSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.depositOverride = false;

                //capper
                robot.setOutPower(gamepad2.right_stick_y);
            }

            //controls deposit toggle
            if(gamepad2.b && !b2pressed && !gamepad2.start)
            {
                b2pressed = true;
                linkedDeposit = !linkedDeposit;
            }
            else
            {
                b2pressed = false;
            }

            //capper
            robot.setHorizontalPosition(horizontalPos);
            robot.setVerticalPosition(verticalPos);
            horizontalPos=HelperMethods.clamp(0,horizontalPos+gamepad2.left_stick_x*(e.seconds()-lastTime),1);
            verticalPos= HelperMethods.clamp(0,verticalPos -gamepad2.left_stick_y*(e.seconds()-lastTime),1);

            //intake override
            if(gamepad1.y)
            {
                robot.intakeArmUp();
                robot.intakeOverride=true;
                if( robot.intakeArm.getMode()!=DcMotor.RunMode.RUN_WITHOUT_ENCODER)
                    robot.intakeArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.intakeArm.setPower(-.5);
            }
            else
            {
                if(robot.intakeArm.getMode()!=DcMotor.RunMode.RUN_TO_POSITION)
                    robot.intakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.intakeOverride = false;
            }

            //Move freight into correct deposit side when toggle is off
            if(!linkedDeposit) {
                if (gamepad2.dpad_left) {

                    leftDepositeRamp = true;
                    robot.depositLeft();
                } else if (gamepad2.dpad_right) {

                    leftDepositeRamp = false;
                    robot.depositRight();
                } else if (gamepad2.dpad_up)
                    robot.depositNeutral();
            }

            //linked deposit ramp controls
            if(linkedDeposit)
            {
                if (gamepad2.dpad_left) {
                    leftDepositeRamp = true;
                    robot.depositLeft();
                    leftRampUp = !leftRampUp;
                }
                else if (gamepad2.dpad_right) {
                    leftDepositeRamp = false;
                    robot.depositRight();
                    rightRampUp = !rightRampUp;
                }
                else if (gamepad2.dpad_up)
                    robot.depositNeutral();
                    if(!leftRampUp)
                        leftRampUp = true;
                    else  if(!rightRampUp)
                        rightRampUp = true;

                if (leftRampUp)
                    robot.leftRampUp();
                else
                    robot.leftRampDown();

                if (leftRampUp)
                    robot.leftRampUp();
                else
                    robot.leftRampDown();
            }


            //unlinked ramp controls
            if(!linkedDeposit) {
                //Toggle left ramp when x is pressed
                if (!x2Pressed && gamepad2.x) {
                    x2Pressed = true;
                    leftRampUp = !leftRampUp;
                } else if (!gamepad2.x)
                    x2Pressed = false;

                if (leftRampUp)
                    robot.leftRampUp();
                else
                    robot.leftRampDown();

                //Toggle right ramp when a is pressed
                if (!a2Pressed && gamepad2.a) {

                    a2Pressed = true;
                    rightRampUp = !rightRampUp;

                } else if (!gamepad2.a)
                    a2Pressed = false;

                if (rightRampUp)
                    robot.rightRampUp();
                else
                    robot.rightRampDown();
            }

            //Set intake power
            if(gamepad1.right_trigger>0)
                robot.setIntakePower(gamepad1.right_trigger);
            else
                robot.setIntakePower(-gamepad1.left_trigger * .5);

            if(gamepad1.b&&!b1Pressed)
            {
                b1Pressed=true;
                slowMode=!slowMode;
            }
            else if(!gamepad1.b)
            {
                b1Pressed=false;
            }

            double driveSpeed=1;
            if(slowMode)
                driveSpeed=.4;



            //Set drivetrain power
            robot.drive(gamepad1.left_stick_y*driveSpeed,gamepad1.left_stick_x*driveSpeed,gamepad1.right_stick_x*driveSpeed);

            //Set duck spinner power
            if(gamepad1.dpad_left || gamepad1.dpad_up || gamepad1.dpad_right || gamepad1.dpad_down)
            {
                robot.setLeftDuckSpinnerPower(-1);
                robot.setRightDuckSpinnerPower(-1);
            }
            else{
                robot.setRightDuckSpinnerPower(0);
                robot.setLeftDuckSpinnerPower(0);
            }

            //Toggle Intake Arm Up & Down when gamepad1.a is pressed
            if (!a1Pressed && gamepad1.a&&!gamepad1.start)
            {
                a1Pressed = true;
                intakeUp = !intakeUp;
            } else if (!gamepad1.a)
                a1Pressed = false;

            if(intakeUp)
            {
                robot.intakeArmUp();
                if(robot.intakeArmPosition()<30)
                    robot.openIntake();
            }
            else
            {
                robot.closeIntake();
               robot.intakeArmDown();
            }





            PIDFCoefficients pid = robot.depositSlide.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("Intake arm position",robot.intakeArmPosition());
            telemetry.addData("pid",pid.p+" "+ pid.i+" "+ pid.d);
            telemetry.addData("Deposit position", robot.depositPosition());
            telemetry.addData("color",robot.colorsensor.red()+" "+robot.colorsensor.green()+" "+robot.colorsensor.blue()+" "+robot.colorsensor.alpha());
            telemetry.addData("Slow-mode", slowMode);
            telemetry.addData("override",robot.depositOverride);
            telemetry.addData("deposit level", robot.depositLevel);
            telemetry.addData("x",robot.x);
            telemetry.addData("y",robot.y);
            telemetry.addData("horiz",horizontalPos);
            telemetry.addData("vert",verticalPos);
            telemetry.addData("theta",robot.theta);
            telemetry.addData("time",e.seconds()-lastTime);
            lastTime=e.seconds();
            telemetry.update();

        }
    }
}
