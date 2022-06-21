package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.helperclasses.HelperMethods;

@TeleOp(name="Teleop",group="Teleop")
public class Teleop extends LinearOpMode
{
    boolean leftStick1 = false;
    boolean x2Pressed = false;
    double horizontalPos=.44;
    double verticalPos=.61;
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
    boolean tseMode = false;
    boolean invertedStrafe = false;
    ElapsedTime intakeTimer;

    ElapsedTime e;

    @Override
    public void runOpMode() throws InterruptedException
    {
        e=new ElapsedTime();
        e.startTime();
        double lastTime=0;
        Hardware.currentOpMode=this;
        Hardware robot = new Hardware(hardwareMap);
        intakeTimer=new ElapsedTime();
        intakeTimer.startTime();
        telemetry.addLine("init done");
        telemetry.update();
        waitForStart();
        //setting position for the ramp servo start
        robot.depositNeutral();
        robot.rightRampUp();
        robot.leftRampUp();
        robot.openIntake();
        robot.intakeStart();

        Hardware.currentOpMode=this;
        //robot.startIntakeThread();
        //robot.activateDeposit();

        while (opModeIsActive())
        {
            //I don't know what this does but I need right trigger for other stuff -Eleanor
            /*if(gamepad2.right_trigger>.4)
            {
                robot.depositSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.depositSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            robot.updatePositionRoadRunner();*/

            robot.deposit();

            //Controller 1 TSE modes with left bumper being driving mode and right bumper to TSE mode
            if (gamepad1.left_bumper)
            {
                tseMode = false;
            }else if(gamepad1.right_bumper)
            {
                tseMode = true;
            }

            //switches inverted drive on and off
            if(gamepad1.left_stick_button && !leftStick1)
            {
                invertedStrafe = !invertedStrafe;
                leftStick1 = true;
            }
            else
            {
                leftStick1 = false;
            }

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
                robot.depositSlide.setPower(Math.abs(gamepad2.left_stick_y-.05)>.05?-gamepad2.left_stick_y:.12);
            }
            else
            {
                if(!robot.depositSlide.getMode().equals(DcMotor.RunMode.RUN_TO_POSITION))
                    robot.depositSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.depositOverride = false;

            }

            //capper
            robot.setHorizontalPosition(1 - horizontalPos);
            robot.setVerticalPosition(verticalPos);

            //intake override
            if(gamepad1.y)
            {
                robot.intakeArmUp();
                robot.intakeOverride=true;

                if( robot.intakeArm.getMode()!=DcMotor.RunMode.RUN_WITHOUT_ENCODER)
                    robot.intakeArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.intakeArm.setPower(-.75);
                if(robot.colorsensor.getDistance(DistanceUnit.INCH)<1.5)
                {
                    robot.intakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.openIntake();
                }
                else
                    robot.closeIntake();
            }
            else
            {
                if(robot.intakeArm.getMode()!=DcMotor.RunMode.RUN_TO_POSITION)
                    robot.intakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.intakeOverride = false;
                robot.intake();
            }

            //controls deposit toggle
            if(gamepad2.b && !b2pressed && !gamepad2.start)
            {
                b2pressed = true;
                linkedDeposit = !linkedDeposit;
            }
            else if(!gamepad2.b)
            {
                b2pressed = false;
            }

            //Move freight into correct deposit side when toggle is off
            if(!linkedDeposit) {
                if (gamepad2.dpad_left) {

                    robot.depositLeft();
                } else if (gamepad2.dpad_right) {

                    robot.depositRight();
                } else if (gamepad2.dpad_up)
                    robot.depositNeutral();
            }

            //linked deposit ramp controls
            if(linkedDeposit)
            {
                if (gamepad2.dpad_left)
                {
                    robot.depositLeft();
                    leftRampUp = false;
                    rightRampUp=true;
                }
                else if (gamepad2.dpad_right)
                {
                    leftRampUp = true;
                    robot.depositRight();
                    rightRampUp = false;
                }
                else if (gamepad2.dpad_up)
                {
                    robot.depositNeutral();
                    leftRampUp = true;
                    rightRampUp = true;
                }
            }

            //raise everything regardless of linked controls
            if (gamepad2.dpad_down)
            {
                robot.depositNeutral();
                leftRampUp = true;
                rightRampUp = true;
            }

            //unlinked ramp controls
            if(!linkedDeposit) {
                //Toggle left ramp when x is pressed
                if (!x2Pressed && gamepad2.x) {
                    x2Pressed = true;
                    leftRampUp = !leftRampUp;
                } else if (!gamepad2.x)
                    x2Pressed = false;


                //Toggle right ramp when a is pressed
                if (!a2Pressed && gamepad2.a) {

                    a2Pressed = true;
                    rightRampUp = !rightRampUp;

                } else if (!gamepad2.a)
                    a2Pressed = false;
            }


            if (leftRampUp)
                robot.leftRampUp();
            else
                robot.leftRampDown();

            if (rightRampUp)
                robot.rightRampUp();
            else
                robot.rightRampDown();

            //Set intake power
            if(gamepad1.right_trigger>0)
                robot.setIntakePower(gamepad1.right_trigger);
            else
                robot.setIntakePower(-gamepad1.left_trigger * .57);

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


            if(!tseMode)
            {
                //set drive power
                if(!invertedStrafe)
                {
                    robot.drive(gamepad1.left_stick_y * driveSpeed, gamepad1.left_stick_x * driveSpeed, gamepad1.right_stick_x * driveSpeed);
                    robot.setOutPower(gamepad2.right_stick_y);
                }
                else
                {
                    robot.drive(gamepad1.left_stick_y * driveSpeed, -gamepad1.left_stick_x * driveSpeed, gamepad1.right_stick_x * driveSpeed);
                    robot.setOutPower(gamepad2.right_stick_y);
                }
            }
            else
            {
                //capper
                if(Math.abs(gamepad2.right_stick_y)>.1)
                    robot.setOutPower(gamepad2.right_stick_y);
                else
                    robot.setOutPower(gamepad1.right_stick_y>0||gamepad1.right_stick_button?gamepad1.right_stick_y:gamepad1.right_stick_y*.15);

                horizontalPos=HelperMethods.clamp(0,horizontalPos+gamepad1.left_stick_x*(e.seconds()-lastTime)*.48,1);
                verticalPos= HelperMethods.clamp(0,verticalPos+gamepad1.left_stick_y*(e.seconds()-lastTime)*.48,1);
            }

            //robot.setRightDuckSpinnerPower(gamepad2.left_stick_button?-.8*gamepad2.left_trigger:-gamepad2.left_trigger);
            //Set duck spinner power
            /*if(gamepad2.left_trigger>.25)
            {
                robot.setLeftDuckSpinnerPower(-1);

            }
            else{
                robot.setLeftDuckSpinnerPower(0);
            }*/

            if(gamepad2.right_trigger == 0)
            {
                if (!gamepad2.left_stick_button)
                    robot.setRightDuckSpinnerPower(-gamepad2.left_trigger);
                else
                    robot.setRightDuckSpinnerPower(gamepad2.left_trigger * -.8);
            }

            if(gamepad2.left_trigger == 0)
            {
                if (!gamepad2.right_stick_button) {
                    robot.setLeftDuckSpinnerPower(gamepad2.right_trigger);
                    robot.setRightDuckSpinnerPower(gamepad2.right_trigger);
                } else {
                    robot.setRightDuckSpinnerPower(gamepad2.right_trigger * .8);
                    robot.setLeftDuckSpinnerPower(gamepad2.right_trigger);
                }
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
                if(robot.colorsensor.getDistance(DistanceUnit.INCH)<1.5)
                    robot.openIntake();

            }
            else
            {
                robot.closeIntake();
                robot.intakeArmDown();
            }


            int distanceMoved = (robot.frontRight.getCurrentPosition() + robot.frontLeft.getCurrentPosition() + robot.backLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) / 4;
            PIDFCoefficients pid = robot.depositSlide.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("Linked Deposit",linkedDeposit);
            telemetry.addData("Deposit Override",robot.depositOverride);
            telemetry.addData("Deposit Level", robot.depositLevel);
            telemetry.addLine();
            telemetry.addData("Inverted Strafe", invertedStrafe);
            telemetry.addData("Slow-Mode", slowMode);
            telemetry.addData("TSE Mode", tseMode);
            telemetry.addLine();
            telemetry.addData("hozizontal", horizontalPos);
            telemetry.addData("vertical", verticalPos);
            /*telemetry.addData("Distance Moved", distanceMoved);
            telemetry.addData("front left", robot.frontLeft.getCurrentPosition());
            telemetry.addData("front right", robot.frontRight.getCurrentPosition());
            telemetry.addData("back left", robot.backLeft.getCurrentPosition());
            telemetry.addData("back right", robot.backRight.getCurrentPosition());
            telemetry.addData("red", robot.intakeColorSensor.red());*/

            //telemetry.addData("intake position", robot.intakeArm.getCurrentPosition());

            //telemetry.addData("Intake Pos",robot.intakeArm.getCurrentPosition());
            //telemetry.addData("horiz",horizontalPos);
            //telemetry.addData("vert",verticalPos);
            //telemetry.addData("voltage", robot.intakeSeeperDraw());
            //telemetry.addData("PID", robot.intakeArm.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
            lastTime=e.seconds();
            telemetry.update();

        }
    }
}
