package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AutoPaths
{

    private LinearOpMode parentOp;
    private Hardware robot;
    public boolean red;
    double forwardStrafePosTicks=0;
    double forwardTicks=0;
    double intakeForwardsTicksAdd=0;
    double intakeRotateMultiply=1;
    double intakeTimeAdd=0;

    public AutoPaths(LinearOpMode opMode,Hardware hardware, boolean isRed)
    {

        parentOp=opMode;
        robot=hardware;
        red=isRed;

    }

    public AutoFunction spinDuck = ()->
    {
        ElapsedTime e = new ElapsedTime();
        e.startTime();
        while(parentOp.opModeIsActive()&&e.seconds()<3)
        {
            double strafe;
            if(red)
                strafe=-.4;
            else
                strafe =.4;
            if(e.seconds()<.25)
                robot.drive(-.3,0,0);
            else
                robot.drive(0,0,0);
            if(red)
                robot.setLeftDuckSpinnerPower(-1);
            else
                robot.setRightDuckSpinnerPower(1);

        }

    };

    public AutoFunction intake = ()->
    {

        robot.intakeArmDown();
        robot.closeIntake();
        ElapsedTime intakeDown = new ElapsedTime();
        intakeDown.startTime();
        while(parentOp.opModeIsActive()&&intakeDown.seconds()<.5)
        {
            if(intakeDown.seconds()>.1)
                robot.setIntakePower(-1);
            robot.intake();
        }
        ElapsedTime wiggle = new ElapsedTime();
        wiggle.startTime();
        while(parentOp.opModeIsActive()&&robot.forwardsTicks()-forwardTicks>-845+intakeForwardsTicksAdd)
        {

            robot.intake();
            parentOp.telemetry.addData("forward",robot.forwardsTicks()-forwardTicks);
            parentOp.telemetry.update();

            if(robot.forwardsTicks()<-700-forwardTicks)
            {
                robot.drive(-.9, .1, 0);
            }
            else
            {
                robot.drive(-1, 0, 0);
            }

            robot.setIntakePower(-1);
        }
        robot.drive(0,0,0);
        ElapsedTime runIntake=new ElapsedTime();
        runIntake.startTime();
        while(parentOp.opModeIsActive()&&runIntake.seconds()<.35+intakeTimeAdd)
        {
            robot.setIntakePower(-1);
            if(runIntake.seconds()<.19)
                robot.drive(-.2, .2, .1*intakeRotateMultiply);
            else if(runIntake.seconds()<.29+intakeTimeAdd/2)
                robot.drive(-.35, 0, -.18*intakeRotateMultiply);
            else robot.drive(0,0,0);
        }
        robot.setIntakePower(0);

    };

    public AutoFunction deposit = ()->
    {

        //leave warehouse
        while(parentOp.opModeIsActive()&&robot.forwardsTicks()-forwardTicks<300)
        {

            parentOp.telemetry.addData("forward",robot.forwardsTicks()-forwardTicks);
            parentOp.telemetry.update();
            robot.intake();
            if(robot.forwardsTicks()-forwardTicks<=-670)
                robot.drive(.55-(670+robot.forwardsTicks()-forwardTicks)/800.0,-.8,0);
            else if(robot.forwardsTicks()<-300&&robot.forwardsTicks()-forwardTicks>-740)
                robot.drive(1,-.1,0);
            else
                robot.drive(1,0,0);
            robot.intakeArmUp();
            robot.closeIntake();
            robot.setIntakePower(-1);
        }

        //drive to shipping hub
        while(parentOp.opModeIsActive()&&robot.forwardsTicksStrafePos()-forwardStrafePosTicks<790)
        {

            parentOp.telemetry.addData("forward",robot.forwardsTicksStrafePos()-forwardStrafePosTicks);
            parentOp.telemetry.update();
            robot.intake();

            robot.drive(.43,1,0);

            robot.intakeArmUp();
            if(robot.forwardsTicksStrafePos()-forwardStrafePosTicks>200)
                robot.openIntake();
            robot.setIntakePower(-1);
        }
        ElapsedTime brakeTime = new ElapsedTime();
        brakeTime.startTime();
        while(brakeTime.seconds()<.1)
            robot.drive(0,0,0);


    };

    public AutoFunction returnFromDeposit=()->
    {

        while(parentOp.opModeIsActive()&&robot.forwardsTicksStrafePos()-forwardStrafePosTicks>-330)
        {
            robot.drive(-1,-1,0);
        }
        while(parentOp.opModeIsActive()&&robot.forwardsTicksStrafePos()-forwardStrafePosTicks>-500)
        {
            robot.drive(-1,0,0);
        }
        ElapsedTime brakeTime = new ElapsedTime();
        brakeTime.startTime();
        if(robot.forwardsTicksStrafePos()-forwardStrafePosTicks<0)
        {
            robot.intakeArmDown();
            robot.intake();
            robot.setIntakePower(-1);
        }
        forwardStrafePosTicks=robot.forwardsTicksStrafePos();
        forwardTicks=robot.forwardsTicks();
        intakeForwardsTicksAdd-=75;
        intakeRotateMultiply+=.2;
        intakeTimeAdd+=.15;

    };

    public AutoFunction park = ()->
    {

        ElapsedTime e = new ElapsedTime();
        while(parentOp.opModeIsActive()&&e.seconds()<.25)
        {
            if (red)
                robot.drive(0, 1, 0);
            else
                robot.drive(0, -1, 0);
        }
        while(parentOp.opModeIsActive()&&e.seconds()<.8)
        {
            if (red)
                robot.drive(-1, 1, 0);
            else
                robot.drive(-1, -1, 0);
        }

    };

    public AutoFunction parkWarehouse = ()->
    {

        ElapsedTime e = new ElapsedTime();
        while(parentOp.opModeIsActive()&&e.seconds()<.9)
        {
            if (red)
                robot.drive(-1, 0, 0);
            else
                robot.drive(-1, 0, 0);
        }
        while(parentOp.opModeIsActive()&&e.seconds()<1.2)
        {
            if (red)
                robot.drive(0, -1, 0);
            else
                robot.drive(0, 1, 0);
        }
        robot.drive(0,0,0);

    };

}
