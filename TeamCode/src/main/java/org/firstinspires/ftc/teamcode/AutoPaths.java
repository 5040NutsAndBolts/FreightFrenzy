package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AutoPaths
{

    private LinearOpMode parentOp;
    private Hardware robot;
    public boolean red;

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
