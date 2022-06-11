package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


public class AutoMethods
{
    static boolean timeStarted = false;
    static ElapsedTime t = new ElapsedTime();

    public static void resetEncoders(Hardware robot)
    {
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public static void raiseIntake(Hardware robot)
    {
        robot.intakeArmUp();
        if (robot.intakeArm.getCurrentPosition() < 10 || timeStarted)
        {

            if(!timeStarted)
            {
                t.startTime();

            }
            timeStarted = true;
            robot.openIntake();
            //hold arm up so the stopper can catch
            if(t.seconds()<1.5)
            {
                robot.intakeArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.intakeArm.setPower(-1);
            }
            else {
                robot.intakeArm.setPower(1);
                robot.intake();
                robot.intakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
        else
            robot.intake();
    }
}
