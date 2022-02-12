package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Scanner;
import java.util.function.Function;
import java.util.regex.Pattern;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.helperclasses.HelperMethods;
import org.firstinspires.ftc.teamcode.helperclasses.LQR;
import org.firstinspires.ftc.teamcode.helperclasses.PathFollowers;
import org.firstinspires.ftc.teamcode.helperclasses.PID;
import org.firstinspires.ftc.teamcode.helperclasses.Point;
import org.firstinspires.ftc.teamcode.helperclasses.TSEFinder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.io.File;
import java.util.List;
import java.util.Scanner;


@Autonomous(name = "remoteAuto", group = "Auto")
public class remoteAuto extends LinearOpMode {

    int openCVDetectionLevel=1;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(new TSEFinder());
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        int auto = 1;


        Hardware.currentOpMode = this;
        Hardware robot = new Hardware(hardwareMap);

        robot.depositNeutral();
        robot.resetOdometry(0, robot.y, 3 * Math.PI / 2);
        while (!isStopRequested() & !isStarted()) {

            if (TSEFinder.screenPosition.x < 115)
                auto = 1;
            else if (TSEFinder.screenPosition.x < 215)
                auto = 2;
            else
                auto = 3;

            telemetry.addData("auto",auto);
            telemetry.update();
        }
        waitForStart();
        //Open CV goes here to spit out 1, 2, or 3
        //moves robot to shipping hub

        robot.intakeArmUp();
        robot.intakeArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //moves the robot closer to hub in auto 3
        double towardsHub=auto==3?1.75:0;

        boolean setMode=true;
        while (robot.x < 16.25+towardsHub) {
            if(robot.x<12)
            {
                robot.depositLevel = auto==3?2:1;
                if(robot.x>7)
                {
                    robot.closeIntake();
                    if(setMode)
                    {
                        robot.intakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        setMode=false;
                    }
                }
                else
                    robot.intakeArm.setPower(-1);
            }
            else
            {
                robot.depositLevel = auto - 1;
                robot.intake();
            }

            robot.deposit();
            robot.updatePositionRoadRunner();
            PathFollowers.linearTolerancePathFollow(robot, -.025, 1.1, 3 * Math.PI / 2, .9, .05, 0.2, .15, 3 * Math.PI / 2, new Point(0, 0));

            if (robot.x > 3)
                robot.rightRampDown();
            if (robot.x > 3.1-(auto==2?1:0))
                robot.depositRight();
            Hardware.currentOpMode.telemetry.addData("x", robot.x);
            Hardware.currentOpMode.telemetry.addData("y", robot.y);
            Hardware.currentOpMode.telemetry.addData("theta", robot.theta);
            Hardware.currentOpMode.telemetry.update();

        }
        robot.drive(0, 0, 0);

        while(opModeIsActive()&robot.x>16)
        {
            PathFollowers.linearTolerancePathFollow(robot,-.65,-.6,3*Math.PI/2,1.5,.04,0.2,.2,3*Math.PI/2,new Point(16,0));
            robot.updatePositionRoadRunner();
        }

        //Drive back to wall
        while(opModeIsActive()&robot.x>1.5)
        {

            telemetry.addData("x",robot.x);
            telemetry.addData("y",robot.y);
            telemetry.addData("theta", robot.theta);
            telemetry.update();
            robot.updatePositionRoadRunner();
            PathFollowers.linearTolerancePathFollow(robot,-.76,-1,3*Math.PI/2,1.5,.04,0.2,.2,3*Math.PI/2,new Point(16,0));


        }
        //deployed intake and start brushes
        robot.depositLevel=0;
        robot.intakeArmDown();
        robot.closeIntake();
        robot.setIntakePower(1);
        robot.rightRampUp();
        robot.depositNeutral();
        ElapsedTime timer = new ElapsedTime();
        timer.startTime();
        while(timer.seconds()<.1)
        {
            robot.drive(-.2,-.65,0);
            robot.updatePositionRoadRunner();
        }
        robot.resetOdometry(0, robot.y, 3*Math.PI/2);
        robot.updatePositionRoadRunner();
        timer.reset();
        boolean hitFreight=false;
        while(opModeIsActive()&&robot.y>-68&&(robot.y>-43||robot.intakeSeeperDraw()<4||timer.seconds()<.05))
        {
            telemetry.addData("x",robot.x);
            telemetry.addData("y",robot.y);
            telemetry.addData("theta", robot.theta);
            telemetry.addData("draw",robot.intakeSeeperDraw());
            telemetry.addData("time",timer.seconds());
            telemetry.update();
            if(robot.intakeSeeperDraw()<3.8&&!hitFreight)
            {
                timer.reset();
            }
            else if(robot.intakeSeeperDraw()>=3.8)
                hitFreight=true;
            robot.deposit();
            double speed=robot.y<-25?-.475:-.8;
            if(hitFreight)
                speed=.05;
            robot.drive(speed,-.1,0);
            robot.intake();
            robot.updatePositionRoadRunnerOnlyRight();
        }
        robot.setIntakePower(.1);
        while(opModeIsActive()&&robot.y<-30)
        {
            robot.deposit();
            robot.drive(1, -.25, 0);
            robot.intake();
            robot.intakeArmUp();
            if (robot.intakeArm.getCurrentPosition() < 30)
                robot.openIntake();
            robot.updatePositionRoadRunnerOnlyRight();
            telemetry.addData("x", robot.x);
            telemetry.addData("y", robot.y);
            telemetry.addData("theta", robot.theta);
            telemetry.update();
        }
        robot.resetDeltaTicks();
        robot.updatePositionRoadRunner();
        robot.resetOdometry(0,robot.y,3*Math.PI/2);
        robot.updatePositionRoadRunner();
        //Drive back to hub
        while(opModeIsActive()&robot.x<21)
        {

            robot.intake();
            if (robot.intakeArm.getCurrentPosition() < 30)
                robot.openIntake();
            telemetry.addData("x",robot.x);
            telemetry.addData("y",robot.y);
            telemetry.addData("theta", robot.theta);
            telemetry.update();
            if(robot.x>2.5)
            {
                robot.deposit();
                robot.depositLevel = 2;
            }
            if(robot.x<4)
                robot.updatePositionRoadRunnerRightAndCenter();
            else
                robot.updatePositionRoadRunner();
            robot.drive(.54,1,0);
            if(robot.x>12)
            {
                robot.depositRight();
                robot.rightRampDown();
            }

        }
        timer.reset();
        while(timer.seconds()<1)
            robot.drive(0,0,0);


         /* if(auto==1)
        {}
        else if(auto==2) {
            robot.setIntakePower(.68);
            PID pid = new PID(3 * Math.PI / 2, .75, .03, .15);
            while (robot.y > -16.5) {
                if (robot.y < -1)
                    robot.depositNeutral();
                robot.updatePositionRoadRunner();


                PathFollowers.curveToFacePoint(robot, pid, -1, 3 * Math.PI / 2, new Point(29, -25));

                telemetry.addData("y", robot.y);
                telemetry.update();
                robot.intake();
                robot.reallyCloseIntake();
                if (robot.y < -4)
                    robot.depositLevel = 0;
                robot.deposit();
                if (robot.y < -11) {
                    robot.intakeArmUp();
                    robot.rightRampUp();

                }
                if (robot.y < -13.5)
                    robot.setIntakePower(.65);
            }
            //Drive to deposit to score duck
            while (robot.y < -5.5) {
                robot.intake();

                if (robot.intakeArm.getCurrentPosition() < 30)
                    robot.openIntake();

                if (robot.y > -6.3) {
                    robot.rightRampDown();
                    robot.deposit();
                    robot.setIntakePower(0);
                }

                robot.updatePositionRoadRunner();
                robot.drive(1, -.05, HelperMethods.clamp(-1, -robot.y / 55 - .41, 0));
            }
        }
        else
        {
            ElapsedTime timer = new ElapsedTime();
            timer.startTime();
            while(timer.seconds()<.35)
            {

                if(timer.seconds()>.15)
                {
                    robot.drive(0, -.45, 0);
                    robot.rightRampUp();
                }
                else
                    robot.drive(0,0,0);

            }
            robot.setIntakePower(.68);
            PID pid = new PID(3 * Math.PI / 2, .72, .03, .16);
            while (robot.y > -4.25) {
                if (robot.y < -1)
                    robot.depositNeutral();
                robot.updatePositionRoadRunner();

                PathFollowers.curveToFacePoint(robot, pid, -.4, 3 * Math.PI / 2, new Point(29, -13));

                telemetry.addData("y", robot.y);
                telemetry.update();
                robot.intake();
                robot.reallyCloseIntake();
                if (robot.y < -1)
                    robot.depositLevel = 0;
                robot.deposit();
                if (robot.y < -2.25) {
                    robot.intakeArmUp();

                    if (robot.intakeArm.getCurrentPosition() < 30)
                        robot.openIntake();

                }
                if (robot.y < -2)
                    robot.setIntakePower(.7);
            }
            timer.reset();
            while(timer.seconds()<.5)
            {
                robot.intakeArmUp();
                robot.intake();
                if (robot.intakeArm.getCurrentPosition() < 30)
                    robot.openIntake();
            }
            //Drive to deposit to score duck
            while (robot.y < -1) {
                robot.intake();
                if (robot.intakeArm.getCurrentPosition() < 30)
                    robot.openIntake();

                if (robot.y > -1.5) {
                    robot.rightRampDown();
                    robot.deposit();
                    robot.setIntakePower(0);
                }

                robot.updatePositionRoadRunner();
                robot.drive(1, 0, HelperMethods.clamp(-1, -robot.y / 40 - .9, 0));
            }

        }
        robot.depositLevel = 2;


        ElapsedTime e = new ElapsedTime();
        e.startTime();
        while(e.seconds()<.7)
        {
            robot.deposit();
            robot.drive(0, 0, 0);
            if(e.seconds()>.4)
                robot.depositRight();
        }

        */

    }

}
