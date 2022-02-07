package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
        robot.resetOdometry(0, 0, 3 * Math.PI / 2);
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
        robot.closeIntake();

        //moves the robot closer to hub in auto 3
        double towardsHub=auto==3?1.75:0;

        while (robot.x < 16.25+towardsHub) {
            robot.depositLevel = auto - 1;
            robot.deposit();
            robot.updatePositionRoadRunner();
            PathFollowers.linearTolerancePathFollow(robot, 0, 1.1, 3 * Math.PI / 2, .9, .05, 0.2, .15, 3 * Math.PI / 2, new Point(0, 0));

            if (robot.x > 13) {
                robot.intakeArmDown();
                robot.intake();
            }
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


        //deployed intake and start brushes

        if(auto==1)
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
        //Drive back to wall
        while(opModeIsActive()&robot.x>1.5)
        {

            telemetry.addData("x",robot.x);
            telemetry.addData("y",robot.y);
            telemetry.update();
            robot.updatePositionRoadRunner();
            PathFollowers.linearTolerancePathFollow(robot,-.75,-1,3*Math.PI/2,1.5,.04,0.2,.2,3*Math.PI/2,new Point(16,0));


        }

        robot.depositLevel=0;
        robot.intakeArmDown();
        robot.closeIntake();
        robot.setIntakePower(1);
        robot.rightRampUp();
        robot.depositNeutral();
        ElapsedTime timer = new ElapsedTime();
        timer.startTime();
        boolean reset=false;
        while(opModeIsActive()&&robot.y>-53&&(robot.intakeSeeperDraw()<3.5||timer.seconds()<.2))
        {
            telemetry.addData("x",robot.x);
            telemetry.addData("y",robot.y);
            telemetry.update();
            if(e.seconds()>.1&&!reset)
            {
                robot.resetOdometry(1, robot.y, robot.theta);
                reset=true;
            }
            robot.deposit();
            PathFollowers.linearTolerancePathFollow(robot,robot.y<-32?-.1:-.75,0,3*Math.PI/2,.1,.025,0.4,.15,3*Math.PI/2,new Point(0,0));
            robot.intake();
            robot.updatePositionRoadRunner();
        }
        while(opModeIsActive()&&robot.y<-32)
        {
            robot.deposit();
            PathFollowers.linearTolerancePathFollow(robot,1,0,3*Math.PI/2,.1,.025,0.4,.15,3*Math.PI/2,new Point(0,0));
            robot.intake();
            robot.updatePositionRoadRunner();
            telemetry.addData("x",robot.x);
            telemetry.addData("y",robot.y);
            telemetry.update();
        }

        //move forward to pick 0up object

        /*switch (openCVDetectionLevel) {

            case 1:
               //barcode closest to storage unit
                //puts freight on the bottom level
                while(robot.y > -3) {
                    if(robot.y<-1)
                        robot.depositNeutral();
                    robot.updatePositionRoadRunner();
                    robot.drive(-1, 0, 0);
                    telemetry.addData("y",robot.y);
                    telemetry.update();
                    robot.depositLevel = 1;
                }
                //lift intake up to get to deposit
                robot.intakeArmUp();

                while(robot.y < 0) {
                    robot.updatePositionRoadRunner();
                    robot.drive(1, .1, 0);
                    robot.deposit();
                    robot.setIntakePower(0);
                }
                robot.rightRampDown();
                robot.depositRight();


                /*
                ENTER TSC CODE GOES HERE
                */

                /*break;

            case 2:
                //barcode in the middle of storage unit and shared shipping hub
                // move backwards to shipping hub
                //puts freight on the second level
                while(robot.y > -6) {
                    if(robot.y>-1)
                        robot.depositNeutral();
                    robot.updatePositionRoadRunner();
                    robot.drive(-1, 0, 0);
                    robot.depositLevel = 2;
                }

                //lift intake up to get to deposit
                robot.intakeArmUp();

                while(robot.y < 0) {
                    robot.updatePositionRoadRunner();
                    robot.drive(1, .1, 0);
                    robot.deposit();
                    robot.setIntakePower(0);
                }
                robot.rightRampDown();
                robot.depositRight();

                /*
                ENTER TSC CODE GOES HERE
                */


               /* break;

            case 3:
                //barcode to thr right of the shipping hub
                // move backwards to shipping hub
                //puts freight on the top level
                while(robot.y > -14) {
                    if(robot.y>-1)
                        robot.depositNeutral();
                    robot.updatePositionRoadRunner();
                    robot.drive(-1, 0, 0);
                    robot.depositLevel = 3;
                }
                //lift intake up to get to deposit
                robot.intakeArmUp();

                while(robot.y < 0) {
                    robot.updatePositionRoadRunner();
                    robot.drive(1, .1, 0);
                    robot.deposit();
                    robot.setIntakePower(0);
                }
                robot.rightRampDown();
                robot.depositRight();


                /*
                ENTER TSC CODE GOES HERE
                */


                /*break;

        }

        while(robot.x > -1 && robot.y > -1) {
            robot.updatePositionRoadRunner();
            robot.drive(-0.5,1, 0);
        }
        robot.depositNeutral();
        robot.rightRampUp();
        robot.drive(0,0,0);

        while(robot.y > -50) {
            robot.updatePositionRoadRunner();
            robot.drive(-1, 0, 0);
        }
        robot.drive(0,0,0);
        */




    }




}
