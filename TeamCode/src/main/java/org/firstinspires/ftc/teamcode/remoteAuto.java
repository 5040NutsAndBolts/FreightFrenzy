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
    public void runOpMode() throws InterruptedException
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(new TSEFinder());
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            { webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT); }

            @Override
            public void onError(int errorCode) { }
        });

        int auto=1;


        Hardware.currentOpMode=this;
        Hardware robot = new Hardware(hardwareMap);
        robot.depositNeutral();
        robot.resetOdometry(0,0,3*Math.PI/2);
        while(!isStopRequested()&!isStarted())
        {

            if(TSEFinder.screenPosition.x < 100)
                auto = 1;
            else if(TSEFinder.screenPosition.x > 100 && TSEFinder.screenPosition.x < 200)
                auto = 2;
            else
                auto = 3;

        }
        waitForStart();
        //Open CV goes here to spit out 1, 2, or 3
        //moves robot to shipping hub
        robot.closeIntake();

        while(robot.x < 14.5) {
            robot.depositLevel=auto-1;
            robot.deposit();
            robot.updatePositionRoadRunner();
            PathFollowers.linearTolerancePathFollow(robot, 0, 1.1, 3*Math.PI/2, .5, .1, .3,.45, new Point(0, 0));
            if(robot.x>13)
            {
                robot.intakeArmDown();
                robot.intake();
            }
            if(robot.x>9.75)
                robot.rightRampDown();
            if(robot.x>9.75)
                robot.depositRight();
            Hardware.currentOpMode.telemetry.addData("x",robot.x);
            Hardware.currentOpMode.telemetry.addData("y",robot.y);
            Hardware.currentOpMode.telemetry.addData("theta",robot.theta);
            Hardware.currentOpMode.telemetry.update();

        }
        robot.drive(0,0,0);




        //deployed intake and start brushes


        robot.setIntakePower(.875);
        while(robot.y > -16) {
            if(robot.y<-1)
                robot.depositNeutral();
            robot.updatePositionRoadRunner();
            PathFollowers.curveToFacePoint(robot,8.2,-1,3*Math.PI/2,new Point(25.5,-34));
            //telemetry.addData("y",robot.y);
            //telemetry.update();
            robot.intake();
            robot.reallyCloseIntake();
            robot.depositLevel = 0;
            robot.deposit();
            if(robot.y<-11)
            {
                robot.intakeArmUp();
                robot.rightRampUp();

            }
            if(robot.y<-13.5)
                robot.setIntakePower(.78);
        }
        //Drive to deposit to score duck
        while(robot.y < -5.5)
        {
            robot.intake();

            if(robot.intakeArm.getCurrentPosition()<30)
                robot.openIntake();

            if (robot.y > -6.3)
            {
                robot.rightRampDown();
                robot.deposit();
                robot.setIntakePower(0);
            }

            robot.updatePositionRoadRunner();
            robot.drive(1, 0, HelperMethods.clamp(-1,-robot.y/29-.2,0));
        }
        robot.depositLevel=2;


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
        while(opModeIsActive()&robot.x>1)
        {

            telemetry.addData("x",robot.x);
            telemetry.addData("y",robot.y);
            telemetry.update();
            robot.updatePositionRoadRunner();
            PathFollowers.linearTolerancePathFollow(robot,-.8*.5,-1*.5,3*Math.PI/2,1.5,.05,0.2,.15,3*Math.PI/2,new Point(16,0));


        }
        robot.depositLevel=0;
        robot.intakeArmDown();
        robot.closeIntake();
        robot.setIntakePower(1);
        robot.rightRampUp();
        while(opModeIsActive()&&robot.y>-40)
        {
            telemetry.addData("x",robot.x);
            telemetry.addData("y",robot.y);
            telemetry.update();
            robot.deposit();
            PathFollowers.linearTolerancePathFollow(robot,-.4,0,3*Math.PI/2,.3,.05,0.2,.15,3*Math.PI/2,new Point(0,0));
            robot.intake();
            robot.updatePositionRoadRunner();
        }
        e.reset();
        while(e.seconds()<.9){robot.drive(0,0,0);}
        while(opModeIsActive()&&robot.y<-35)
        {
            robot.deposit();
            PathFollowers.linearTolerancePathFollow(robot,.4,0,3*Math.PI/2,.3,.1,0.2,.1,3*Math.PI/2,new Point(0,0));
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
