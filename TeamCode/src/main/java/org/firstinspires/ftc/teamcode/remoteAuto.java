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

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.helperclasses.HelperMethods;
import org.firstinspires.ftc.teamcode.helperclasses.LQR;
import org.firstinspires.ftc.teamcode.helperclasses.PathFollowers;
import org.firstinspires.ftc.teamcode.helperclasses.Point;

import java.io.File;
import java.util.List;
import java.util.Scanner;


@Autonomous(name = "remoteAuto", group = "Auto")
public class remoteAuto extends LinearOpMode {

    int openCVDetectionLevel=1;

    @Override
    public void runOpMode() throws InterruptedException
    {
        Hardware.currentOpMode=this;
        Hardware robot = new Hardware(hardwareMap);
        robot.depositNeutral();
        robot.resetOdometry(0,0,3*Math.PI/2);
        waitForStart();
        //Open CV goes here to spit out 1, 2, or 3
        //moves robot to shipping hub
        robot.rightRampDown();
        robot.closeIntake();

        while(robot.x < 17.5) {
            robot.updatePositionRoadRunner();
            PathFollowers.linearTolerancePathFollow(robot, 0, 1, 3*Math.PI/2, .5, .1, .3,.45, new Point(0, 0));
            if(robot.x>13)
            {
                robot.intakeArmDown();
                robot.intake();
            }
            if(robot.x>9)
                robot.depositRight();


        }
        robot.drive(0,0,0);




        //deployed intake and start brushes


        robot.setIntakePower(1);
        while(robot.y > -13.75) {
            if(robot.y<-1)
                robot.depositNeutral();
            robot.updatePositionRoadRunner();
            PathFollowers.curveToFacePoint(robot,.6,-1,3*Math.PI/2,new Point(30,-32));
            //telemetry.addData("y",robot.y);
            //telemetry.update();
            robot.intake();
            robot.depositLevel = 0;
            if(robot.y<-10.5)
            {
                robot.intakeArmUp();
                robot.rightRampUp();
            }
        }
        while(robot.y < -4.25)
        {
            robot.intake();

            if(robot.intakeArm.getCurrentPosition()<30)
                robot.intakeHalfWay();

            if (robot.y > -8)
            {
                robot.rightRampDown();

                robot.depositLevel=2;
                robot.deposit();
                robot.setIntakePower(0);
            }
            if(robot.y>-6)
                robot.depositRight();
            robot.updatePositionRoadRunner();
            robot.drive(1, 0, HelperMethods.clamp(-1,robot.y/11.5+.61,0));
        }
        ElapsedTime e = new ElapsedTime();
        e.startTime();
        while(e.seconds()<2)
        {
            robot.drive(0, 0, 0);
            robot.deposit();
        }

        //move forward to pick up object

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
