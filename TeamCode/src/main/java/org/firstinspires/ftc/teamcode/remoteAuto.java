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
        ElapsedTime totalAutoTime = new ElapsedTime();
        totalAutoTime.startTime();
        //Open CV goes here to spit out 1, 2, or 3
        //moves robot to shipping hub

        robot.intakeArmUp();
        robot.intakeArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //moves the robot closer to hub in auto 3
        double towardsHub=auto==3?3:0;

        boolean setMode=true;
        while (robot.x < 17.5+towardsHub) {
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
            PathFollowers.linearTolerancePathFollow(robot, -.025, 1.6-robot.x/16, 3 * Math.PI / 2, .9, .05, 0.2, .15, 3 * Math.PI / 2, new Point(0, 0));

            if (robot.x > 3)
                robot.rightRampDown();
            if (robot.x > 5.5-(auto==2?1:0))
                robot.depositRight();
            Hardware.currentOpMode.telemetry.addData("x", robot.x);
            Hardware.currentOpMode.telemetry.addData("y", robot.y);
            Hardware.currentOpMode.telemetry.addData("theta", robot.theta);
            Hardware.currentOpMode.telemetry.update();

        }
        robot.drive(0, 0, 0);
        while (opModeIsActive() & robot.x > 15.5) {
            PathFollowers.linearTolerancePathFollow(robot, -.4, -.35, 3 * Math.PI / 2, 1.5, .04, 0.2, .2, 3 * Math.PI / 2, new Point(16, 0));
            robot.updatePositionRoadRunner();
        }

        //Drive back to wall
        while (opModeIsActive() & robot.x > 1.5) {

            telemetry.addData("x", robot.x);
            telemetry.addData("y", robot.y);
            telemetry.addData("theta", robot.theta);
            telemetry.update();
            robot.updatePositionRoadRunner();
            robot.drive(-.7, -1,-.1);


        }
        //deployed intake and start brushes
        robot.depositLevel = 0;
        robot.intakeArmDown();
        robot.closeIntake();
        robot.setIntakePower(1);
        robot.rightRampUp();
        robot.depositNeutral();
        ElapsedTime timer = new ElapsedTime();
        timer.startTime();
        while (timer.seconds() < .4) {
            robot.drive(-.17, -.4, 0);
            robot.updatePositionRoadRunner();
        }
        robot.resetOdometry(0, robot.y, 3 * Math.PI / 2);
        robot.updatePositionRoadRunner();
        timer.reset();
        ElapsedTime t=new ElapsedTime();
        boolean timeStarted=false;
        boolean hitFreight = false;
        robot.closeIntake();
        while(opModeIsActive()&&robot.y>-68&&(robot.y>-43||timer.seconds()<.05)&&robot.intakeSeeperDraw()<4.3) {
            telemetry.addData("x", robot.x);
            telemetry.addData("y", robot.y);
            telemetry.addData("theta", robot.theta);
            telemetry.addData("draw", robot.intakeSeeperDraw());
            telemetry.addData("time", timer.seconds());
            telemetry.update();
            if (robot.intakeSeeperDraw() < 3.75 && !hitFreight) {
                timer.reset();
            } else if (robot.intakeSeeperDraw() >= 3.75)
                hitFreight = true;
            robot.deposit();
            double speed = robot.y < -25 ? -.4 : -.8;
            if (hitFreight)
                speed = .025;
            robot.drive(speed, -.1, 0);
            robot.intake();
            robot.updatePositionRoadRunnerOnlyRight();
        }

        robot.setIntakePower(.8);
        while (opModeIsActive() && robot.y < -25) {
            if(robot.intakeSeeperDraw()>7)
                robot.setIntakePower(-.3);
            robot.deposit();
            robot.drive(1, -.275, 0);

            robot.intakeArmUp();
            if (robot.intakeArm.getCurrentPosition() < 25||timeStarted) {

                if(!timeStarted)
                {
                    t.startTime();

                }
                timeStarted=true;
                robot.openIntake();
                if(t.seconds()<1.5) {
                    robot.intakeArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.intakeArm.setPower(-1);
                }
                    else {
                    robot.intakeArm.setPower(-1);
                    robot.intake();
                    robot.intakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            }
            else
                robot.intake();
            robot.updatePositionRoadRunnerOnlyRight();
            telemetry.addData("x", robot.x);
            telemetry.addData("y", robot.y);
            telemetry.addData("theta", robot.theta);
            telemetry.update();
        }
        robot.resetDeltaTicks();
        robot.updatePositionRoadRunner();
        robot.resetOdometry(0, robot.y, 3 * Math.PI / 2);
        robot.updatePositionRoadRunner();
        //Drive back to hub
        while (opModeIsActive() & robot.x < 21.5) {

            robot.intake();
            if (robot.intakeArm.getCurrentPosition() < 30)
                robot.openIntake();
            telemetry.addData("x", robot.x);
            telemetry.addData("y", robot.y);
            telemetry.addData("theta", robot.theta);
            telemetry.update();
            if (robot.x > 2.5) {
                robot.deposit();
                robot.depositLevel = 2;
            }
            if (robot.x < 4)
                robot.updatePositionRoadRunnerRightAndCenter();
            else
                robot.updatePositionRoadRunner();
            robot.drive(.48, 1, 0);
            if (robot.x > 16) {
                robot.depositRight();
                robot.rightRampDown();
            }

        }
        timer.reset();
        while(timer.seconds()<.3)
            robot.drive(0,0,0);
        for(int i = 0; i<2; i++) {
            while (opModeIsActive() & robot.x > 15.5) {
                PathFollowers.linearTolerancePathFollow(robot, -.4, -.35, 3 * Math.PI / 2, 1.5, .04, 0.2, .2, 3 * Math.PI / 2, new Point(16, 0));
                robot.updatePositionRoadRunner();
            }

            //Drive back to wall
            while (opModeIsActive() & robot.x > 7.1) {

                telemetry.addData("x", robot.x);
                telemetry.addData("y", robot.y);
                telemetry.addData("theta", robot.theta);
                telemetry.update();
                robot.updatePositionRoadRunner();
                PathFollowers.linearTolerancePathFollow(robot, -.76, -1, 3 * Math.PI / 2, 1.5, .04, 0.2, 0, 3 * Math.PI / 2, new Point(20, -5));


            }
            //deployed intake and start brushes
            robot.depositLevel = 0;
            robot.intakeArmDown();
            robot.closeIntake();
            robot.setIntakePower(1);
            robot.rightRampUp();
            robot.depositNeutral();
            timer = new ElapsedTime();
            timer.startTime();
            while (timer.seconds() < .35) {
                robot.drive(-.15, -.4, 0);
                robot.updatePositionRoadRunner();
            }
            robot.resetOdometry(0, robot.y, 3 * Math.PI / 2);
            robot.updatePositionRoadRunner();
            timer.reset();
            hitFreight = false;
            while(opModeIsActive()&&robot.y>-73&&(robot.y>-40||timer.seconds()<.39||robot.intakeArm.getCurrentPosition()<60)&&(robot.y>-40||robot.intakeSeeperDraw()<4.05||robot.intakeArm.getCurrentPosition()<60)) {
                telemetry.addData("x", robot.x);
                telemetry.addData("y", robot.y);
                telemetry.addData("theta", robot.theta);
                telemetry.addData("draw", robot.intakeSeeperDraw());
                telemetry.addData("time", timer.seconds());
                telemetry.update();
                if (robot.intakeSeeperDraw() < 3.9 && !hitFreight||robot.intakeArm.getCurrentPosition()<60) {
                    timer.reset();
                } else if (robot.intakeSeeperDraw() >= 3.9)
                    hitFreight = true;
                robot.deposit();
                double speed = robot.y < -32 ? -.34 : -.85;
                if (hitFreight)
                    speed = .065;
                if(timer.seconds()>.125)
                    robot.drive(0,0,0);
                else
                    robot.drive(speed, -.08, 0);
                robot.intake();
                robot.updatePositionRoadRunnerOnlyRight();
            }

            robot.resetOdometry(0,-77.5+robot.distanceSensor.getVoltage()/6.21*2000,3*Math.PI/2);

            timeStarted=false;
            t=new ElapsedTime();
            robot.setIntakePower(.6);
            while (opModeIsActive() && robot.y < -29) {
                if(robot.intakeSeeperDraw()>7)
                    robot.setIntakePower(-.3);
                robot.drive(1, -.25, 0);
                robot.deposit();
                robot.intakeArmUp();
                if (robot.intakeArm.getCurrentPosition() < 30||timeStarted) {

                    if(!timeStarted)
                    {
                        t.startTime();

                    }
                    timeStarted=true;
                    robot.openIntake();
                    if(t.seconds()<1.5) {
                        robot.intakeArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        robot.intakeArm.setPower(-1);
                    }
                    else {
                        robot.intakeArm.setPower(-1);
                        robot.intake();
                        robot.intakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                }
                else
                    robot.intake();
                robot.updatePositionRoadRunnerOnlyRight();
                telemetry.addData("x", robot.x);
                telemetry.addData("y", robot.y);
                telemetry.addData("theta", robot.theta);
                telemetry.update();
            }
            robot.resetDeltaTicks();
            robot.updatePositionRoadRunner();
            robot.resetOdometry(0, robot.y, 3 * Math.PI / 2);
            robot.updatePositionRoadRunner();
            //Drive back to hub
            while (opModeIsActive() & robot.x < 21.5) {

                robot.intake();
                if (robot.intakeArm.getCurrentPosition() < 30)
                    robot.openIntake();
                telemetry.addData("x", robot.x);
                telemetry.addData("y", robot.y);
                telemetry.addData("theta", robot.theta);
                telemetry.update();
                if (robot.x > 4) {
                    robot.deposit();
                    robot.depositLevel = 2;
                }
                if (robot.x < 4)
                    robot.updatePositionRoadRunnerRightAndCenter();
                else
                    robot.updatePositionRoadRunner();
                robot.drive(.485*(35-robot.x)/35, 1*(35-robot.x)/35, 0);
                if (robot.x > 17) {
                    robot.depositRight();
                    robot.rightRampDown();
                }

            }
            timer.reset();
            while(timer.seconds()<.3)
                robot.drive(0,0,0);

        }



        //Final Park
        //Drive back to wall
        while (opModeIsActive() & robot.x > 7) {

            telemetry.addData("x", robot.x);
            telemetry.addData("y", robot.y);
            telemetry.addData("theta", robot.theta);
            telemetry.update();
            robot.updatePositionRoadRunner();
            PathFollowers.linearTolerancePathFollow(robot, -.76, -1, 3 * Math.PI / 2, 1.5, .04, 0.2, 0, 3 * Math.PI / 2, new Point(20, -5));


        }
        robot.depositLevel = 0;
        robot.intakeArmDown();
        robot.closeIntake();
        robot.setIntakePower(1);
        robot.rightRampUp();
        robot.depositNeutral();
        timer = new ElapsedTime();
        timer.startTime();
        while (timer.seconds() < .25) {
            robot.drive(-.2, -.55, 0);
            robot.updatePositionRoadRunner();
        }
        robot.resetOdometry(0, robot.y, 3 * Math.PI / 2);
        robot.updatePositionRoadRunner();
        timer.reset();
        hitFreight = false;
        while(opModeIsActive()&&robot.y>-73&&(robot.y>-45||timer.seconds()<.39||robot.intakeArm.getCurrentPosition()<60)&&(robot.intakeSeeperDraw()<4.05||robot.intakeArm.getCurrentPosition()<60)) {
            telemetry.addData("x", robot.x);
            telemetry.addData("y", robot.y);
            telemetry.addData("theta", robot.theta);
            telemetry.addData("draw", robot.intakeSeeperDraw());
            telemetry.addData("time", timer.seconds());
            telemetry.update();
            if (robot.intakeSeeperDraw() < 3.9 && !hitFreight||robot.intakeArm.getCurrentPosition()<60) {
                timer.reset();
            } else if (robot.intakeSeeperDraw() >= 3.9)
                hitFreight = true;
            robot.deposit();
            double speed = robot.y < -30 ? -.35 : -.85;
            if (hitFreight)
                speed = .05;
            if(timer.seconds()>.1)
                robot.drive(0,0,0);
            else
                robot.drive(speed, -.08, 0);
            robot.intake();
            robot.updatePositionRoadRunnerOnlyRight();
        }

        //Drive back to wall
        while (opModeIsActive() & robot.x > 6.25) {

            telemetry.addData("x", robot.x);
            telemetry.addData("y", robot.y);
            telemetry.addData("theta", robot.theta);
            telemetry.update();
            robot.updatePositionRoadRunner();
            PathFollowers.linearTolerancePathFollow(robot, -.76, -1, 3 * Math.PI / 2, 1.5, .04, 0.2, 0, 3 * Math.PI / 2, new Point(20, -5));


        }
        //deployed intake and start brushes
        robot.depositLevel = 0;
        robot.intakeArmDown();
        robot.closeIntake();
        robot.setIntakePower(1);
        robot.rightRampUp();
        robot.depositNeutral();
        timer = new ElapsedTime();
        timer.startTime();
        while (timer.seconds() < .25) {
            robot.drive(-.2, -.55, 0);
            robot.updatePositionRoadRunner();
        }

        while(opModeIsActive())
        {
            robot.drive(0,0,0);
            robot.intake();
            robot.intakeArmUp();
            if(robot.intakeArm.getCurrentPosition()<30)
                robot.openIntake();
            if(totalAutoTime.seconds()<2)
                robot.setIntakePower(.7);
        }



    }

}
