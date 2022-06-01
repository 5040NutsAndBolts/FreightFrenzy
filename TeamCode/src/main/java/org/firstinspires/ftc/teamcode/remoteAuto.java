package org.firstinspires.ftc.teamcode;

import android.app.Application;
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
import java.util.Arrays;
import java.util.Scanner;
import java.util.function.Function;
import java.util.regex.Pattern;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
    //about 42.5 ticks per inch
    //x is between blue and red
    //y is along one color wall
    //fl br strafe towards blue hub

    double distanceMoved = 0;
    final double TPI = 42.5;
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

        int auto = 3;

        Hardware.currentOpMode = this;
        Hardware robot = new Hardware(hardwareMap);
        FileWriter f = null;
        robot.resetStaticMotors();
        try {
            f = new FileWriter(Environment.getExternalStorageDirectory()+"/testingData.txt");
        } catch (IOException e) {
            e.printStackTrace();
        }
        robot.depositNeutral();
        //robot.resetOdometry(0, robot.y, 3 * Math.PI / 2);
        int[] colors=new int[3];
        colors[0] = robot.lineColorSensor.alpha();
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
        robot.setVerticalPosition(1);
        robot.setHorizontalPosition(.5);
        colors[1] = robot.lineColorSensor.alpha();
        double ambientIntakeColor=robot.intakeColorSensor.red();
        ElapsedTime totalAutoTime = new ElapsedTime();
        totalAutoTime.startTime();
        //Open CV goes here to spit out 1, 2, or 3
        //moves robot to shipping hub

        robot.intakeArmUp();
        robot.intakeArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //moves the robot closer to hub in auto 3
        double towardsHub=auto==3?53:0;

        boolean setMode=true;

        //drives towards hub to deposit preload
        while (opModeIsActive() && distanceMoved < 786.25+towardsHub)
        {
            distanceMoved = (robot.backRight.getCurrentPosition()  + robot.frontLeft.getCurrentPosition()) / 2;
            if(distanceMoved < 510)
            {
                robot.depositLevel = auto==3?2:1;
                if(distanceMoved > 382.5)
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
            PathFollowers.linearTolerancePathFollow(robot, -.05, 1.7-robot.x/14.7-(auto==1?.2:0), 3 * Math.PI / 2, .9, .05, 0.2, .15, 3 * Math.PI / 2, new Point(0, 0));

            if (distanceMoved > 187)
                robot.rightRampDown();
            if (distanceMoved > 365.5)
                robot.depositRight();

            Hardware.currentOpMode.telemetry.addData("x", robot.x);
            Hardware.currentOpMode.telemetry.addData("y", robot.y);
            Hardware.currentOpMode.telemetry.addData("theta", robot.theta);
            Hardware.currentOpMode.telemetry.update();
        } //end of drive to hub while loop

        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        distanceMoved = 0;

        ElapsedTime waitAtHub = new ElapsedTime();
        waitAtHub.startTime();
        while(opModeIsActive() && waitAtHub.seconds()<.38)
            robot.drive(0, 0, 0);

        ElapsedTime slowToWall = new ElapsedTime();

        //Drive back to wall
        while (opModeIsActive() && distanceMoved < 722.5)
        {
            distanceMoved = (robot.backLeft.getCurrentPosition()  + robot.frontRight.getCurrentPosition()) / 2;

            telemetry.addData("x", robot.x);
            telemetry.addData("y", robot.y);
            telemetry.addData("theta", robot.theta);
            telemetry.update();
            robot.updatePositionRoadRunner();
            double speedPercent=Math.min(slowToWall.seconds()/.35,.62)+.38;
            robot.drive(-.25*speedPercent, -1*speedPercent,0);
        }

        colors[2] = robot.lineColorSensor.alpha();

        Arrays.sort(colors);
        int lineValue= (int) Math.round((double)colors[1]*1.1);

        //deployed intake and start brushes
        robot.depositLevel = 0;
        robot.intakeArmDown();
        robot.closeIntake();
        robot.setIntakePower(1);
        robot.rightRampUp();
        robot.depositNeutral();
        ElapsedTime timer = new ElapsedTime();
        timer.startTime();
        while (opModeIsActive() && timer.seconds() < .3) {
            robot.drive(-.21, -.3-timer.seconds()/1.8, 0);
            robot.updatePositionRoadRunner();
        }

        //robot.resetOdometry(0, robot.y, 3 * Math.PI / 2);
        robot.updatePositionRoadRunner();
        timer.reset();
        ElapsedTime t=new ElapsedTime();
        boolean timeStarted=false;
        boolean hitFreight = false;
        robot.closeIntake();
        ElapsedTime intakePowerOff = new ElapsedTime();
        intakePowerOff.startTime();
        ElapsedTime rotate = new ElapsedTime();
        rotate.startTime();

        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        distanceMoved = 0;

        //drive into warehouse to grab freight
        while(opModeIsActive() &&
                distanceMoved < 3230 &&
                (distanceMoved < 1700 || timer.seconds() < .07 || robot.colorsensor.getDistance(DistanceUnit.INCH)<1.5) &&
                (distanceMoved < 1700 || (robot.intakeSeeperDraw() < 4.6 &&
                        robot.intakeColorSensor.red()<ambientIntakeColor+20) || robot.colorsensor.getDistance(DistanceUnit.INCH)<1.5) &&
                timer.seconds()<1)
        {
            distanceMoved = (robot.frontLeft.getCurrentPosition() + robot.frontRight.getCurrentPosition() + robot.backLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) /4;

            telemetry.addData("x", robot.x);
            telemetry.addData("y", robot.y);
            telemetry.addData("theta", robot.theta);
            telemetry.addData("draw", robot.intakeSeeperDraw());
            telemetry.addData("time", timer.seconds());
            telemetry.update();

            if ((robot.intakeSeeperDraw() < 4.5 || distanceMoved < 1700) && !hitFreight)
            {
                timer.reset();
                intakePowerOff.reset();
            }
            else if (robot.intakeSeeperDraw() >= 4.5)
                hitFreight = true;

            robot.deposit();
            if(intakePowerOff.seconds() > 2)
            {
                robot.setIntakePower(0);
                if(intakePowerOff.seconds() > 2.2)
                    intakePowerOff.reset();
            }
            else
            {
                robot.setIntakePower(.89);
            }
            double speed = distanceMoved < 1232.5 ? (distanceMoved < 1487.5?-.19:-.255) : -.69;

            //slow down after the robot has freight
            if (hitFreight)
                speed = .04;
            double rotation = rotate.seconds() > 2.7?.1:0;
            robot.drive(speed, rotate.seconds() > 2.7?0:-.1, rotation);
            robot.intake();
            robot.updatePositionRoadRunnerOnlyRight();
        } //end of drive to warehouse while loop

        //use ultra sonic sensor to reset position
        if(robot.distanceSensor.getVoltage() / 6.234 * 2000 < 40)
        {
            //robot.resetOdometry(0, -86 + robot.distanceSensor.getVoltage() / 6.234 * 2000, 3 * Math.PI / 2);
            distanceMoved = 3655 + robot.distanceSensor.getVoltage() / 6.234 * 2000;
        }

        boolean hitLine=false;

        robot.setIntakePower(.23);

        ElapsedTime slowStrafe = new ElapsedTime();
        slowStrafe.startTime();
        intakePowerOff.reset();

        //ensure robot is fully in warehouse
        while (opModeIsActive() && distanceMoved < 2061.25)
        {
            //if motor stalls run intake backwards to decrease freight

            robot.setIntakePower(.23);
            robot.deposit();
            robot.drive(-.3, -.4+HelperMethods.clamp(0,slowStrafe.seconds()/1.5,.2), 0);

            //raise intake arm
            robot.intakeArmUp();
            if (robot.intakeArm.getCurrentPosition() < 10||timeStarted) {

                if(!timeStarted)
                {
                    t.startTime();

                }
                timeStarted=true;
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
            robot.updatePositionRoadRunnerOnlyRight();
            telemetry.addData("x", robot.x);
            telemetry.addData("y", robot.y);
            telemetry.addData("theta", robot.theta);
            telemetry.update();
        } //end of fully in warehouse while loop

        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        distanceMoved = 0;

        //leaves warehouse I think
        //driving backwards so distance should be negative
        while (opModeIsActive() && distanceMoved > -1153.88)
        {
            distanceMoved = (robot.frontLeft.getCurrentPosition() + robot.frontRight.getCurrentPosition() + robot.backLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) /4;

            if(robot.lineColorSensor.alpha() > lineValue * .99 && !hitLine)
            {
                hitLine=true;
                distanceMoved = 1487.5;
                robot.setIntakePower(0);
            }

            robot.deposit();
            robot.drive(1, -.4+HelperMethods.clamp(0,slowStrafe.seconds()/1.5,.2), 0);

            //raise intake arm
            robot.intakeArmUp();
            if (robot.intakeArm.getCurrentPosition() < 10||timeStarted) {

                if(!timeStarted)
                {
                    t.startTime();

                }
                timeStarted=true;
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
            robot.updatePositionRoadRunnerOnlyRight();
            telemetry.addData("x", robot.x);
            telemetry.addData("y", robot.y);
            telemetry.addData("theta", robot.theta);
            telemetry.update();
        } //end of leave warehouse while loop

        robot.setIntakePower(1);
        robot.resetDeltaTicks();
        robot.updatePositionRoadRunner();
        robot.resetOdometry(0, robot.y, 3 * Math.PI / 2);
        robot.updatePositionRoadRunner();

        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        distanceMoved = 0;

        //Drive back to hub to cycle first freight
        while (opModeIsActive() & distanceMoved < 811.75)
        {
            distanceMoved = (robot.backRight.getCurrentPosition()  + robot.frontLeft.getCurrentPosition()) / 2;

            robot.intake();
            if (robot.intakeArm.getCurrentPosition() < 10)
                robot.openIntake();

            telemetry.addData("x", robot.x);
            telemetry.addData("y", robot.y);
            telemetry.addData("theta", robot.theta);
            telemetry.update();

            if (distanceMoved > 178.5) {
                robot.deposit();
                robot.depositLevel = 2;
                robot.closeIntake();
            }
            if (distanceMoved < 170)
                robot.updatePositionRoadRunnerRightAndCenter();
            else
            {
                robot.updatePositionRoadRunner();
                robot.setIntakePower(-.5);
            }
            robot.drive(.1, 1, 0);
            if (distanceMoved > 745.875)
            {
                robot.depositRight();
                if(distanceMoved > 769.25)
                    robot.rightRampDown();
            }

        } //end of drive back to hub while loop

        //stop at hub to deposit
        timer.reset();
        while(opModeIsActive() && timer.seconds() < .38)
            robot.drive(0,0,0);
        boolean stopped=false;

        //This loop runs all cycles except for the first
        for(int i = 0; i < 3 && totalAutoTime.seconds() < 23.8; i++)
        {

            ElapsedTime proportionalSpeedIntoWall = new ElapsedTime();
            proportionalSpeedIntoWall.startTime();

            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            distanceMoved = 0;

            //Drive back to wall
            while (opModeIsActive() & distanceMoved < 306)
            {
                distanceMoved = (robot.backLeft.getCurrentPosition()  + robot.frontRight.getCurrentPosition()) / 2;

                telemetry.addData("x", robot.x);
                telemetry.addData("x", robot.x);
                telemetry.addData("y", robot.y);
                telemetry.addData("theta", robot.theta);
                telemetry.update();
                robot.updatePositionRoadRunner();
                double speedPercent=Math.min(proportionalSpeedIntoWall.seconds()/.4,.5)+.5;
                PathFollowers.linearTolerancePathFollow(robot, -.3*speedPercent, -1*speedPercent, 3 * Math.PI / 2, 1.5, .04, 0.2, 0, 3 * Math.PI / 2, new Point(20, -5));

            } //end of drive back to wall while loop

            //deployed intake and start brushes
            robot.depositLevel = 0;
            robot.intakeArmDown();
            robot.closeIntake();
            robot.setIntakePower(1);
            robot.rightRampUp();
            robot.depositNeutral();
            timer = new ElapsedTime();
            //square up to wall
            timer.startTime();

            while (opModeIsActive()&&timer.seconds() < .45)
            {
                robot.drive(-.19, -.34-timer.seconds()*1.5, 0);
                robot.updatePositionRoadRunner();
            }

            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            distanceMoved = 0;

            robot.updatePositionRoadRunner();
            timer.reset();
            intakePowerOff.reset();
            double totalWiggle=0;
            hitFreight = false;
            ElapsedTime wiggle = new ElapsedTime();
            wiggle.startTime();

            //intakes freight I think
            while(opModeIsActive() &&
                    wiggle.seconds() < 4.6 &&
                    distanceMoved < 3102.5 &&
                    (distanceMoved < 1657.5 || timer.seconds() < .17 || robot.colorsensor.getDistance(DistanceUnit.INCH) < 1.5) &&
                    (distanceMoved < 1657.5 || (robot.intakeColorSensor.red() < ambientIntakeColor + 20 &&
                            robot.intakeSeeperDraw() < 5) || robot.colorsensor.getDistance(DistanceUnit.INCH) < 1.5) &&
                    timer.seconds() < 1)
            {
                distanceMoved = (robot.frontLeft.getCurrentPosition() + robot.frontRight.getCurrentPosition() + robot.backLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) /4;

                telemetry.addData("x", robot.x);
                telemetry.addData("y", robot.y);
                telemetry.addData("theta", robot.theta);
                telemetry.addData("draw", robot.intakeSeeperDraw());
                telemetry.addData("color",robot.colorsensor.getDistance(DistanceUnit.INCH));
                telemetry.update();

                if ((robot.intakeSeeperDraw() < 4.5 || robot.colorsensor.getDistance(DistanceUnit.INCH) < 1.5 || distanceMoved < 1657.5) && !hitFreight) {
                    timer.reset();
                    intakePowerOff.reset();
                } else if (robot.intakeSeeperDraw() >= 4.5)
                    hitFreight = true;

                if(intakePowerOff.seconds()>2)
                {
                    robot.setIntakePower(0);
                    if(intakePowerOff.seconds()>2.14)
                        intakePowerOff.reset();
                }
                else
                {
                    robot.setIntakePower(1);
                }

                robot.deposit();
                double speed = distanceMoved < 1368.5 - .02 * i ? (distanceMoved < 2232.25?-.21:-.255-i*.025) : -.73-i*.025;
                double rotation = wiggle.seconds()>3.05 ? (wiggle.seconds()>3.25?.23:.13):0;

                if (hitFreight)
                    speed = .065;
                else if(wiggle.seconds()>3.8)
                    speed=-.12;

                if(timer.seconds()>.05)
                    robot.drive(0,0,0);
                else
                    robot.drive(speed, wiggle.seconds()>2.9 ?0:-.22, rotation);

                robot.intake();
                robot.updatePositionRoadRunnerOnlyRight();
                totalWiggle=wiggle.seconds();
            } //end of intake freight while loop

            //use ultra sonic sensor to reset position
            if(robot.distanceSensor.getVoltage()/6.234*2000<40)
            {
                //robot.resetOdometry(0, -84.5 + robot.distanceSensor.getVoltage() / 6.234 * 2000, 3 * Math.PI / 2);
                distanceMoved = 3591.25 + robot.distanceSensor.getVoltage() / 6.234 * 2000;
            }

            timeStarted=false;
            t=new ElapsedTime();
            slowStrafe = new ElapsedTime();
            slowStrafe.startTime();
            robot.setIntakePower(.23);
            ElapsedTime outtakeTimer = new ElapsedTime();
            boolean outtake=false;
            outtakeTimer.startTime();
            robot.intakeArmUp();

            hitLine=false;


            //make sure robot fully enters warehouse
            while (opModeIsActive() && distanceMoved < 2496.875)
            {
                distanceMoved = (robot.frontLeft.getCurrentPosition() + robot.frontRight.getCurrentPosition() + robot.backLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) /4;

                //if motor is stalling run intake backwards to free freight
                /*if(robot.intakeSeeperDraw()>8.2)
                {
                    outtakeTimer.reset();
                    outtake=true;
                }
                if(outtake&&outtakeTimer.seconds()<.1)
                    robot.setIntakePower(-.15);
                else if (outtakeTimer.seconds()>.1) {
                    outtakeTimer.reset();
                    robot.setIntakePower(.22);
                }*/

                robot.drive(-.55, -.4+HelperMethods.clamp(0,1-slowStrafe.seconds()/1.5,.2), 0);
                robot.deposit();
                //bring intake arm up
                robot.intakeArmUp();
                if (robot.intakeArm.getCurrentPosition() < 10||timeStarted) {

                    if(!timeStarted)
                    {
                        t.startTime();

                    }
                    timeStarted=true;
                    robot.openIntake();
                    if(t.seconds()<1.8) {
                        robot.intakeArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        robot.intakeArm.setPower(-.9);
                    }
                    else {
                        robot.intakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.intakeArm.setPower(1);
                        robot.intake();
                    }
                }
                else
                    robot.intake();
                robot.updatePositionRoadRunnerOnlyRight();
                telemetry.addData("x", robot.x);
                telemetry.addData("y", robot.y);
                telemetry.addData("theta", robot.theta);
                telemetry.update();
            } //end of enters warehouse while loop

            robot.intakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.setIntakePower(.23);
            robot.resetDeltaTicks();
            robot.updatePositionRoadRunner();

            if(totalAutoTime.seconds()>25.5)
            {
                stopped=true;
                break;
            }

            //Leave warehouse
            while (opModeIsActive() && distanceMoved > 1243.13)
            {
                distanceMoved = (robot.frontLeft.getCurrentPosition() + robot.frontRight.getCurrentPosition() + robot.backLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) /4;

                if(robot.lineColorSensor.alpha()>lineValue&&!hitLine)
                {
                    hitLine=true;
                    robot.resetOdometry(0, -35, 3 * Math.PI / 2);
                    robot.setIntakePower(0);
                }

                //if motor is stalling run intake backwards to free freight
                /*if(robot.intakeSeeperDraw()>8.5)
                {
                    outtakeTimer.reset();
                    outtake=true;
                }
                if(outtake&&outtakeTimer.seconds()<.1)
                    robot.setIntakePower(-.15);
                else if (outtakeTimer.seconds()>.1) {
                    outtakeTimer.reset();
                    robot.setIntakePower(.22);
                }*/
                double rotation=0;
                if(totalWiggle>3.4&& distanceMoved > 2125)
                    rotation=-.05;
                robot.drive(1, -.4+HelperMethods.clamp(0,1-slowStrafe.seconds()/1.5,.2), rotation);
                robot.deposit();
                //bring intake arm up
                robot.intakeArmUp();
                if (robot.intakeArm.getCurrentPosition() < 10||timeStarted) {

                    if(!timeStarted)
                    {
                        t.startTime();

                    }
                    timeStarted=true;
                    robot.openIntake();
                    if(t.seconds()<1.8) {
                        robot.intakeArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        robot.intakeArm.setPower(-.9);
                    }
                    else {
                        robot.intakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.intakeArm.setPower(1);
                        robot.intake();
                    }
                }
                else
                    robot.intake();
                robot.updatePositionRoadRunnerOnlyRight();
                telemetry.addData("x", robot.x);
                telemetry.addData("y", robot.y);
                telemetry.addData("theta", robot.theta);
                telemetry.update();
            } //end of leave warehouse loop

            robot.intakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.setIntakePower(1);
            robot.resetDeltaTicks();
            robot.updatePositionRoadRunner();
            robot.resetOdometry(0, robot.y, 3 * Math.PI / 2);
            robot.updatePositionRoadRunner();

            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            distanceMoved = 0;

            //Drive back to hub to score freight
            while (opModeIsActive() & distanceMoved < 977.5)
            {
                distanceMoved = (robot.backRight.getCurrentPosition()  + robot.frontLeft.getCurrentPosition()) / 2;

                robot.intake();

                if (robot.intakeArm.getCurrentPosition() < 10)
                    robot.openIntake();

                telemetry.addData("x", robot.x);
                telemetry.addData("y", robot.y);
                telemetry.addData("theta", robot.theta);
                telemetry.update();

                //Send up deposit for scoring freight
                if (distanceMoved > 178.5) {
                    robot.deposit();
                    robot.depositLevel = 2;
                    robot.closeIntake();
                }
                if (distanceMoved < 170)
                    robot.updatePositionRoadRunnerRightAndCenter();
                else
                {
                    robot.updatePositionRoadRunner();
                    robot.setIntakePower(-.5);
                }

                //drive in direction of hub with proportional slowing as it approaches
                robot.drive(.12*(1572.5-distanceMoved)/37, 1*(1572.5-distanceMoved)/37, 0);

                //send freight out
                if (distanceMoved > 888.25)
                {
                    robot.depositRight();
                    if(distanceMoved > 892.5)
                    robot.rightRampDown();
                }

            } //end of drive to hun loop

            //Stop at hub to deposit
            timer.reset();
            while(opModeIsActive() && timer.seconds()<.39)
                robot.drive(0,0,0);

        }

        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        distanceMoved = 0;

        //Final Park
        //Drive back to wall
        while (opModeIsActive() && distanceMoved < 722.5 && !stopped)
        {
            distanceMoved = (robot.backLeft.getCurrentPosition()  + robot.frontRight.getCurrentPosition()) / 2;

            telemetry.addData("x", robot.x);
            telemetry.addData("y", robot.y);
            telemetry.addData("theta", robot.theta);
            telemetry.update();
            robot.updatePositionRoadRunner();
            PathFollowers.linearTolerancePathFollow(robot, -.76, -1, 3 * Math.PI / 2, 1.5, .04, 0.2, 0, 3 * Math.PI / 2, new Point(20, -5));
        }

        robot.intakeArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.depositLevel = 0;
        robot.intakeArmUp();

        robot.setIntakePower(0);
        robot.rightRampUp();
        robot.depositNeutral();
        timer = new ElapsedTime();
        timer.startTime();
        robot.intakeArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ElapsedTime intakeUpTimer = new ElapsedTime();
        intakeUpTimer.startTime();

        while (opModeIsActive() && timer.seconds() < .23 && !stopped)
        {
            robot.drive(-.2, -.55, 0);
            robot.updatePositionRoadRunner();
            if(robot.depositSlide.getCurrentPosition()<40)
                robot.intakeArm.setPower(-1);
            if(robot.intakeArm.getCurrentPosition()<10)
            {
                robot.openIntake();
            }
            else
                intakeUpTimer.reset();
            if(intakeUpTimer.seconds()>.5)
                robot.setIntakePower(0);
        }

        //robot.resetOdometry(0, robot.y, 3 * Math.PI / 2);

        robot.updatePositionRoadRunner();
        timer.reset();
        boolean up=false;

        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        distanceMoved = 0;

        //drive to warehouse for final park
        while(opModeIsActive() && distanceMoved < 2507 && !stopped)
        {
            distanceMoved = (robot.frontLeft.getCurrentPosition() + robot.frontRight.getCurrentPosition() + robot.backLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) /4;

            if(robot.depositSlide.getCurrentPosition() < 40)
                robot.intakeArm.setPower(-1);
            double speed = distanceMoved > 1445 ? -.4 : -.95;
            robot.drive(speed, -.08, 0);
            if(robot.colorsensor.getDistance(DistanceUnit.INCH)<1.5||up) {
                robot.openIntake();
                robot.intakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.intakeArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                up=true;
            }
            else
                intakeUpTimer.reset();
            if(intakeUpTimer.seconds()>1)
                robot.intakeArm.setPower(0);
            else if(up)
                robot.intakeArm.setPower(-.55);
            robot.deposit();
            robot.updatePositionRoadRunnerOnlyRight();
        }

        robot.setIntakePower(0);



        //Stop in place for park
        while(opModeIsActive() && intakeUpTimer.seconds() < 1.5)
        {
            robot.drive(0,0,0);
            if(robot.colorsensor.getDistance(DistanceUnit.INCH) < 1.5||up)
            {
                robot.openIntake();
                robot.intakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.intakeArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                up=true;
            }
            else
                intakeUpTimer.reset();
            if(intakeUpTimer.seconds()>.1)
                robot.intakeArm.setPower(0);
            else if(up)
                robot.intakeArm.setPower(-.55);
            robot.deposit();

        }

    }

}
