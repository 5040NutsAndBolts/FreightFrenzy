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

import org.firstinspires.ftc.teamcode.AutoMethods;


@Autonomous(name = "BlueCycleAuto", group = "Auto")
public class BlueCycleAuto extends LinearOpMode
{
    //about 42.5 ticks per inch
    //x is between blue and red
    //y is along one color wall
    //FL BR strafe towards blue hub

    double distanceMoved = 0;

    int openCVDetectionLevel = 1;

    @Override
    public void runOpMode() throws InterruptedException
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Right Webcam"), cameraMonitorViewId);

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
        try
        {
            f = new FileWriter(Environment.getExternalStorageDirectory() + "/testingData.txt");
        } catch (IOException e) {
            e.printStackTrace();
        }
        robot.depositNeutral();
        //robot.resetOdometry(0, robot.y, 3 * Math.PI / 2);
        int[] colors = new int[3];
        colors[0] = robot.lineColorSensor.alpha();

        while (!isStopRequested() & !isStarted())
        {

            if (TSEFinder.screenPosition.x < 50)
                auto = 1;
            else if (TSEFinder.screenPosition.x < 150)
                auto = 2;
            else
                auto = 3;

            telemetry.addData("auto", auto);
            telemetry.update();
        }

        waitForStart();
        robot.setVerticalPosition(1);
        robot.setHorizontalPosition(.5);
        colors[1] = robot.lineColorSensor.alpha();
        double ambientIntakeColor = robot.intakeColorSensor.red();
        ElapsedTime totalAutoTime = new ElapsedTime();
        totalAutoTime.startTime();
        //Open CV goes here to spit out 1, 2, or 3

        //robot.intakeArmUp();
        robot.intakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        boolean setMode = true;
        boolean hitLine = false;

        robot.depositLevel = auto - 1;

        telemetry.addData("distance moved", distanceMoved);
        telemetry.update();

        //drive backwards to center wall
        while(opModeIsActive() && distanceMoved < 800)
        {
            distanceMoved = (robot.frontRight.getCurrentPosition() + robot.frontLeft.getCurrentPosition() + robot.backLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) / 4;
            robot.drive(1, 0,0);
            robot.deposit();

            telemetry.addData("distance moved", distanceMoved);
            telemetry.update();
        }

        AutoMethods.resetEncoders(robot);
        distanceMoved = 0;

        //drive to hub
        while(opModeIsActive() && distanceMoved > (-950+(auto==1?0:-150)))
        {
            distanceMoved = (robot.frontLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) / 2;
            robot.drive(0, .7+distanceMoved/2500, 0);
            robot.deposit();

            if (distanceMoved < -400)
                robot.rightRampDown();
            if (distanceMoved < -600)
                robot.depositRight();

            telemetry.addData("distance moved", distanceMoved);
            telemetry.update();
        }

        //drives back to wall
        while(opModeIsActive() && distanceMoved < -10)
        {
            distanceMoved = (robot.frontLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) / 2;
            robot.drive(0, -.8, 0);

            telemetry.addData("distance moved", distanceMoved);
            telemetry.update();
        }

        colors[2] = robot.lineColorSensor.alpha();

        Arrays.sort(colors);
        int lineValue= (int) Math.round((double)colors[1]*1.1);

        AutoMethods.resetEncoders(robot);
        distanceMoved = 0;

        robot.depositLevel = 0;
        robot.deposit();
        robot.depositNeutral();
        robot.rightRampUp();

        ElapsedTime intakeTimer = new ElapsedTime();
        intakeTimer.startTime();
        ElapsedTime curveTimer = new ElapsedTime();
        curveTimer.startTime();
        boolean color = false;
        boolean draw = false;
        //drives into warehouse and intakes 1
        while(opModeIsActive() && distanceMoved > -3450&&curveTimer.seconds()<5)
        {
            distanceMoved = (robot.frontRight.getCurrentPosition() + robot.frontLeft.getCurrentPosition() + robot.backLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) / 4;

            double curve=curveTimer.seconds()>1.6?.13:0;
            if(curveTimer.seconds()>3.5)
                curve=-.15;
            else if(curveTimer.seconds()>2.5)
                curve=.18;

            if(distanceMoved<-1400)
                robot.drive(-.2, 0, curve);
            else if(distanceMoved<-900)
                robot.drive(-1-distanceMoved/2000,-.1,0);
            else
                robot.drive(-.7, -.1, 0);

            robot.intakeArmDown();
            robot.closeIntake();
            //robot.intake();

            if(distanceMoved < -1200)
            {
                robot.setIntakePower(1);
                robot.intake();
            }

            if(robot.intakeSeeperDraw() < 4.9)
                intakeTimer.reset();
            else if(intakeTimer.seconds()>.1) {
                draw=true;
                break;
            }
            if(robot.intakeColorSensor.red() > ambientIntakeColor + 50) {
                color = true;
                break;
            }
            //if(robot.colorsensor.getDistance(DistanceUnit.INCH) > 1.5)
                //break;


            telemetry.addData("distance moved", distanceMoved);
            //telemetry.addData("intake draw", robot.intakeSeeperDraw());
            //telemetry.addLine("intaking");
            telemetry.update();
        }

        //.robot.setIntakePower(.7);

        ElapsedTime t = new ElapsedTime();
        //t.startTime();
        robot.resetEncoder=false;
        boolean intakeUp = false;

        double distanceMovedModifier=0;

        double finalCurveTime= curveTimer.seconds();
        ElapsedTime curveBack = new ElapsedTime();

        //drives from warehouse to center wall
        while(opModeIsActive() && distanceMoved < -600)
        {
            robot.intakeArmUp();
            robot.intake();
            if(robot.intakeArm.getCurrentPosition() < 10 && !intakeUp)
            {
                robot.intakeArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                intakeUp = true;
                t.reset();
                t.startTime();
            }

            if(intakeUp)
                robot.intakeArm.setPower(-.7);
            if(intakeUp && t.seconds() > .05)
            {
                robot.reallyOpenIntake();
                robot.setIntakePower(0);
            }
            else
                robot.setIntakePower(.7);

            distanceMoved = (robot.frontRight.getCurrentPosition() + robot.frontLeft.getCurrentPosition() + robot.backLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) / 4+distanceMovedModifier;
            double curve=0;
            if(finalCurveTime-curveBack.seconds()>0)
                curve=-.1;
            robot.drive(.7, -.3, curve);

            if(robot.lineColorSensor.alpha() > lineValue * .985 && !hitLine)
            {
                hitLine=true;
                distanceMovedModifier=-distanceMoved-1850;
            }
            else
                hitLine = false;

            if(finalCurveTime-curveBack.seconds()>0)


            telemetry.addData("hit line", hitLine);
            telemetry.update();
        }

        AutoMethods.resetEncoders(robot);
        distanceMoved = 0;

        robot.intakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.intakeArm.setPower(0);
        //robot.setIntakePower(0);

        //drive to hub to score freight
        robot.depositLevel = 3;
        while(opModeIsActive() && distanceMoved > -1250)
        {
            robot.setIntakePower(1);
            distanceMoved = (robot.frontLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) / 2;
            robot.drive(0, .6+distanceMoved/3600, 0);

            if(distanceMoved < -300)
            {
                robot.deposit();
                robot.setIntakePower(0);
            }

            if (distanceMoved < -800)
                robot.rightRampDown();
            if (distanceMoved < -950)
                robot.depositRight();

            telemetry.addData("hit line", hitLine);
            telemetry.update();
        }

        //drives back to wall
        while(opModeIsActive() && distanceMoved < 200)
        {
            distanceMoved = (robot.frontLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) / 2;
            robot.drive(0, -.65, 0);
            robot.setIntakePower(-.5);

            if(distanceMoved>-50)
            {
                robot.depositLevel=0;
                robot.deposit();
            }
            telemetry.addData("distance moved", distanceMoved);
            telemetry.update();
        }

        //subsequent cycles
        for(int i = 0; i < 3 && totalAutoTime.seconds() < 23.8; i++)
        {
            AutoMethods.resetEncoders(robot);
            distanceMoved = 0;

            robot.depositLevel = 0;
            robot.deposit();
            robot.depositNeutral();
            robot.rightRampUp();
            robot.setIntakePower(0);

            //drives into warehouse 2+
            color=false;
            draw=false;
            distanceMovedModifier=0;
            curveTimer.reset();
            while(opModeIsActive() && distanceMoved > -3550&&curveTimer.seconds()<5)
            {
                robot.intakeArmDown();
                robot.closeIntake();

                distanceMoved = (robot.frontRight.getCurrentPosition() + robot.frontLeft.getCurrentPosition() + robot.backLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) / 4+distanceMovedModifier;
                double curve=curveTimer.seconds()>1.6?.13:0;
                if(curveTimer.seconds()>3.5)
                    curve=-.15;
                else if(curveTimer.seconds()>2.5)
                    curve=.18;

                if(distanceMoved<-1400)
                    robot.drive(-.2, 0, curve);
                else if(distanceMoved<-900)
                    robot.drive(-1-distanceMoved/2000,-.1,0);
                else
                    robot.drive(-.7, -.1, 0);

                if(distanceMoved<-200)
                    robot.intake();
                if(distanceMoved < -1200)
                    robot.setIntakePower(1);


                if(robot.lineColorSensor.alpha() > lineValue * .99 && !hitLine)
                {
                    hitLine=true;
                    distanceMovedModifier=-distanceMoved-1850;
                }
                else
                    hitLine = false;

                if(robot.intakeSeeperDraw() < 4.9)
                    intakeTimer.reset();
                else if(intakeTimer.seconds()>.1) {
                    draw=true;
                }
                if(robot.intakeColorSensor.red() > ambientIntakeColor + 50)
                {
                    color=true;
                    break;
                }

                telemetry.addData("distance moved", distanceMoved);
                telemetry.addLine("entering warehouse");
                telemetry.update();
            }

            robot.depositLevel = 2;
            intakeUp = false;
            robot.resetEncoder = false;
            hitLine = false;
            finalCurveTime=curveTimer.seconds();
            curveBack.reset();

            //goes from warehouse to center wall
            while(opModeIsActive() && distanceMoved < -600 )
            {

                robot.intakeArmUp();
                robot.intake();
                if(robot.intakeArm.getCurrentPosition() < 10 && !intakeUp)
                {
                    robot.intakeArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    intakeUp = true;
                    t.reset();
                    t.startTime();
                }

                if(intakeUp)
                    robot.intakeArm.setPower(-.8);
                if(intakeUp && t.seconds() > .05)
                {
                    robot.reallyOpenIntake();
                    robot.setIntakePower(0);
                }
                else
                    robot.setIntakePower(.7);



                distanceMoved = (robot.frontRight.getCurrentPosition() + robot.frontLeft.getCurrentPosition() + robot.backLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) / 4;
                double curve=0;
                if(finalCurveTime-curveBack.seconds()>0)
                    curve=-.1;
                robot.drive(.7, -.3, curve);

                if(robot.lineColorSensor.alpha() > lineValue * .985 && !hitLine)
                {
                    hitLine=true;
                    distanceMoved = -1850;
                }



                telemetry.addData("hit line", hitLine);
                telemetry.update();
            }

            AutoMethods.resetEncoders(robot);
            distanceMoved = 0;
            robot.reallyOpenIntake();

            //drive to hub
            while(opModeIsActive() && distanceMoved > -1250)
            {

                robot.intake();
                robot.setIntakePower(1);

                distanceMoved = (robot.frontLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) / 2;
                robot.drive(0, .6+distanceMoved/3600, 0);

                if(distanceMoved < -300)
                    robot.deposit();

                if (distanceMoved < -800)
                    robot.rightRampDown();
                if (distanceMoved < -1050)
                    robot.depositRight();

                telemetry.addData("distance moved", distanceMoved);
                telemetry.update();
            }


            robot.intakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //drives back to wall1
            while(opModeIsActive() && distanceMoved < 75)
            {
                robot.setIntakePower(-.5);
                distanceMoved = (robot.frontLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) / 2;
                if(distanceMoved>-50)
                {
                    robot.depositLevel=0;
                    robot.deposit();
                }
                robot.drive(0, -.6, 0);

                telemetry.addData("distance moved", distanceMoved);
                telemetry.update();

            }

        } //end of subsequent cycles for loop
        robot.intakeArm.setPower(0);
        AutoMethods.resetEncoders(robot);
        distanceMoved = 0;

        //drives to warehouse to park
        while(opModeIsActive() && distanceMoved > -2450)
        {
            distanceMoved = (robot.frontRight.getCurrentPosition() + robot.frontLeft.getCurrentPosition() + robot.backLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) / 4;
            robot.drive(-.8, -.2, 0);

            robot.depositLevel = 0;
            robot.deposit();
            robot.rightRampUp();
            robot.depositNeutral();

            telemetry.addData("distance moved", distanceMoved);
            telemetry.update();
        }

        robot.drive(0,0,0);
    }
}