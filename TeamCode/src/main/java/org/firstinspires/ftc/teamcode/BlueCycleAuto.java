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

            if (TSEFinder.screenPosition.x < 115)
                auto = 1;
            else if (TSEFinder.screenPosition.x < 215)
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
        while(opModeIsActive() && distanceMoved < 850)
        {
            distanceMoved = (robot.frontRight.getCurrentPosition() + robot.frontLeft.getCurrentPosition() + robot.backLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) / 4;
            robot.drive(.6, 0,0);

            telemetry.addData("distance moved", distanceMoved);
            telemetry.update();
        }

        AutoMethods.resetEncoders(robot);
        distanceMoved = 0;

        //drive to hub
        while(opModeIsActive() && distanceMoved > -925)
        {
            distanceMoved = (robot.frontLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) / 2;
            robot.drive(0, .5, 0);

            if(distanceMoved < -300)
                robot.deposit();

            if (distanceMoved < -500)
                robot.rightRampDown();
            if (distanceMoved < -600)
                robot.depositRight();

            telemetry.addData("distance moved", distanceMoved);
            telemetry.update();
        }

        //drives back to wall
        while(opModeIsActive() && distanceMoved < 0)
        {
            distanceMoved = (robot.frontLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) / 2;
            robot.drive(0, -.6, 0);

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

        //drives into warehouse and intakes 1
        while(opModeIsActive() && distanceMoved > -3200)
        {
            distanceMoved = (robot.frontRight.getCurrentPosition() + robot.frontLeft.getCurrentPosition() + robot.backLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) / 4;
            robot.drive(-.4, -.1, 0);

            robot.intakeArmDown();
            robot.closeIntake();
            //robot.intake();

            if(distanceMoved < -1200)
            {
                robot.setIntakePower(1);
            }

            if(robot.intakeSeeperDraw() >= 4.5)
                break;
            if(robot.intakeColorSensor.red() > ambientIntakeColor + 40)
                break;
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

        //drives from warehouse to center wall
        while(opModeIsActive() && distanceMoved < -400)
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
            if(intakeUp && t.seconds() > .7)
                robot.openIntake();
            if(intakeUp && t.seconds() > 1.2)
                robot.intakeArm.setPower(0);

            robot.setIntakePower(.5);

            distanceMoved = (robot.frontRight.getCurrentPosition() + robot.frontLeft.getCurrentPosition() + robot.backLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) / 4;
            robot.drive(.5, -.3, 0);

            if(robot.lineColorSensor.alpha() > lineValue * .99 && !hitLine)
            {
                hitLine=true;
                distanceMoved = -1850;
            }
            else
                hitLine = false;

            telemetry.addData("distance moved", distanceMoved);
            telemetry.addData("intake position", robot.intakeArm.getCurrentPosition());
            telemetry.addData("seconds", t.seconds());
            telemetry.addData("intake up", intakeUp);
            telemetry.addData("intake target position", robot.intakeArm.getTargetPosition());
            telemetry.addData("intake power", robot.intakeArm.getPower());
            telemetry.addData("intake mode", robot.intakeArm.getMode());
            telemetry.addData("color distance", robot.intakeColorSensor.getDistance(DistanceUnit.INCH));

            telemetry.update();
        }

        AutoMethods.resetEncoders(robot);
        distanceMoved = 0;

        robot.intakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.intakeArm.setPower(0);
        //robot.setIntakePower(0);

        //drive to hub to score freight
        robot.depositLevel = 3;
        while(opModeIsActive() && distanceMoved > -950)
        {
            distanceMoved = (robot.frontLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) / 2;
            robot.drive(0, .35, 0);

            if(distanceMoved < -300)
            {
                robot.deposit();
                robot.setIntakePower(0);
            }

            if (distanceMoved < -700)
                robot.rightRampDown();
            if (distanceMoved < -850)
                robot.depositRight();

            telemetry.addData("distance moved", distanceMoved);
            telemetry.update();
        }

        //drives back to wall
        while(opModeIsActive() && distanceMoved < -10)
        {
            distanceMoved = (robot.frontLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) / 2;
            robot.drive(0, -.6, 0);

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
            while(opModeIsActive() && distanceMoved > -3200)
            {
                robot.intakeArmDown();
                robot.closeIntake();

                distanceMoved = (robot.frontRight.getCurrentPosition() + robot.frontLeft.getCurrentPosition() + robot.backLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) / 4;
                robot.drive(-.5, -.1, 0);

                if(distanceMoved < -1200)
                {
                    robot.setIntakePower(1);
                }

                if(robot.lineColorSensor.alpha() > lineValue * .99 && !hitLine)
                {
                    hitLine=true;
                    distanceMoved = -1850;
                }
                else
                    hitLine = false;

                if(robot.intakeSeeperDraw() >= 4.5)
                {
                    //wait(100);
                    break;
                }
                if(robot.intakeColorSensor.red() > ambientIntakeColor + 40)
                {
                    //wait(100);
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
            int addDistance = 0;

            //goes from warehouse to center wall
            while(opModeIsActive() && distanceMoved < -500 + addDistance)
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
                if(intakeUp && t.seconds() > .7)
                    robot.openIntake();
                if(intakeUp && t.seconds() > 1.2)
                    robot.intakeArm.setPower(0);

                distanceMoved = (robot.frontRight.getCurrentPosition() + robot.frontLeft.getCurrentPosition() + robot.backLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) / 4;
                robot.drive(.5, -.3, 0);

                if(robot.lineColorSensor.alpha() > lineValue * .99 && !hitLine)
                {
                    hitLine=true;
                    distanceMoved = -1850;
                }

                if(!hitLine)
                {
                    addDistance = -200;
                }

                telemetry.addData("distance moved", distanceMoved);
                telemetry.addData("hit line?", hitLine);
                telemetry.update();
            }

            AutoMethods.resetEncoders(robot);
            distanceMoved = 0;

            //drive to hub
            while(opModeIsActive() && distanceMoved > -1100)
            {
                robot.setIntakePower(0);

                distanceMoved = (robot.frontLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) / 2;
                robot.drive(0, .35, 0);

                if(distanceMoved < -300)
                    robot.deposit();

                if (distanceMoved < -700)
                    robot.rightRampDown();
                if (distanceMoved < -1000)
                    robot.depositRight();

                telemetry.addData("distance moved", distanceMoved);
                telemetry.update();
            }

            robot.setIntakePower(-.5);

            //drives back to wall
            while(opModeIsActive() && distanceMoved < 10)
            {
                distanceMoved = (robot.frontLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) / 2;
                robot.drive(0, -.6, 0);

                telemetry.addData("distance moved", distanceMoved);
                telemetry.update();
            }

        } //end of subsequent cycles for loop

        AutoMethods.resetEncoders(robot);
        distanceMoved = 0;

        //drives to warehouse to park
        while(opModeIsActive() && distanceMoved > -2500)
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