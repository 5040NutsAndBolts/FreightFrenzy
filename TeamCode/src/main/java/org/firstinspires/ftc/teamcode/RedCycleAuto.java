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


@Autonomous(name = "RedCycleAuto", group = "Auto")
public class RedCycleAuto extends LinearOpMode
{
    //about 42.5 ticks per inch
    //x is between blue and red
    //y is along one color wall
    //FR BL strafe towards red hub

    double distanceMoved = 0;

    int openCVDetectionLevel = 1;

    @Override
    public void runOpMode() throws InterruptedException
    {
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
        //moves robot to shipping hub

        robot.intakeArmUp();
        robot.intakeArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        boolean setMode = true;

        robot.depositLevel = auto - 1;

        //drive to hub
        while(opModeIsActive() && distanceMoved < 786.25)
        {
            distanceMoved = (robot.frontRight.getCurrentPosition() + robot.backLeft.getCurrentPosition()) / 2;
            robot.drive(0, .6, 0);

            if(distanceMoved < 510)
                robot.deposit();

            if (distanceMoved > 187)
                robot.leftRampDown();
            if (distanceMoved > 365.5)
                robot.depositLeft();
        }

        //drives back to wall
        while(opModeIsActive() && distanceMoved > 10)
        {
            distanceMoved = (robot.frontRight.getCurrentPosition() + robot.backLeft.getCurrentPosition()) / 2;
            robot.drive(0, -.6, 0);
        }

        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        distanceMoved = 0;

        //drives into warehouse and intakes
        while(opModeIsActive() && distanceMoved < 3060)
        {
            distanceMoved = (robot.frontRight.getCurrentPosition() + robot.frontLeft.getCurrentPosition() + robot.backLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) / 4;
            robot.drive(.8, 0, 0);

            if(distanceMoved > 2040)
            {
                robot.setIntakePower(.89);
            }

            if(robot.intakeSeeperDraw() >= 4.5)
            {
                robot.setIntakePower(0);
                robot.intake();
                break;
            }
        }

        //drives back to center wall
        while(opModeIsActive() && distanceMoved > 0)
        {
            distanceMoved = (robot.frontRight.getCurrentPosition() + robot.frontLeft.getCurrentPosition() + robot.backLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) / 4;
            robot.drive(-.8, 0, 0);
        }

        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        distanceMoved = 0;

        //drive to hub to score freight
        robot.depositLevel = 3;
        while(opModeIsActive() && distanceMoved < 786.25)
        {
            distanceMoved = (robot.frontRight.getCurrentPosition() + robot.backLeft.getCurrentPosition()) / 2;
            robot.drive(0, .6, 0);

            if(distanceMoved < 510)
                robot.deposit();

            if (distanceMoved > 187)
                robot.leftRampDown();
            if (distanceMoved > 365.5)
                robot.depositLeft();
        }

        //drives back to wall
        while(opModeIsActive() && distanceMoved > 10)
        {
            distanceMoved = (robot.frontRight.getCurrentPosition() + robot.backLeft.getCurrentPosition()) / 2;
            robot.drive(0, -.6, 0);
        }

        //subsequent cycles
        for(int i = 0; i < 3 && totalAutoTime.seconds() < 23.8; i++)
        {
            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            distanceMoved = 0;

            //drives to warehouse
            while(opModeIsActive() && distanceMoved < 3060)
            {
                distanceMoved = (robot.frontRight.getCurrentPosition() + robot.frontLeft.getCurrentPosition() + robot.backLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) / 4;
                robot.drive(.8, 0, 0);

                if(distanceMoved > 2040)
                {
                    robot.setIntakePower(.89);
                }

                if(robot.intakeSeeperDraw() >= 4.5)
                {
                    robot.setIntakePower(0);
                    robot.intake();
                    break;
                }
            }

            //drives back to center wall
            while(opModeIsActive() && distanceMoved > 0)
            {
                distanceMoved = (robot.frontRight.getCurrentPosition() + robot.frontLeft.getCurrentPosition() + robot.backLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) / 4;
                robot.drive(-.8, 0, 0);
            }

            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            distanceMoved = 0;

            //drive to hub
            while(opModeIsActive() && distanceMoved < 786.25)
            {
                distanceMoved = (robot.frontRight.getCurrentPosition() + robot.backLeft.getCurrentPosition()) / 2;
                robot.drive(0, .6, 0);

                if(distanceMoved < 510)
                    robot.deposit();

                if (distanceMoved > 187)
                    robot.leftRampDown();
                if (distanceMoved > 365.5)
                    robot.depositLeft();
            }

            //drives back to wall
            while(opModeIsActive() && distanceMoved > 10)
            {
                distanceMoved = (robot.frontRight.getCurrentPosition() + robot.backLeft.getCurrentPosition()) / 2;
                robot.drive(0, -.6, 0);
            }

        } //end of subsequent cycles for loop

        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        distanceMoved = 0;

        //drives to warehouse to park
        while(opModeIsActive() && distanceMoved < 2500)
        {
            distanceMoved = (robot.frontRight.getCurrentPosition() + robot.frontLeft.getCurrentPosition() + robot.backLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) / 4;
            robot.drive(.8, 0, 0);
        }
    }
}