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


@Autonomous(name = "BlueDuckAuto", group = "Auto")
public class BlueDuckAuto extends LinearOpMode
{
    public int autoType = 1;
    //about 42.5 ticks per inch
    //x is between blue and red
    //y is along one color wall
    //FL BR strafe towards blue hub

    double distanceMoved = 0;

    int openCVDetectionLevel = 1;

    Hardware robot;

    public void thisSideDuckSpin(double power)
    {
        robot.setRightDuckSpinnerPower(power);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        int auto = 3;

        if (autoType == 1)
        {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Left Webcam"), cameraMonitorViewId);

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

            while (!isStopRequested() & !isStarted()) {

                if (TSEFinder.screenPosition.x < 10)
                    auto = 3;
                else if (TSEFinder.screenPosition.x < 150)
                    auto = 2;
                else
                    auto = 1;

                telemetry.addData("auto", auto);
                telemetry.update();
            }
        }
        else
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

            while (!isStopRequested() & !isStarted()) {

                if (TSEFinder.screenPosition.x < 50)
                    auto = 1;
                else if (TSEFinder.screenPosition.x < 150)
                    auto = 2;
                else
                    auto = 3;

                telemetry.addData("auto", auto);
                telemetry.update();
            }
        }

            Hardware.currentOpMode = this;
            robot = new Hardware(hardwareMap);
            FileWriter f = null;
            robot.resetStaticMotors();
            try {
                f = new FileWriter(Environment.getExternalStorageDirectory() + "/testingData.txt");
            } catch (IOException e) {
                e.printStackTrace();
            }
            robot.depositNeutral();
            //robot.resetOdometry(0, robot.y, 3 * Math.PI / 2);
            int[] colors = new int[3];
            colors[0] = robot.lineColorSensor.alpha();

            waitForStart();
            robot.setVerticalPosition(.61);
            robot.setHorizontalPosition(.44);
            colors[1] = robot.lineColorSensor.alpha();
            double ambientIntakeColor = robot.intakeColorSensor.red();
            ElapsedTime totalAutoTime = new ElapsedTime();
            totalAutoTime.startTime();
            //Open CV goes here to spit out 1, 2, or 3
            //moves robot to shipping hub

            robot.intakeArmUp();
            robot.intakeArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            boolean setMode = true;

            //strafe towards center
            while (opModeIsActive() && distanceMoved < 210) {
                robot.depositLevel = 1;
                robot.deposit();
                robot.intakeArm.setPower(-.8);
                robot.openIntake();
                robot.intakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                distanceMoved = autoType * (robot.frontLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) / 2;
                robot.drive(0, -.5 * autoType, 0);

                telemetry.addData("distance moved", distanceMoved);
                telemetry.addLine("driving sideways");
                telemetry.update();
            }

            AutoMethods.resetEncoders(robot);
            distanceMoved = 0;
            robot.intakeArm.setPower(0);

            //drive to duck wheel
            while (opModeIsActive() && distanceMoved > -930 ) {
                robot.depositLevel = 0;
                robot.deposit();
                robot.intakeArm.setPower(-.8);
                robot.openIntake();

                distanceMoved = (robot.frontRight.getCurrentPosition() + robot.frontLeft.getCurrentPosition() + robot.backLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) / 4;
                robot.drive(-.6, 0, 0);

                telemetry.addData("distance moved", distanceMoved);
                telemetry.addLine("driving forward");
                telemetry.update();
            }

            //spin duck
            while (opModeIsActive() && totalAutoTime.seconds() < 4 + (autoType == -1 ? 1 : 0))
            {
                robot.drive(0, 0, 0);
                thisSideDuckSpin(-.7);
                robot.intakeArm.setPower(-.8);
                robot.openIntake();

                telemetry.addData("distance moved", distanceMoved);
                telemetry.update();
            }

            robot.setRightDuckSpinnerPower(0);
            AutoMethods.resetEncoders(robot);
            distanceMoved = 0;
            robot.intakeArm.setPower(0);

            //park in storage unit
            while (opModeIsActive() && distanceMoved < 1250) {
                distanceMoved = autoType * (robot.frontLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) / 2;
                robot.drive(0, -.5 * autoType, 0);

                telemetry.addData("distance moved", distanceMoved);
                telemetry.update();
            }

            AutoMethods.resetEncoders(robot);
            distanceMoved = 0;

            //drive against wall
            while (opModeIsActive() && totalAutoTime.seconds() < 6.5 + (autoType == -1 ? 1 : 0))
            {
                distanceMoved = autoType * (robot.frontRight.getCurrentPosition() + robot.frontLeft.getCurrentPosition() + robot.backLeft.getCurrentPosition() + robot.backRight.getCurrentPosition()) / 4;
                robot.drive(-.2, 0, 0);

                telemetry.addData("distance moved", distanceMoved);
                telemetry.addData("seconds", totalAutoTime.seconds());
                telemetry.update();
            }

            telemetry.addLine("robot stopped");
            telemetry.update();
            robot.drive(0, 0, 0);
        }
    }


