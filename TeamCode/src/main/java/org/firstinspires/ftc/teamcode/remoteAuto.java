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

import java.io.File;
import java.util.List;
import java.util.Scanner;


@Autonomous(name = "remoteAuto", group = "Auto")
public class remoteAuto extends LinearOpMode {

    int openCVDetectionLevel;

    @Override
    public void runOpMode() throws InterruptedException
    {
        Hardware.currentOpMode=this;
        Hardware robot = new Hardware(hardwareMap);

        waitForStart();
        //Open CV goes here to spit out 1, 2, or 3


        //moves robot to shipping hub
        while(robot.x < 30) {
            robot.drive(0, 1, 0);
        }
        robot.drive(0,0,0);

        //disployed intake and start brushes
        robot.intakeArmDown();
        robot.setIntakePower(1);

        //move forward to pick up object
        while(robot.y < 6) {
            robot.drive(1, 0, 0);
        }
        robot.drive(0,0,0);

        //lift intake up to get to deposite
        robot.intakeArmUp();

        //move backwards to shipping hub and puting the frieght lined up to deposit
        while(robot.y > 0) {
            robot.drive(-1, 0, 0);
            robot.depositRight();
            robot.deposit();
            robot.deposit();
        }
        robot.drive(0,0,0);


        switch (openCVDetectionLevel) {

            case 1:
               //barcode clostest to storage unit

                break;

            case 2:
                //barcode in the middle of storage unit and shared shipping hub


                break;

            case 3:
                //barcode to thr right of the shipping hub


                break;
        }

    }




}
