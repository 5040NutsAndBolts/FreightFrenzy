package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helperclasses.HelperMethods;
import org.firstinspires.ftc.teamcode.helperclasses.PathFollowers;
import org.firstinspires.ftc.teamcode.helperclasses.Point;

@Autonomous(name="testAuto",group="Auto")
public class testPathAuto extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException {
        Hardware.currentOpMode = this;
        Hardware robot = new Hardware(hardwareMap);
        robot.resetOdometry(0,0,3*Math.PI/2);
        waitForStart();
        while (opModeIsActive()) {
            PathFollowers.linearTolerancePathFollow(robot, -.5, -.5, 3 * Math.PI / 2, 10, .05, 0.2, .15, 3 * Math.PI / 2, new Point(0, 0));
            robot.updatePositionRoadRunner();
        }
    }


}
