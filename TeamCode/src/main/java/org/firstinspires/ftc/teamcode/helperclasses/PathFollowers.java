package org.firstinspires.ftc.teamcode.helperclasses;

import org.firstinspires.ftc.teamcode.Hardware;

public class PathFollowers
{

    public static void linearTolerancePathFollow(Hardware robot, double forwards, double sideways, double angle, double perpendicularTolerance, double angleTolerance, double linearProportionalSpeed, double rotationalProportionalSpeed, Point start)
    {

        //angle difference
        double diff = angle-robot.theta;
        if(diff<-Math.PI)
            diff=Math.PI*2+diff;
        if(diff>Math.PI)
            diff=Math.PI*2-diff;

        //turn robot oriented drive into field oriented
        double fieldForwards=forwards*Math.cos(angle)+sideways*Math.sin(angle);
        double fieldSideways=forwards*Math.sin(angle)+sideways*Math.cos(angle);
        double realRotation=0;

        //calculate distance to drive line
        double lineConstant=-(fieldForwards* start.y+fieldSideways*start.x);
        double distanceToLine=(robot.x*fieldSideways+ robot.y*fieldForwards+lineConstant)/Math.sqrt(Math.pow(fieldForwards,2)+Math.pow(fieldSideways,2));


        //adjust drive is distance to line is out of tolerance
        if(Math.abs(distanceToLine)>perpendicularTolerance)
        {
            double normalizedVector = distanceToLine/Math.sqrt(Math.pow(forwards, 2) + Math.pow(sideways, 2));
            double sidewaysToLineRobotOriented=forwards*normalizedVector;
            double forwardsToLineRobotOriented=sideways*normalizedVector;
            fieldForwards+=linearProportionalSpeed*(-sidewaysToLineRobotOriented*Math.sin(robot.theta)+forwardsToLineRobotOriented*Math.cos(robot.theta));
            fieldSideways+=linearProportionalSpeed*(-sidewaysToLineRobotOriented*Math.cos(robot.theta)+forwardsToLineRobotOriented*Math.sin(robot.theta));
        }

        double realForward;
        double realSideways;

        //if angle is out of tolerance drive along adjust path to regain angle
        if(!HelperMethods.nearAngle(robot.theta,angle,angleTolerance))
        {

            realRotation = diff*rotationalProportionalSpeed;
            realForward=fieldForwards*Math.cos(robot.theta)+fieldSideways*Math.sin(robot.theta);
            realSideways=(fieldForwards*Math.sin(robot.theta)+fieldSideways*Math.cos(robot.theta));
        }
        else
        {
            realForward=fieldForwards*Math.cos(angle)+fieldSideways*Math.sin(angle);
            realSideways=(fieldForwards*Math.sin(angle)+fieldSideways*Math.cos(angle));
        }

        robot.drive(realForward,realSideways,realRotation);

    }

    public static void curveIntakeTowardsPoint(Hardware robot, double proportionalRotateSpeed, double forwardSpeed, double angleToFace, Point point)
    {

        double angleToPoint=robot.theta-Math.atan2(point.y-robot.y,point.x-robot.x)-angleToFace;
        double sidewaysSpeed=0.808736084*proportionalRotateSpeed*angleToPoint;
        double rotationSpeed=-0.588171697675*proportionalRotateSpeed*angleToPoint;
        robot.drive(forwardSpeed,sidewaysSpeed,rotationSpeed);
        Hardware.currentOpMode.telemetry.addData("forward",forwardSpeed);
        Hardware.currentOpMode.telemetry.addData("sideways",sidewaysSpeed);
        Hardware.currentOpMode.telemetry.addData("rotate",rotationSpeed);
        Hardware.currentOpMode.telemetry.update();

    }

    public static void curveToFacePoint(Hardware robot, double proportionalRotateSpeed, double forwardSpeed, double angleToFace, Point point)
    {

        double angleToPoint=robot.theta+Math.atan2(point.x-robot.x,point.y-robot.y)-angleToFace;
        //angle difference
        while(angleToPoint<-Math.PI)
            angleToPoint=Math.PI*2+angleToPoint;
        while(angleToPoint>Math.PI)
            angleToPoint=Math.PI*2-angleToPoint;
        double rotationSpeed=proportionalRotateSpeed*angleToPoint;
        robot.drive(forwardSpeed,0,rotationSpeed);
        Hardware.currentOpMode.telemetry.addData("atan",Math.atan2(point.y-robot.y,point.x-robot.x));
        Hardware.currentOpMode.telemetry.addData("angle",angleToPoint);
        Hardware.currentOpMode.telemetry.addData("forward",forwardSpeed);
        Hardware.currentOpMode.telemetry.addData("rotate",rotationSpeed);
        Hardware.currentOpMode.telemetry.update();

    }


}
