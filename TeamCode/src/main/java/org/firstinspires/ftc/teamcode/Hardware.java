package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static java.lang.Math.abs;

public class Hardware
{

    //The linear slide motors
    public DcMotorEx depositSlide;

    private CRServo leftDuckSpinner;
    private DcMotor rightDuckSpinner;

    //drive train motors
    public DcMotorEx frontLeft;
    public DcMotorEx backLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backRight;

    public DcMotor intakeArm;
    private DcMotorEx intakeSweeper;
    public RevColorSensorV3 colorsensor;

    //Deposit servo flicker and ramps
    private Servo depositFlicker;
    private Servo rightRamp;
    private Servo leftRamp;
    private Servo intakeBlocker;

    private Servo leftOdomServo;
    private Servo rightOdomServo;
    private Servo centerOdomServo;

    public int depositLevel=0;

    public boolean depositOverride;
    public boolean intakeOverride;

    private Servo capperVertical;
    private Servo capperHorozontal;
    private CRServo capperOut;

    public AnalogInput distanceSensor;
    public AnalogInput sideDistanceSensor;

    public static LinearOpMode currentOpMode;

    private boolean intakeUp = true;

    public RevBulkData bulkData;
    public ExpansionHubEx expansionHub;

    private static DcMotor staticIntake;
    private static DcMotorEx staticDeposit;

    public double x=0;
    public double y=0;
    public double theta=0;

    public RevColorSensorV3 lineColorSensor;

    public ColorRangeSensor intakeColorSensor;

    BNO055IMU imu;


    private static final double ODOM_TICKS_PER_IN = 1898.130719;

    public static double trackwidth = 10.39701829;

    public ExpansionHubMotor leftOdom, rightOdom, centerOdom;
    // Real world distance traveled by the wheels
    public double leftOdomTraveled, rightOdomTraveled, centerOdomTraveled;

    // Odometry encoder positions
    public int leftEncoderPos, centerEncoderPos, rightEncoderPos;

    public ThreeTrackingWheelLocalizer odom = new ThreeTrackingWheelLocalizer(
            new ArrayList<>(Arrays.asList(
                    new Pose2d(5.685874282, 0, Math.PI / 2),
                    new Pose2d(0, trackwidth/2, 0),
                    new Pose2d(0, -trackwidth/2, 0)))) {
        @Override
        public List<Double> getWheelPositions() {
            ArrayList<Double> wheelPositions = new ArrayList<>(3);
            wheelPositions.add(centerOdomTraveled);
            wheelPositions.add(leftOdomTraveled);
            wheelPositions.add(rightOdomTraveled);
            return wheelPositions;
        }
    };

    public void resetStaticMotors()
    {

        staticDeposit=null;
        staticIntake=null;

    }

    public Hardware(HardwareMap hardwareMap)
    {

        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Control Hub");
        leftOdom = (ExpansionHubMotor) hardwareMap.dcMotor.get("Front Left");
        rightOdom = (ExpansionHubMotor) hardwareMap.dcMotor.get("Front Right");
        centerOdom = (ExpansionHubMotor) hardwareMap.dcMotor.get("Back Left");

        //Intake
        intakeSweeper = hardwareMap.get(DcMotorEx.class,"Intake Sweeper");
        if(staticIntake!=null)
            intakeArm=staticIntake;
        {
            intakeArm = hardwareMap.dcMotor.get("Intake Arm");
            staticIntake = intakeArm;
        }
        intakeBlocker = hardwareMap.servo.get("Intake Blocker");
        intakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeArm.setTargetPosition(0);
        intakeArm.setPower(0);
        intakeArm.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //intakeArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Color sensor for intake to sense if something is in the intake
        colorsensor = hardwareMap.get(RevColorSensorV3.class,"Intake Color Sensor");

        //Duck spinners
        leftDuckSpinner = hardwareMap.crservo.get("Left Duck Spinner");
        rightDuckSpinner = hardwareMap.get(DcMotor.class,"Right Duck Spinner");
        rightDuckSpinner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDuckSpinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDuckSpinner.setDirection(DcMotorSimple.Direction.REVERSE);

        //Drive motors
        frontLeft = hardwareMap.get(DcMotorEx.class,"Front Left");
        backLeft = hardwareMap.get(DcMotorEx.class,"Back Left");
        frontRight = hardwareMap.get(DcMotorEx.class,"Front Right");
        backRight = hardwareMap.get(DcMotorEx.class,"Back Right");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Deposit
        if(staticDeposit!=null)
            depositSlide=staticDeposit;
        else
        {
            depositSlide = hardwareMap.get(DcMotorEx.class, "Deposit Slide");
            staticDeposit=depositSlide;
        }
        depositSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        depositSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        depositSlide.setTargetPosition(0);
        depositSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRamp = hardwareMap.get(Servo.class, "Left Ramp");
        rightRamp = hardwareMap.get(Servo.class, "Right Ramp");
        depositFlicker = hardwareMap.get(Servo.class, "Deposit Flicker");

        //capper slides
        capperVertical = hardwareMap.servo.get("Capper Vertical");
        capperHorozontal = hardwareMap.servo.get("Capper Horizontal");
        capperOut = hardwareMap.crservo.get("Capper Out");

        leftOdomServo=hardwareMap.servo.get("Left Odom Servo");
        rightOdomServo=hardwareMap.servo.get("Right Odom Servo");
        centerOdomServo=hardwareMap.servo.get("Center Odom Servo");

        distanceSensor = hardwareMap.get(AnalogInput.class, "Distance Sensor");
        sideDistanceSensor = hardwareMap.get(AnalogInput.class, "Side Distance Sensor");

        lineColorSensor = hardwareMap.get(RevColorSensorV3.class,"Line Sensor");

        intakeColorSensor = hardwareMap.get(ColorRangeSensor.class, "Intake Color Sensor");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        currentOpMode.telemetry.addLine("imu init");
        while(!imu.isGyroCalibrated()&&!currentOpMode.isStopRequested());


    }

    /*public void odomDown()
    {

        leftOdomServo.setPosition(.492);
        rightOdomServo.setPosition(.492);
        centerOdomServo.setPosition(.492);

    }
    public void odomUp()
    {

        leftOdomServo.setPosition(0);
        rightOdomServo.setPosition(0);
        centerOdomServo.setPosition(0);

    }*/

    //Deposit ramp positions
    public void leftRampUp(){leftRamp.setPosition(.1);}
    public void leftRampDown(){leftRamp.setPosition(.47);}
    public void rightRampUp(){rightRamp.setPosition(.97);}
    public void rightRampDown(){rightRamp.setPosition(.39);}

    //Deposit flicker positions
    public void depositLeft(){depositFlicker.setPosition(0);}
    public void depositRight(){depositFlicker.setPosition(.95);}
    public void depositHalfRight(){depositFlicker.setPosition(.6);}
    public void depositNeutral(){depositFlicker.setPosition(.6);}
    public int depositPosition(){return depositSlide.getCurrentPosition();}


    public void deposit()
    {


        if(!depositOverride)
        {

            if ((depositLevel == 0)) {
                if (depositSlide.getCurrentPosition() < 35) {
                    depositSlide.setPower(0);
                    if(depositSlide.getZeroPowerBehavior()!=DcMotor.ZeroPowerBehavior.FLOAT)
                        depositSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                } else {
                    depositSlide.setTargetPosition(0);
                    depositSlide.setPower(1);
                }

            } else if (depositLevel == 1)
            {
                if (depositSlide.getCurrentPosition() < 505||depositSlide.getCurrentPosition() > 509)
                {
                    depositSlide.setPower(1);
                    depositSlide.setTargetPosition(507);
                }
                else
                {
                    depositSlide.setPower(0);
                    if(depositSlide.getZeroPowerBehavior()!=DcMotor.ZeroPowerBehavior.BRAKE)
                        depositSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }

            } else
            {

                if (depositSlide.getCurrentPosition() > 1345)
                {
                    depositSlide.setPower(0);
                    if(depositSlide.getZeroPowerBehavior()!=DcMotor.ZeroPowerBehavior.BRAKE)
                        depositSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                } else
                {
                    depositSlide.setTargetPosition(1400);
                    depositSlide.setPower(1);
                }
            }
        }

    }



    //Run ducks spinners
    public void setLeftDuckSpinnerPower(double power){leftDuckSpinner.setPower(power);}
    public void setRightDuckSpinnerPower(double power){rightDuckSpinner.setPower(power/2.8);}

    //Intake methods
    public void setIntakePower(double power){intakeSweeper.setPower(power);}
    public void intakeArmUp() { intakeUp=true; }
    public void intakeArmDown() { intakeUp=false; }

    //capper methods
    public void setHorizontalPosition(double pos){capperHorozontal.setPosition(pos);}
    public void setVerticalPosition(double pos){capperVertical.setPosition(pos);}
    public void setOutPower(double power){capperOut.setPower(power);}


    //lower level = 180, middle level = 1300
    boolean resetEncoder = true;

    public void intake()
    {


        if(!intakeOverride)
        {
            if ((intakeUp))
            {

                if (colorsensor.getDistance(DistanceUnit.INCH)<1.5&&!resetEncoder)
                {
                    intakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    intakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    resetEncoder = true;

                } else if(resetEncoder)
                {
                    intakeArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    intakeArm.setPower(0);
                }
                else
                {
                    intakeArm.setTargetPosition(0);
                    intakeArm.setPower(1);
                }

            } else
            {
                resetEncoder = false;
                if (intakeArm.getCurrentPosition() > 103)
                {
                    intakeArm.setPower(0);
                    if(intakeArm.getZeroPowerBehavior()!=DcMotor.ZeroPowerBehavior.BRAKE)
                        intakeArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                } else
                {
                    intakeArm.setPower(0);
                    if(intakeArm.getZeroPowerBehavior()!=DcMotor.ZeroPowerBehavior.FLOAT)
                        intakeArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                }

            }
        }

    }

    public double forwardsTicks()
    {

        return ((double)(frontLeft.getCurrentPosition()+frontRight.getCurrentPosition()+backLeft.getCurrentPosition()+backRight.getCurrentPosition()))/4.0;

    }

    public double forwardsTicksStrafePos()
    {
        return ((double)(frontRight.getCurrentPosition()+backLeft.getCurrentPosition()))/2.0;
    }


    public void updatePositionRoadRunner() {
        try
        {
            bulkData = expansionHub.getBulkInputData();
        }catch(Exception e)
        {

            return;

        }

        // Change in the distance (centimeters) since the last update for each odometer
        double deltaLeftDist = -(getDeltaLeftTicks()/ ODOM_TICKS_PER_IN );
        double deltaRightDist = -(getDeltaRightTicks()/ ODOM_TICKS_PER_IN );
        double deltaCenterDist = getDeltaCenterTicks()/ ODOM_TICKS_PER_IN;

        leftOdomTraveled += deltaLeftDist;
        rightOdomTraveled += deltaRightDist;
        centerOdomTraveled += deltaCenterDist;

        odom.update();
        theta = odom.getPoseEstimate().component3();
        x = -odom.getPoseEstimate().component1();
        y = -odom.getPoseEstimate().component2();



    }

    public void updatePositionRoadRunnerRightAndCenter() {
        try
        {
            bulkData = expansionHub.getBulkInputData();
        }catch(Exception e)
        {

            return;

        }

        // Change in the distance (centimeters) since the last update for each odometer
        getDeltaLeftTicks();
        double deltaRightDist = -(getDeltaRightTicks()/ ODOM_TICKS_PER_IN );
        double deltaCenterDist = getDeltaCenterTicks()/ ODOM_TICKS_PER_IN;

        leftOdomTraveled += deltaRightDist;
        rightOdomTraveled += deltaRightDist;
        centerOdomTraveled += deltaCenterDist;

        odom.update();

        x = -odom.getPoseEstimate().component1();
        y = -odom.getPoseEstimate().component2();




    }
    public void updatePositionRoadRunnerOnlyRight() {
        try
        {
            bulkData = expansionHub.getBulkInputData();
        }catch(Exception e)
        {

            return;

        }

        // Change in the distance (centimeters) since the last update for each odometer
        getDeltaLeftTicks();
        double deltaRightDist = -(getDeltaRightTicks()/ ODOM_TICKS_PER_IN );
        getDeltaCenterTicks();

        leftOdomTraveled += deltaRightDist;
        rightOdomTraveled += deltaRightDist;

        odom.update();
        theta = odom.getPoseEstimate().component3();
        x = -odom.getPoseEstimate().component1();
        y = -odom.getPoseEstimate().component2();




    }

    /**
     * Resets the delta on all odometry encoders back to 0
     * */
    public void resetDeltaTicks() {
        leftEncoderPos = bulkData.getMotorCurrentPosition(leftOdom);
        rightEncoderPos = bulkData.getMotorCurrentPosition(rightOdom);
        centerEncoderPos = bulkData.getMotorCurrentPosition(centerOdom);
    }

    private int getDeltaLeftTicks() {
        try
        {
            int total=bulkData.getMotorCurrentPosition(leftOdom);
            int oldPos = leftEncoderPos;
            leftEncoderPos=total;
            return oldPos - total;
        }catch(Exception e)
        {

            return 0;

        }
    }

    private int getDeltaRightTicks() {
        try
        {
            int total=bulkData.getMotorCurrentPosition(rightOdom);
            int oldPos = rightEncoderPos;
            rightEncoderPos=total;
            return oldPos - total;
        }catch(Exception e)
        {

            return 0;

        }
    }

    private int getDeltaCenterTicks() {
        try
        {
            int total=bulkData.getMotorCurrentPosition(centerOdom);
            int oldPos = centerEncoderPos;
            centerEncoderPos=total;
            return oldPos - total;
        }catch(Exception e)
        {

            return 0;

        }
    }

    public void softBrake()
    {
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /**
     * Resets odometry position and values back to specific values
     *
     * @param x     X position to reset encoders to
     * @param y     Y position to reset encoders to
     * @param theta Rotational value to reset encoders to
     */
    public void resetOdometry(double x, double y, double theta) {
        odom.setPoseEstimate(new Pose2d(-x, -y, theta));

        leftOdomTraveled = 0;
        rightOdomTraveled = 0;
        leftOdomTraveled = 0;
        leftEncoderPos = 0;

        rightEncoderPos = 0;
        centerEncoderPos = 0;

        // Resets encoder values then sets them back to run without encoders because wheels and odometry are same pointer
        leftOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        centerOdom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerOdom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public int intakeArmPosition() {return intakeArm.getCurrentPosition();}
    public void resetIntakeArmPosition(){intakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}
    public void intakeStart() {intakeBlocker.setPosition(0.1);}
    public void openIntake() {intakeBlocker.setPosition(.74);}
    //public void intakeHalfWay(){intakeBlocker.setPosition(.31);}
    public void closeIntake(){intakeBlocker.setPosition(0.532);}
    public void reallyCloseIntake(){intakeBlocker.setPosition(0.75);}

    //Set drive power
    public void drive(double forward, double sideways, double rotation) {

        //adds all the inputs together to get the number to scale it by
        double scale = abs(rotation) + abs(forward) + abs(sideways);

        //scales the inputs when needed
        if (scale > 1) {
            forward /= scale;
            rotation /= scale;
            sideways /= scale;
        }
        //setting the motor powers to move
        frontLeft.setPower(forward - rotation - sideways);
        backLeft.setPower(forward - rotation + sideways);
        frontRight.setPower(forward + rotation + sideways);
        backRight.setPower(forward + rotation - sideways);
    }

    public double intakeSeeperDraw()
    {

        return intakeSweeper.getCurrent(CurrentUnit.AMPS);

    }

    //public boolean intakeHasFreight()
    //{

        //return intakeSweeper.getCurrent(CurrentUnit.AMPS)
    //}

}
