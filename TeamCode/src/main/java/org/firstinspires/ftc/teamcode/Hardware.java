package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;


import static java.lang.Math.abs;

public class Hardware
{

    //The linear slide motors
    public DcMotor depositSlide;

    private CRServo leftDuckSpinner;
    private CRServo rightDuckSpinner;

    //drive train motors
    private DcMotorEx frontLeft;
    private DcMotorEx backLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backRight;

    public DcMotor intakeArm;
    private CRServo intakeSweeper;
    public RevColorSensorV3 colorsensor;

    //Deposit servo flicker and ramps
    private Servo depositFlicker;
    private Servo rightRamp;
    private Servo leftRamp;
    private Servo intakeBlocker;

    public byte depositLevel=0;

    public boolean depositOverride;
    public boolean intakeOverride;

    public static LinearOpMode currentOpMode;

    private boolean intakeUp = true;

    public RevBulkData bulkData;
    public ExpansionHubEx expansionHub;

    private int frontLeftTicks;
    private int backLeftTicks;
    private int frontRightTicks;
    private int backRightTicks;

    public double x=0;
    public double y=0;
    public double theta=0;

    BNO055IMU imu;

    private static final double inchesPerTick=11.0/1440.0;

    public Hardware(HardwareMap hardwareMap)
    {

        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Control Hub");

        //Intake
        intakeSweeper = hardwareMap.crservo.get("Intake Sweeper");
        intakeArm = hardwareMap.dcMotor.get("Intake Arm");
        intakeBlocker = hardwareMap.servo.get("Intake Blocker");
        intakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeArm.setTargetPosition(0);
        intakeArm.setPower(1);
        intakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //intakeArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Color sensor for intake to sense if something is in the intake
        colorsensor = hardwareMap.get(RevColorSensorV3.class,"Intake Color Sensor");

        //Duck spinners
        leftDuckSpinner = hardwareMap.crservo.get("Left Duck Spinner");
        rightDuckSpinner = hardwareMap.crservo.get("Right Duck Spinner");

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
        depositSlide = hardwareMap.get(DcMotor.class, "Deposit Slide");
        depositSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        depositSlide.setTargetPosition(0);
        depositSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRamp = hardwareMap.get(Servo.class, "Left Ramp");
        rightRamp = hardwareMap.get(Servo.class, "Right Ramp");
        depositFlicker = hardwareMap.get(Servo.class, "Deposit Flicker");

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

    //Deposit ramp positions
    public void leftRampUp(){leftRamp.setPosition(.8);}
    public void leftRampDown(){leftRamp.setPosition(.585);}
    public void rightRampUp(){rightRamp.setPosition(.48);}
    public void rightRampDown(){rightRamp.setPosition(.7);}

    //Deposit flicker positions
    public void depositLeft(){depositFlicker.setPosition(.25);}
    public void depositRight(){depositFlicker.setPosition(.75);}
    public void depositNeutral(){depositFlicker.setPosition(.5);}
    public int depositPosition(){return depositSlide.getCurrentPosition();}




    public void deposit()
    {


          if(!depositOverride)
          {

            if ((depositLevel == 0)) {
                if (depositSlide.getCurrentPosition() < 190) {
                    depositSlide.setPower(0);
                    if(depositSlide.getZeroPowerBehavior()!=DcMotor.ZeroPowerBehavior.BRAKE)
                        depositSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                } else {
                    depositSlide.setTargetPosition(180);
                    depositSlide.setPower(1);
                }

            } else if (depositLevel == 1)
            {
                if (depositSlide.getCurrentPosition() < 1330||depositSlide.getCurrentPosition() > 1350)
                {
                    depositSlide.setPower(1);
                    depositSlide.setTargetPosition(1340);
                    depositSlide.setTargetPosition(1340);
                }
                else
                    {
                    depositSlide.setPower(0);
                        if(depositSlide.getZeroPowerBehavior()!=DcMotor.ZeroPowerBehavior.BRAKE)
                            depositSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }

            } else
            {

                if (depositSlide.getCurrentPosition() > 2980)
                {
                    depositSlide.setPower(0);
                    if(depositSlide.getZeroPowerBehavior()!=DcMotor.ZeroPowerBehavior.BRAKE)
                        depositSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                } else
                {
                    depositSlide.setTargetPosition(2990);
                    depositSlide.setPower(1);
                }
            }
          }

        }



    //Run ducks spinners
    public void setLeftDuckSpinnerPower(double power){leftDuckSpinner.setPower(power);}
    public void setRightDuckSpinnerPower(double power){rightDuckSpinner.setPower(power);}

    //Intake methods
    public void setIntakePower(double power){intakeSweeper.setPower(power);}
    public void intakeArmUp() { intakeUp=true; }
    public void intakeArmDown() { intakeUp=false; }

    //lower level = 180, middle level = 1300
    boolean blue = true;

    public void intake()
    {


        if(!intakeOverride)
        {
            if ((intakeUp))
            {

                if (colorsensor.blue() > 800 || colorsensor.getDistance(DistanceUnit.INCH) < 1 && !blue)
                {
                    intakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    intakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    blue = true;
                    intakeArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    intakeArm.setPower(0);
                } else
                {
                    intakeArm.setTargetPosition(-20);
                    intakeArm.setPower(1);
                }

            } else
            {
                blue = false;
                if (intakeArm.getCurrentPosition() > 160)
                {
                    intakeArm.setPower(0);
                    if(intakeArm.getZeroPowerBehavior()!=DcMotor.ZeroPowerBehavior.BRAKE)
                        intakeArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                } else
                {
                    intakeArm.setTargetPosition(165);
                    intakeArm.setPower(1);
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

    public void updateInchesMoved()
    {

        try
        {
            bulkData = expansionHub.getBulkInputData();
        }catch(Exception e)
        {

            return;

        }

        double inchesLF = inchesPerTick*(double)getDeltaFrontLeftTicks();
        double inchesLB = inchesPerTick*(double)getDeltaBackLeftTicks();
        double inchesRF = inchesPerTick*(double)getDeltaFrontRightTicks();
        double inchesRB = inchesPerTick*(double)getDeltaBackRightTicks();

        currentOpMode.telemetry.addData("delta",inchesLF);

        double inchesForward = (inchesLF + inchesLB + inchesRF + inchesRB) / 4.0;
        double inchesSideways = (-inchesLF + inchesLB + inchesRF - inchesRB) / 4.0;

        theta=imu.getAngularOrientation().firstAngle;
        y+=inchesForward*Math.cos(theta)+inchesSideways*Math.sin(theta);
        x+=inchesForward*Math.sin(theta)+inchesSideways*Math.cos(theta);

        resetDeltaTicks();

    }

    public int intakeArmPosition() {return intakeArm.getCurrentPosition();}
    public void resetIntakeArmPosition(){intakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}
    public void openIntake() {intakeBlocker.setPosition(.27);}
    public void closeIntake(){intakeBlocker.setPosition(.45);}

    public int getDeltaFrontLeftTicks(){return frontLeftTicks-bulkData.getMotorCurrentPosition(frontLeft);}
    public int getDeltaFrontRightTicks(){return frontRightTicks-bulkData.getMotorCurrentPosition(frontRight);}
    public int getDeltaBackLeftTicks(){return backLeftTicks-bulkData.getMotorCurrentPosition(backLeft);}
    public int getDeltaBackRightTicks(){return backRightTicks-bulkData.getMotorCurrentPosition(backRight);}

    public void resetDeltaTicks()
    {
        frontLeftTicks=bulkData.getMotorCurrentPosition(frontLeft);
        frontRightTicks=bulkData.getMotorCurrentPosition(frontRight);
        backLeftTicks=bulkData.getMotorCurrentPosition(backLeft);
        backRightTicks=bulkData.getMotorCurrentPosition(backRight);
    }

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

}
