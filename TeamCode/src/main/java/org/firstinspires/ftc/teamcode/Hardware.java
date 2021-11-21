package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Math.abs;

public class Hardware
{

    //The linear slide motors
    public DcMotor depositSlide;

    private CRServo leftDuckSpinner;
    private CRServo rightDuckSpinner;

    //drive train motors
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;

    private CRServo intakeSweeper;

    //Deposit servo flicker and ramps
    private Servo depositFlicker;
    private Servo rightRamp;
    private Servo leftRamp;

    public Hardware(HardwareMap hardwareMap)
    {

        //Intake
        intakeSweeper = hardwareMap.crservo.get("Intake Sweeper");

        //Duck spinners
        leftDuckSpinner = hardwareMap.crservo.get("Left Duck Spinner");
        rightDuckSpinner = hardwareMap.crservo.get("Right Duck Spinner");

        //Drive motors
        frontLeft = hardwareMap.dcMotor.get("Front Left");
        backLeft = hardwareMap.dcMotor.get("Back Left");
        frontRight = hardwareMap.dcMotor.get("Front Right");
        backRight = hardwareMap.dcMotor.get("Back Right");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //Deposit
        depositSlide = hardwareMap.get(DcMotor.class, "Deposit Slide");
        leftRamp = hardwareMap.get(Servo.class, "Left Ramp");
        rightRamp = hardwareMap.get(Servo.class, "Right Ramp");
        depositFlicker = hardwareMap.get(Servo.class, "Deposite Flicker");

    }

    //Deposit ramp positions
    public void leftRampUp(){leftRamp.setPosition(0);}
    public void leftRampDown(){leftRamp.setPosition(1);}
    public void rightRampUp(){rightRamp.setPosition(0);}
    public void rightRampDown(){rightRamp.setPosition(1);}

    //Deposit flicker positions
    public void depositLeft(){depositFlicker.setPosition(0);}
    public void depositRight(){depositFlicker.setPosition(1);}
    public void depositNeutral(){depositFlicker.setPosition(.5);}

    //Run ducks spinners
    public void setLeftDuckSpinnerPower(double power){leftDuckSpinner.setPower(power);}
    public void setRightDuckSpinnerPower(double power){rightDuckSpinner.setPower(power);}

    //Intake methods
    public void setIntakePower(double power){intakeSweeper.setPower(power);}

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
        frontLeft.setPower(forward + rotation - sideways);
        backLeft.setPower(forward + rotation + sideways);
        frontRight.setPower(forward - rotation + sideways);
        backRight.setPower(forward - rotation - sideways);
    }

}
