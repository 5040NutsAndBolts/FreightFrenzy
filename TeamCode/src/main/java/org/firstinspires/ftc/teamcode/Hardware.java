package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Math.abs;

public class Hardware
{

    //The linear slide motors
    public DcMotor depositSlide;

    //drive train motors
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;

    DcMotor intake;

    //Deposit servo flicker and ramps
    private Servo depositFlicker;
    private Servo rightRamp;
    private Servo leftRamp;

    public Hardware(HardwareMap hardwareMap)
    {

        //drive motors
        frontLeft = hardwareMap.dcMotor.get("Front Left");
        backLeft = hardwareMap.dcMotor.get("Back Left");
        frontRight = hardwareMap.dcMotor.get("Front Right");
        backRight = hardwareMap.dcMotor.get("Back Right");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotor.class, "Intake");
        depositSlide = hardwareMap.get(DcMotor.class, "Deposit Slide");
        leftRamp = hardwareMap.get(Servo.class, "Left Ramp");
        rightRamp = hardwareMap.get(Servo.class, "Right Ramp");
        depositFlicker = hardwareMap.get(Servo.class, "Deposite Flicker");

    }

    public void leftRampUp(){leftRamp.setPosition(0);}
    public void leftRampDown(){leftRamp.setPosition(1);}
    public void rightRampUp(){rightRamp.setPosition(0);}
    public void rightRampDown(){rightRamp.setPosition(1);}

    public void depositLeft(){depositFlicker.setPosition(0);}
    public void depositRight(){depositFlicker.setPosition(1);}
    public void depositNeutral(){depositFlicker.setPosition(.5);}

    public void setIntakePower(double power){intake.setPower(power);}

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
        //Left Front = +Speed + Turn - Strafe      Right Front = +Speed - Turn + Strafe
        //Left Rear  = +Speed + Turn + Strafe      Right Rear  = +Speed - Turn - Strafe
    }

}
