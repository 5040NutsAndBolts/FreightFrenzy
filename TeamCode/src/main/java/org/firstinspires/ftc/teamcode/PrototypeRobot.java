package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="",group="Teleop")
public class PrototypeRobot extends LinearOpMode
{

    //The linear slide motors
    DcMotor rightDepositSlide;
    DcMotor leftDepositSlide;

    //Deposit servo flicker and ramps
    Servo depositFlicker;
    Servo rightRamp;
    Servo leftRamp;
    boolean despositeFlickerToggle = false;
    boolean leftRampToggle = false;
    boolean rightRampToggle = false;

    @Override
    public void runOpMode() throws InterruptedException
    {
        waitForStart();


        //Naming proper name used on driver hub
        leftDepositSlide = hardwareMap.get(DcMotor.class, "Left Slide");
        rightDepositSlide = hardwareMap.get(DcMotor.class, "Right Slide");
        leftRamp = hardwareMap.get(Servo.class, "Left Ramp");
        rightRamp = hardwareMap.get(Servo.class, "Right Ramp");
        depositFlicker = hardwareMap.get(Servo.class, "Deposite Flicker");

        //setting position for the ramp servo start
        leftRamp.setPosition(0);
        rightRamp.setPosition(0);
        depositFlicker.setPosition(1);

        //Button Pushing to get deposit ramps to open and side picking



        while(opModeIsActive()) {

            //Slide outtake motor controller set up (linear slides)
            leftDepositSlide.setPower(gamepad2.left_stick_y);
            rightDepositSlide.setPower(gamepad2.left_stick_y);


            //Flicker goes one way or other way depends on stuff I didn't add

            if (!despositeFlickerToggle) {
                if (gamepad2.x) {
                    despositeFlickerToggle = true;

                    depositFlicker.setPosition(0);
                }
            }
            if (despositeFlickerToggle) {
                if (gamepad2.x) { //jdfifuj\judsguidfidfd
                    despositeFlickerToggle = false;

                    depositFlicker.setPosition(2);
                }
            }
                telemetry.addData("Direction State:", despositeFlickerToggle);


                    //Button Pushing to get deposit ramps to open
                    //right ramp moving open and closed

                    if (!rightRampToggle) {
                        if (gamepad2.a) {
                            rightRampToggle = true;

                            rightRamp.setPosition(1);
                        }
                    }
                    if (rightRampToggle) {
                        if (gamepad2.a) {
                            rightRampToggle = false;

                            rightRamp.setPosition(0);
                        }
                    }
                    telemetry.addData("Direction State:", rightRampToggle);


                    //left ramp moving open and closed

                    if (!leftRampToggle) {
                        if (gamepad2.b) {
                            leftRampToggle = true;

                            leftRamp.setPosition(1);
                        }
                    }
                    if (leftRampToggle) {
                        if (gamepad2.b) {
                            leftRampToggle = false;

                            leftRamp.setPosition(0);
                        }
                    }
                    telemetry.addData("Direction State:", leftRampToggle);

                }

            }
        }