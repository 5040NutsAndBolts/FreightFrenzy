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

//testing
import java.io.File;
import java.io.IOException;
//end of test

@Autonomous(name="Auto",group="Auto")
public class Auto extends LinearOpMode
{


    ArrayList<AutoFunction> autoSteps;
    AutoPaths paths;
    boolean red;

    int increment=1;

    boolean backPressed=false;
    boolean saveConfig=false;
    boolean loadConfig=false;
    boolean aPressed=false;
    boolean bPressed=false;
    boolean xPressed=false;
    boolean yPressed=false;
    boolean upPressed=false;
    boolean downPressed=false;
    boolean leftPressed=false;
    boolean rightPressed=false;
    boolean leftJoystickLeft=false;
    boolean leftJoystickRight=false;
    String errors="";



    @Override
    public void runOpMode() throws InterruptedException
    {

        Hardware.currentOpMode=this;
        Hardware robot = new Hardware(hardwareMap);
        //gets data and separates it by line breaks
        int selection=0;
        String configName = "";
        String data="";
        String fileName="default";
        String[] splitData=new String[0];
        try
        {
            data = new Scanner(new File(Environment.getExternalStorageDirectory() +"/"+ fileName)).useDelimiter("\\Z").next();
            splitData=data.split("\n");
        } catch (FileNotFoundException e)
        {
            errors+=e;
        }

        //initialize the list of auto paths
        paths = new AutoPaths(this,robot,false);


        //load, modify, and save configs using controller input during init period
        while(!isStarted())
        {

            //is robot on redSide
            if(gamepad2.x)
                paths.red=false;
            else if(gamepad2.b)
                paths.red=true;
            //makes back button save configs
            if(gamepad1.back)
            {
                saveConfig=true;
                loadConfig=false;
            }
            //makes left bumper not save or load configs
            if(gamepad1.left_bumper)
            {
                saveConfig = false;
                loadConfig=false;
            }

            //makes right bumper load configs
            if(gamepad1.right_bumper)
            {
                saveConfig = false;
                loadConfig=true;
            }

            //saves configs
            if(saveConfig)
            {
                if(gamepad1.a&&!aPressed)
                {
                    aPressed=true;
                    configName+="a";
                }
                else if(!gamepad1.a)
                    aPressed=false;
                if(gamepad1.b&&!bPressed)
                {
                    bPressed=true;
                    configName+="b";
                }
                else if(!gamepad1.b)
                    bPressed=false;
                if(gamepad1.x&&!xPressed)
                {
                    xPressed=true;
                    configName+="x";
                }
                else if(!gamepad1.x)
                    xPressed=false;
                if(gamepad1.y&&!yPressed)
                {
                    yPressed=true;
                    configName+="y";
                }
                else if(!gamepad1.y)
                    yPressed=false;
                //delete characters
                if(gamepad1.dpad_left&&!leftPressed)
                {
                    leftPressed=true;
                    if(configName.length()>=1)
                        configName=configName.substring(0,configName.length()-1);
                }
                else if(!gamepad1.dpad_left)
                    leftPressed=false;
                telemetry.addLine(configName);
                if(gamepad1.left_trigger>.4)
                    saveConfigToFile(configName,data);
            }

            //loads configs
            else if(loadConfig)
            {
                if(gamepad1.a&&!aPressed)
                {
                    aPressed=true;
                    configName+="a";
                }
                else if(!gamepad1.a)
                    aPressed=false;
                if(gamepad1.b&&!bPressed)
                {
                    bPressed=true;
                    configName+="b";
                }
                else if(!gamepad1.b)
                    bPressed=false;
                if(gamepad1.x&&!xPressed)
                {
                    xPressed=true;
                    configName+="x";
                }
                else if(!gamepad1.x)
                    xPressed=false;
                if(gamepad1.y&&!yPressed)
                {
                    yPressed=true;
                    configName+="y";
                }
                else if(!gamepad1.y)
                    yPressed=false;
                telemetry.addLine(configName);
                if(gamepad1.left_trigger>.4)
                {
                    try
                    {
                        data = new Scanner(new File(Environment.getExternalStorageDirectory() +"/"+ configName)).useDelimiter("\\Z").next();
                        splitData=data.split("\n");
                    } catch (FileNotFoundException e)
                    {
                        errors+=e;
                    }
                }

            }
            else
            {
                //changes selected line
                if(gamepad1.dpad_up&&!upPressed)
                {
                    upPressed=true;
                    if(selection>0)
                        selection--;
                }
                else if(!gamepad1.dpad_up)
                    upPressed=false;
                if(gamepad1.dpad_down&&!downPressed)
                {
                    downPressed=true;
                    if(selection<splitData.length)
                        selection++;
                }
                else if(!gamepad1.dpad_down)
                    downPressed=false;


                //changes increment amount
                if(gamepad1.left_stick_x > .5&&!leftJoystickRight)
                {
                    leftJoystickRight = true;
                    if(increment < 2)
                      increment++;
                }
                else if(gamepad1.left_stick_x < .5)
                    leftJoystickRight = false;

                if(gamepad1.left_stick_x < -.5&&!leftJoystickLeft)
                {
                    leftJoystickLeft = true;
                    if(increment > 0)
                      increment--;
                }
                else if(gamepad1.left_stick_x > -.5)
                    leftJoystickLeft = false;

                //cycle between functions and increment or decrement waits
                if(gamepad1.dpad_left&&!leftPressed)
                {
                    leftPressed=true;
                    if(isNumeric(splitData[selection]))
                    {
                        if(increment==0)
                            splitData[selection]=(Double.parseDouble(splitData[selection])-.01)+"";
                        if(increment==1)
                            splitData[selection]=(Double.parseDouble(splitData[selection])-.1)+"";
                        if(increment==2)
                            splitData[selection]=(Double.parseDouble(splitData[selection])-1)+"";
                    }
                    else
                    {
                        for(int i = 1; i<paths.allPaths.size(); i++)
                        {
                            if(splitData[selection].equals(paths.allPaths.get(i).name.substring(0,paths.allPaths.get(i).name.length()-1)))
                            {
                                splitData[selection]=paths.allPaths.get(i-1).name;
                                break;
                            }
                        }
                    }
                    data="";
                    for(String s:splitData)
                    {
                        data+=s+"\n";
                    }
                }
                else if(!gamepad1.dpad_left)
                    leftPressed=false;
                if(gamepad1.dpad_right&&!rightPressed)
                {
                    rightPressed=true;
                    if(isNumeric(splitData[selection]))
                    {
                        if(increment==0)
                            splitData[selection]=(Double.parseDouble(splitData[selection])+.01)+"";
                        if(increment==1)
                            splitData[selection]=(Double.parseDouble(splitData[selection])+.1)+"";
                        if(increment==2)
                            splitData[selection]=(Double.parseDouble(splitData[selection])+1)+"";
                    }
                    else
                    {
                        for(int i = 0; i<paths.allPaths.size()-1; i++)
                        {
                            if(splitData[selection].contains(paths.allPaths.get(i).name))
                            {
                                splitData[selection]=paths.allPaths.get(i+1).name;
                                break;
                            }
                        }
                    }
                    data="";
                    for(String s:splitData)
                    {
                        data+=s+"\n";
                    }
                }
                else if(!gamepad1.dpad_right)
                    rightPressed=false;
                //add auto step to file
                if(gamepad1.a&&!aPressed)
                {
                    aPressed=true;
                    String[] oldData = splitData;
                    splitData=new String[splitData.length+1];
                    for(int i = 0; i<oldData.length; i++)
                    {
                        if(i<selection)
                            splitData[i]=oldData[i];
                        else
                            splitData[i+1]=oldData[i];
                    }
                    splitData[selection]=paths.allPaths.get(0).name;
                    data="";
                    for(String s:splitData)
                    {
                        data+=s+"\n";
                    }
                }
                else if(!gamepad1.a)
                {
                    aPressed=false;
                }

                //deletes a step from auto
                if(gamepad1.b&&!bPressed)
                {
                   bPressed=true;
                   String[] oldData = splitData;
                   splitData=new String[splitData.length-1];
                   for(int i = 0; i<oldData.length-1; i++)
                   {
                     if(i<selection)
                         splitData[i] = oldData[i];
                     else
                         splitData[i] = oldData[i+1];
                   }
                   data="";
                   for(String s:splitData)
                   {
                       data+=s+"+\n";
                   }
                }
                else if(!gamepad1.b)
                {
                    bPressed=false;
                }

                //adds wait
                if(gamepad1.x&&!xPressed)
                {
                    xPressed=true;
                    String[] oldData = splitData;
                    splitData=new String[splitData.length+1];
                    for(int i = 0; i<oldData.length; i++)
                    {
                        if(i<selection)
                            splitData[i]=oldData[i];
                        else
                            splitData[i+1]=oldData[i];
                    }
                    splitData[selection]="1";
                    data="";
                    for(String s:splitData)
                    {
                        data+=s+"+\n";
                    }
                }
                else if(!gamepad1.x)
                {
                    xPressed=false;
                }

                //print auto functions in order
                if(splitData.length < 17 )
                {
                    for(int i = 0; i<splitData.length; i++)
                        telemetry.addLine((i==selection?"*":"")+splitData[i]);
                }
                else
                {
                    for(int i = splitData.length-17; i< splitData.length; i++)
                        telemetry.addLine((i==selection?"*":"")+splitData[i]);
                }
                if(selection==splitData.length)
                    telemetry.addLine("*");
            }

            telemetry.addLine(errors);
            telemetry.addData("red",paths.red);

            telemetry.update();

        }
        waitForStart();

        //turn each string in auto data into a function
        autoSteps = parseData(splitData);

        //run each step of auto in order
        for(AutoFunction step:autoSteps)
        {

            step.step();

        }


    }




    //saves config to a file
    public void saveConfigToFile(String name,String data)
    {

        try

        {

            File file = new File(Environment.getExternalStorageDirectory() +"/"+name);
            if(!file.exists())
                file.createNewFile();
            FileWriter writer = new FileWriter(Environment.getExternalStorageDirectory() +"/"+name);
            writer.write(data);
            writer.close();

        } catch (IOException e)
        {

            errors+=e;

        }

    }

    //converts strings into functions
    public ArrayList<AutoFunction> parseData(String[] data)
    {

        ArrayList<AutoFunction> functions = new ArrayList<>();

        for(String function:data)
        {

            telemetry.addLine(function);
            //checks input string against list of paths
            boolean isPath=false;
            for(AutoFunction funct:paths.allPaths)
            {

                if(function.contains(funct.name))
                {
                    functions.add(funct);
                    isPath=true;
                    break;
                }

            }


            if(!isPath)
            {
                functions.add(new AutoFunction()
                {
                    @Override
                    void step()
                    {
                        ElapsedTime e = new ElapsedTime();
                        while (opModeIsActive() && e.seconds() < Double.parseDouble(function)) ;
                    }
                });

            }

        }

        telemetry.addData("red",red);

        telemetry.update();

        return functions;

    }

    //checks if lines are valid doubles
    private Pattern pattern = Pattern.compile("-?\\d+(\\.\\d+)?");

    public boolean isNumeric(String strNum) {
        if (strNum == null) {
            return false;
        }
        return pattern.matcher(strNum).matches();
    }

}
