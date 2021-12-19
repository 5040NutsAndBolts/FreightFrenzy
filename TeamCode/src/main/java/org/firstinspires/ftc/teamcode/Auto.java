package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helperclasses.HelperMethods;

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
            if(gamepad1.left_trigger>.4)
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
                    configName+="a";
                }
                else if(!gamepad1.y)
                    yPressed=false;
                telemetry.addLine(configName);
                if(gamepad1.right_bumper)
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
                    configName+="a";
                }
                else if(!gamepad1.y)
                    yPressed=false;
                telemetry.addLine(configName);
                if(gamepad1.right_bumper)
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
                    if(selection<splitData.length-1)
                        selection++;
                }
                else if(!gamepad1.dpad_down)
                    downPressed=false;

                //cycle between functions and increment or decrement waits
                if(gamepad1.dpad_left&&!leftPressed)
                {
                    leftPressed=true;
                    if(isNumeric(splitData[selection]))
                    {
                        splitData[selection]=(Double.parseDouble(splitData[selection])-.1)+"";
                    }
                    else
                    {
                        for(int i = 1; i<paths.allPaths.size(); i++)
                        {
                            if(splitData[selection].contains(paths.allPaths.get(i).name))
                            {
                                splitData[selection]=paths.allPaths.get(i-1).name;
                                break;
                            }
                        }
                    }
                    data="";
                    for(String s:splitData)
                    {
                        data+=s+"+\n";
                    }
                }
                else if(!gamepad1.dpad_left)
                    leftPressed=false;
                if(gamepad1.dpad_right&&!rightPressed)
                {
                    rightPressed=true;
                    if(isNumeric(splitData[selection]))
                    {
                        splitData[selection]=(Double.parseDouble(splitData[selection])+.1)+"";
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
                        data+=s+"+\n";
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
                        data+=s+"+\n";
                    }
                }
                else if(!gamepad1.a)
                {
                    aPressed=false;
                }
                if(gamepad1.b&&!bPressed)
                {
                    bPressed=true;
                }
                else if(!gamepad1.b)
                {
                    bPressed=false;
                }

                //print auto functions in order
                for(int i = 0; i<splitData.length; i++)
                    telemetry.addLine((i==selection?"*":"")+splitData[i]);

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
