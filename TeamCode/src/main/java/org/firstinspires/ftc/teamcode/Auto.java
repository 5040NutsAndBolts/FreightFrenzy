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

@Autonomous(name="Auto",group="Auto")
public class Auto extends LinearOpMode
{


    ArrayList<AutoFunction> autoSteps;
    AutoPaths paths;

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



    @Override
    public void runOpMode() throws InterruptedException
    {

        int selection=0;
        String configName = "";
        String data="";
        String fileName="default";
        String errors="";
        String[] splitData=new String[0];
        try
        {
            data = new Scanner(new File(Environment.getExternalStorageDirectory() +"/"+ fileName)).useDelimiter("\\Z").next();
            splitData=data.split("\n");
        } catch (FileNotFoundException e)
        {
            errors+=e;
        }
        paths = new AutoPaths(this);
        while(!isStarted())
        {

            if(gamepad1.back)
            {
                saveConfig=true;
                loadConfig=false;
            }
            if(gamepad1.left_bumper)
            {
                saveConfig = false;
                loadConfig=false;
            }
            if(gamepad1.right_bumper)
            {
                saveConfig = false;
                loadConfig=true;
            }

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
                if(gamepad1.left_bumper)
                {
                    try
                    {
                        data = new Scanner(new File(Environment.getExternalStorageDirectory() +"/"+ fileName)).useDelimiter("\\Z").next();
                        splitData=data.split("\n");
                    } catch (FileNotFoundException e)
                    {
                        errors+=e;
                    }
                }
            }
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
                if(gamepad1.dpad_left)
                    saveConfigToFile(configName,data);
            }
            else
            {

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

                if(gamepad1.dpad_left&&!leftPressed)
                {
                    leftPressed=true;
                    if(isNumeric(splitData[selection]))
                    {
                        splitData[selection]=(Double.parseDouble(splitData[selection])-.1)+"";
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
                    data="";
                    for(String s:splitData)
                    {
                        data+=s+"+\n";
                    }
                }
                else if(!gamepad1.dpad_right)
                    rightPressed=false;

                for(int i = 0; i<splitData.length; i++)
                    telemetry.addLine((i==selection?"*":"")+splitData[i]);

            }

            telemetry.addLine(errors);

            telemetry.update();

        }
        waitForStart();

        autoSteps = parseData(splitData);

        for(AutoFunction step:autoSteps)
        {

            step.step();

        }


    }

    public void saveConfigToFile(String name,String data)
    {

        try
        {
            new FileWriter(name).write(data);
        } catch (IOException e)
        {
            e.printStackTrace();
        }

    }

    public ArrayList<AutoFunction> parseData(String[] data)
    {

        ArrayList<AutoFunction> functions = new ArrayList<>();

        for(String functionS:data)
        {

            switch(functionS)
            {

                case "park":
                    functions.add(paths.park);
                    break;
                default:
                    functions.add(()->
                    {
                        ElapsedTime e = new ElapsedTime();
                        while (opModeIsActive()&&e.seconds() < Double.parseDouble(functionS));
                    });

            }

        }

        return functions;

    }

    private Pattern pattern = Pattern.compile("-?\\d+(\\.\\d+)?");

    public boolean isNumeric(String strNum) {
        if (strNum == null) {
            return false;
        }
        return pattern.matcher(strNum).matches();
    }

}
