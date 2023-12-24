package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;


public class Proximity {

    // todo: write your code here
    
    ColorSensor color;
    DistanceSensor distance;
    
    
    Proximity(String name,HardwareMap hardwareMap)
    {
        color =  hardwareMap.get(ColorSensor.class, name);
        distance = hardwareMap.get(DistanceSensor.class,name);
    }
    
    double get_distance()
    {
       return  distance.getDistance(DistanceUnit.CM);
    }
    
    float red()
    {
        return color.red();
    }
    float blue()
    {
        return color.blue();
    }
    float green()
    {
        return color.green();
    }
    
    float hue()
    {
        return red()+blue()+green()/3;
    }
    
    void enable(boolean value)
    {
        color.enableLed(value);
    }
    
    double get_stone()
    {
       return (color.red() * color.green()) / Math.pow(color.blue(), 2);  
    }
    
    
    
    
    
    
    
}
