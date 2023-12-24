package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;


public class Telescopic {

    Servo grip;
    DcMotorImplEx lift;
    double min,max,now;
    int target,Max;
    boolean is_open,state,state1,lift1;
    int old_position;
    
     Telescopic (HardwareMap hardwareMap,String motor,String servo)
    {
        lift = hardwareMap.get(DcMotorImplEx.class,motor);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        grip = hardwareMap.get(Servo.class,servo);
        target = 0;
        state = false;
        state1 = false;
        lift1 = false;
        old_position = 100;
        is_open = true;
        now = 0;
    }
    
    Telescopic (HardwareMap hardwareMap,String motor,String servo,int MAX)
    {
        lift = hardwareMap.get(DcMotorImplEx.class,motor);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        grip = hardwareMap.get(Servo.class,servo);
        target = 0;
        state = false;
        state1 = false;
        lift1 = false;
        Max = MAX;
        old_position = 100;
        is_open = true;
    }
   
    
    void set_min_max(double Min,double Max)
    {
        min =Min;
        max =Max;
        
        grip.setPosition(min);
        
    }
    
    void grip_open()
    {
        grip.setPosition(min);
    }
    
    void grip_close()
    {
        grip.setPosition(max);
    }
    
    void flip()
    {
        if(is_open)
        {
           grip.setPosition(min);
        }
         else
        {
            grip.setPosition(max);
        }
        
        is_open = !is_open ; 
    }
    
    
    // void grip_motion(boolean input)
    // {
        
    //     if (input)
    //     {
    //         if(state)
    //         {
    //             return;
    //         }
    //         else
    //         {
    //             state = true;
    //             flip();
    //         }
    //     }
    //     else
    //     {
    //         state =false;
    //         return;
    //     }
    // }
    
    void move()
    {
        int current_position = lift.getCurrentPosition();
        
        
        
        int difference = target - current_position;
        
        int H_limit = 40;
        
        if(difference>H_limit)
        {
            difference = H_limit;
        }
        if(difference<-H_limit)
        {
            difference = -H_limit;
        }

        
        if(Math.abs(difference) > 2)
        {
           double  speed = 0.03*difference;
            
            if(Math.abs(speed) < 0.1 )
            {
                speed = 0.1 * (speed/Math.abs(speed));
            }
            lift.setPower(speed);
        }
        else
        {
            lift.setPower(0);
        }
    }
    
    void change(int amount)
    {
        target = target + amount;
        if(target> Max)
        {
            target =Max;
        }
        else if(target < 0)
        {
            target = 0;
        }
        
    }
    void increment(int amount)
    {
        target = target+amount;
    }
    
    void decrement(int amount)
    {
        target = target-amount;
        
    }
    
    int print()
    {
        return lift.getCurrentPosition();
    }
    
    int printtarget()
    {
        return target;
    }
    
    void lift_max(boolean input)
    {
       if (input)
        {
            if(lift1)
            {
                return;
            }
            else
            {
                lift1 = true;
                if(target != Max)
                {
                    target = 0;
                }
                else
                {
                    target = Max;
                }
            }
        }
        else
        {
            lift1 =false;
            return;
        }  
    }
    
    
    
    
    void lift_motion(boolean input)
    {
       if (input)
        {
            if(state1)
            {
                return;
            }
            else
            {
                state1 = true;
                if(target == 0)
                {
                    if(is_open)
                    target = old_position + 100;
                    else
                    target = old_position;
                }
                else
                {
                    old_position = target;
                    target = 0;
                }
            }
        }
        else
        {
            state1 =false;
            return;
        }  
    }
    
    void set(int input)
    {
        target = input;
    }
    
}
