package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import com.qualcomm.robotcore.hardware.HardwareDevice;
public class Drive {
    
    public DcMotorImplEx[] motors;

    public Drive(HardwareMap hardwareMap,String[] config)
    {
        int num =4;
        motors = new DcMotorImplEx[4];
        for (int i=0;i<4;i++)
        
        {
            motors[i]=hardwareMap.get(DcMotorImplEx.class, config[i]);
        }
        correct_direction();
        //PID();
        mode(0);
    }
    
    public void PID()
    {
        PIDCoefficients temp = new PIDCoefficients(0.3,0.04,0.7);
        for (int i=0;i<4;i++)
        {
        motors[i].setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION,temp );
        }    
            
    }
    
    public void correct_direction() {
       DcMotorSimple.Direction left_direction = DcMotorSimple.Direction.REVERSE;
        motors[3].setDirection(left_direction);
        motors[1].setDirection(left_direction);
    }
    
    
    public void set(double fr,double fl,double br,double bl)
    {
        motors[0].setPower(fr);
        motors[1].setPower(fl);
        motors[2].setPower(br);
        motors[3].setPower(bl);
    }
        public void setAll(double power)
    {
        motors[0].setPower(power);
        motors[1].setPower(power);
        motors[2].setPower(power);
        motors[3].setPower(power);
    }
    
    public void mode(int i)
    {
        if (i==0)
        {
            for(int num =0 ;num <4 ; num = num + 1)
            {
                motors[num].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
        }
        if (i==1)
        {
            for(int num =0 ;num <4 ; num = num + 1)
            {
                motors[num].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
        if (i==2)
        {
            for(int num =0 ;num <4 ; num = num + 1)
            {
                motors[num].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
        if (i==3)
        {
           for(int num =0 ;num <4 ; num = num + 1)
            {
                motors[num].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } 
        }
        
    }
    
    boolean is_busy()
    {
        return motors[0].isBusy() ||motors[1].isBusy() ||motors[2].isBusy() ||motors[3].isBusy() ;
    }

};
