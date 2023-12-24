package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.ArmMotion;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import java.lang.Math;

public class ArmPID {
    private DcMotorEx arm1;
    private DcMotorEx arm2;
    private Blinker control_Hub;
    private Blinker expansion_Hub_2;
    private IMU imu;
    
    ArmPID(DcMotorEx arm1_temp, DcMotorEx arm2_temp){
        arm1 = arm1_temp;
        arm2 = arm2_temp;
    }
    // todo: write your code here
    
    /**
     * Moves the arm to a target angle using a "P with finer P" system.
     * Compares calculated power output to maximum power possible for current holding power
     * (Required to avoid jerk)
     * Takes minimum and moves motor
     *
     * @param  target_angle  the target angle to move the arm to
     *
     */
    public void moveArm(double target_angle, DcMotorEx arm){
        double angle = getCurrentAngle(arm);
        double error = Math.abs(target_angle-angle);
        double MaximumPower = 0.1;
        double kp;
        double PIDPower;
        double PowerOutput;
        int counter = 0;
        while (error > 1){
            angle = getCurrentAngle(arm);
            counter++;
            StaticVars.telemetry.addData("Counter: ", counter);
            StaticVars.telemetry.update();
            error = Math.abs(target_angle-angle);
            //arm2
            if(arm  == arm2){
                kp = 0.01;
                
            //arm1 - Coarse
            }else if (error > 10)
                kp = 0.04;
            // arm1 - Fine
            else{
                kp = 0.004;
            }
            PIDPower = kp*error;
            StaticVars.telemetry.addData("PIDPower", PIDPower);
            StaticVars.telemetry.addData("Current",arm.getCurrent(CurrentUnit.AMPS));
            //Compare power added percentage to maximum power percetage
            PowerOutput = Math.min(PIDPower,MaximumPower);
            arm.setPower(PowerOutput);
        }
        //arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        arm.setPower(getHoldingPower(target_angle));
        
        
    }
    private double getHoldingPower(double angle){
        return 0.0005;
    }

    private double getCurrentAngle(DcMotorEx arm){
        double angle;
        if(arm == arm2){
             angle = (arm.getCurrentPosition() * 0.23) + 5 ;
        }else{
             angle = (arm.getCurrentPosition() * 0.279069) ;
        }
        StaticVars.telemetry.addData("Angle: ", angle);
        StaticVars.telemetry.addData("arm1_position: ", arm1.getCurrentPosition());
        StaticVars.telemetry.addData("arm2_position: ", arm2.getCurrentPosition());
        return angle;
    }
}    