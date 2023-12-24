package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.concurrent.TimeUnit;
import java.lang.Math;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.IMU;
//import org.firstinspires.ftc.teamcode.ArmPID;
@Autonomous

public class ArmMotion extends LinearOpMode{
    private DcMotorEx arm1;
    private DcMotorEx arm2;
    private IMU imu;
    private VoltageSensor ControlHub_VoltageSensor;
    // todo: write your code here
    @Override
    public void runOpMode(){
        waitForStart();
        arm1 = hardwareMap.get(DcMotorEx.class, "ARM1");
        arm2 = hardwareMap.get(DcMotorEx.class, "ARM2");
        arm1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm2.setDirection(DcMotorEx.Direction.REVERSE);
        
        ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        

        while(must()  ){
            movearm2(160);
        }
    }
    
    
    public boolean movearm1(int target){
        int position = arm1.getCurrentPosition();
        double angle = (position*1);
        telemetry.addData("Arm position" , position);
        telemetry.addData("Arm angle" , angle);
        double error = target - angle;
        int multiplier = 1;
        if (error < 0){
            multiplier = -1;
        }
        if(Math.abs(error ) < 2){
            arm1.setPower(getPowerForVoltage(0.01) * multiplier);
            telemetry.addData("","Holding");
            return false;
        }
        else{
            arm1.setPower(getPowerForVoltage(0.8)* multiplier);
            telemetry.addData("","Moving");
            return true;
        }
    }
    
public boolean movearm2(int target){
        int position = arm2.getCurrentPosition();
        double angle = (position*1);
        telemetry.addData("Arm position" , position);
        telemetry.addData("Arm angle" , angle);
        double error = target - angle;
        int multiplier = 1;
        if (error < 0){
            multiplier = -1;
        }
        if(Math.abs(error ) < 2){
            arm2.setPower(getPowerForVoltage(0.03) * multiplier);
            telemetry.addData("","Holding");
            return false;
        }
        else if (Math.abs(error) > 30){
            arm2.setPower(getPowerForVoltage(0.8)* multiplier);
            telemetry.addData("","Coarse Moving");
            return true;
        }else{
            arm2.setPower(getPowerForVoltage(0.35)* multiplier);
            telemetry.addData("","Fine Moving");
            return true;
        }
    }
    
    double getPowerForVoltage(double voltage){
        double power = voltage/ControlHub_VoltageSensor.getVoltage();
        telemetry.addData("Motor Power:" , power);
        return power;
    }
    
    boolean must(){
        telemetry.update();
        return opModeIsActive();
    }
}