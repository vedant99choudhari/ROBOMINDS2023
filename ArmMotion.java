package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotor;

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
        ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        

        while(must()){
            int position = arm1.getCurrentPosition();
        double angle =(position*0.7);
        telemetry.addData("Arm position" , position);
        telemetry.addData("Arm angle" , angle);
        telemetry.addData("Arm angle" , angle);
            telemetry.update();
        }
    }
    
    
    public void move(int target){
        int position = arm1.getCurrentPosition();
        double angle =(position*0.7);
        telemetry.addData("Arm position" , position);
        telemetry.addData("Arm angle" , angle);
        telemetry.addData("Arm angle" , angle);
        if(target - angle < 10){
            arm1.setPower(getPowerForVoltage(0.01));
        }
        else{
            arm1.setPower(getPowerForVoltage(0.6));
            
        }
    }
    
    
    
    double getPowerForVoltage(double voltage){
        double power = voltage/ControlHub_VoltageSensor.getVoltage();
        telemetry.addData("Motor Power:" , power);
        telemetry.update();
        return power;
    }
    
    boolean must(){
        telemetry.update();
        return opModeIsActive();
    }
}