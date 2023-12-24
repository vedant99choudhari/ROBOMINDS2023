package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;


@Autonomous
public class ArmMotion extends LinearOpMode{
    private DcMotorEx arm1;
    private DcMotorEx arm2;
    private Servo arm3;
    private IMU imu;
    private VoltageSensor ControlHub_VoltageSensor;
    
    // todo: write your code here
    @Override
    public void runOpMode(){
        waitForStart();
        StaticVars.telemetry = telemetry;
        StaticVars.field = "NA";
        arm1 = hardwareMap.get(DcMotorEx.class, "ARM1");
        arm2 = hardwareMap.get(DcMotorEx.class, "ARM2");
        arm3 = hardwareMap.get(Servo.class,"ARM3");
        arm1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
    
        
        
        
        while(must()){
        arm1.setPower(getPowerForArm(0.08));
            
            
        }
    }
    
    double getPowerForArm(double voltage){
        double power = voltage / ControlHub_VoltageSensor.getVoltage();
        telemetry.addData("Motor power", power);
        telemetry.update();
        return power;
        
    }
    boolean must(){
        return opModeIsActive();
    }
}