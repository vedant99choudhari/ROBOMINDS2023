package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.ArmPID;
@Autonomous

public class ArmMotion extends LinearOpMode{
    private DcMotorEx arm1;
    private DcMotorEx arm2;
    private Blinker control_Hub;
    private Blinker expansion_Hub_2;
    private IMU imu;
    
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
        StaticVars vars = new StaticVars(telemetry);
        ArmPID arm = new ArmPID(arm1,arm2);
        arm.moveArm(45,arm1);

        while(must()){
            //arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
            //arm2.setPower(0.3);
            
        }
    }
    boolean must(){
        return opModeIsActive();
    }
}