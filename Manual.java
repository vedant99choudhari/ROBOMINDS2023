package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp

public class Manual extends LinearOpMode{
    private DcMotor arm1;
    private DcMotor arm2;
    private DcMotor bL;
    private DcMotor bR;
    private ColorSensor cOLOURL;
    private ColorSensor cOLOURR;
    private Blinker control_Hub;
    private DistanceSensor dISTANCEB;
    private DistanceSensor dISTANCEFL;
    private DistanceSensor dISTANCER;
    private Blinker expansion_Hub_2;
    private DcMotor fL;
    private DcMotor fR;
    private DcMotor hANGER;
    private HardwareDevice webcam_1;
    private Servo arm3;
    private IMU imu;
    private Servo left;
    private boolean left_open;
    private boolean right_open;
    private Servo right;
    private VoltageSensor ControlHub_VoltageSensor;
    int arm1_target;
    int arm2_target;
    int level = 0;
    Drive2023 mecanum;
    // todo: write your code here
    public void runOpMode(){
        arm1 = hardwareMap.get(DcMotorEx.class, "ARM1");
        arm2 = hardwareMap.get(DcMotorEx.class, "ARM2");
        arm1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm2.setDirection(DcMotorEx.Direction.REVERSE);
        ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        
        String [] motorConfig = {"FR","FL","BR","BL"};
        mecanum = new Drive2023(hardwareMap, motorConfig);
        
        StaticVars vars = new StaticVars(telemetry,"RED");
        
        
        mecanum.setMotorMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        mecanum.setMotorMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        
        right = hardwareMap.get(Servo.class, "right");
        arm3 = hardwareMap.get(Servo.class, "arm3");
        left = hardwareMap.get(Servo.class, "left");
        left_open  = true;
        right_open = true;
        waitForStart();
        while(must()){
            driver1();
            move();
            telemetry.update();
        }
        
        
    }
    void driver1(){
        telemetry.addData("level", level);
        if(isBumperLPressed()){
            if(left_open){
                left.setPosition(0.4);
                left_open = false;
            }
            else{
              left.setPosition(0.9);
                left_open = true;  
            }
        }
        if(isBumperRPressed()){
            if(right_open){
                right.setPosition(0.5);
                right_open = false;
            }
            else{
              right.setPosition(0);
                right_open = true;  
            }
        }
        if(isButtonAPressed()){
            if(level <= 5){
                level++;
            }
        }
        if(isButtonBPressed()){
            if(level >= 1){
                level--;
            }
        }
        switch (level){
            case 1:{
                arm1_target = 0;
                arm2_target = 140;
                
                }
                break;
            case 2:{
                arm1_target = 0;
                arm2_target = 160;
            
            }
            break;
            case 3:{
                arm1_target = 0;
                arm2_target = 180;
                arm3.setPosition(0.6);
            }
            break;
            case 4:{
                arm1_target = 0;
                arm2_target = 200;
                arm3.setPosition(0.73);
            }
            break;
            case 5:{
                arm1_target = 80;
                arm2_target = 170;
                arm3.setPosition(0.75);
            }
            break;
            case 6:{
                arm1_target = 90;
                arm2_target = 180;
                arm3.setPosition(0.75);
            }
            break;
            default:
            
        }
        if(isButtonXPressed()){
            while(level>=1){
                level--;
                while(must() && arm2.isBusy()){
                
                }
            }
            arm3.setPosition(0.3);
            arm1_target = 0;
            arm2_target = 0;
            
        }
        
        
        
        
    }
    
    void move()
    {
            double limit = 0.75;
            double speed_y = limit*this.gamepad1.right_stick_y;
            double speed_x = -limit *this.gamepad1.right_stick_x;
            double speed_rotate = this.gamepad1.left_stick_x;
            telemetry.addData("SpeedX", speed_x);
            telemetry.addData("SpeedY", speed_y);
            telemetry.update();
            if(this.gamepad1.right_trigger > 0.3)
            {
                speed_y = speed_y/2;
                speed_x = speed_x/2;
                speed_rotate = speed_rotate /2;
            }
            else if(this.gamepad1.right_bumper)
            {
                speed_y = speed_y/3;
                speed_x = speed_x/3;
                speed_rotate = speed_rotate /3;
            }

            double angle = Math.toDegrees( Math.atan2(speed_y,speed_x));
            double speed = Math.sqrt(2) * Math.sqrt((speed_y*speed_y) + (speed_x *speed_x));

            double diagonal1 = -speed * Math.sin(Math.toRadians(angle+45));
            double diagonal2 = speed * Math.cos(Math.toRadians(angle+45));
            
            
            if(Math.abs(speed_rotate) > 0.1)
            {
            mecanum.setPowers(speed_rotate,-speed_rotate,speed_rotate,-speed_rotate); 
            }
            else
            {
                mecanum.setPowers(diagonal2, diagonal1, diagonal1, diagonal2);
            }
            

    }
    public boolean movearm1(){
        int position = arm1.getCurrentPosition();
        double angle = (position*1);
        telemetry.addData("Arm position" , position);
        telemetry.addData("Arm angle" , angle);
        double error = arm1_target - angle;
        int multiplier = 1;
        if (error < 0){
            multiplier = -1;
        }
        if(Math.abs(error ) < 4){
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
    
    public boolean movearm2(){
        int position = arm2.getCurrentPosition();
        double angle = (position*1);
        telemetry.addData("Arm position" , position);
        telemetry.addData("Arm angle" , angle*0.24);
        double error = arm2_target - angle;
        telemetry.addData("error" , error);
        int multiplier = 1;
        if (error < 0){
            multiplier = -1;
        }
        error = Math.abs(error);
        if(error < 3){
            arm2.setPower(getPowerForVoltage(0.03) * multiplier);
            telemetry.addData("","Holding");
            return false;
        }
        else if (error > 40){
            arm2.setPower(getPowerForVoltage(0.8)* multiplier);
            telemetry.addData("","Coarse Moving");
            return true;
        }else if(angle <= 150){
            arm2.setPower(getPowerForVoltage(0.1 +(error * 0.01) )* multiplier);
            telemetry.addData("","Fine Moving less than 150");
            return true;
        }else{
            arm2.setPower(getPowerForVoltage(0.1 + (error * 0.01))* multiplier);
            telemetry.addData("","Fine Moving more than 150");
            return true;
        }
    }
    
    double getPowerForVoltage(double voltage){
        double power = voltage/ControlHub_VoltageSensor.getVoltage();
        telemetry.addData("Motor Power:" , power);
        return power;
    }
    
    boolean isButtonAPressed(){
        if(this.gamepad1.a){
            while(this.gamepad1.a&& must()){
                
            }
            return true;
        }
        
        return false;
    }
    
    boolean isButtonBPressed(){
        if(this.gamepad1.b){
            while(this.gamepad1.b&& must()){
                
            }
            return true;
        }
        
        return false;
    }
    boolean isButtonXPressed(){
        if(this.gamepad1.x){
            while(this.gamepad1.x&& must()){
                
            }
            return true;
        }
        
        return false;
    }
        boolean isBumperLPressed(){
        if(this.gamepad1.left_bumper){
            while(this.gamepad1.left_bumper&& must()){
                
            }
            return true;
        }
        
        return false;
    }
    boolean isBumperRPressed(){
        if(this.gamepad1.right_bumper){
            while(this.gamepad1.right_bumper&& must()){
                
            }
            return true;
        }
        
        return false;
    }
    public boolean must(){
        arm1.setTargetPosition(arm1_target);
        arm2.setTargetPosition(arm2_target);
        arm1.setPower(getPowerForVoltage(0.8));
        arm2.setPower(getPowerForVoltage(0.8));
        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("Arm1", arm1.getCurrentPosition());
        telemetry.addData("Arm2", arm2.getCurrentPosition());
        return opModeIsActive();
    }
}