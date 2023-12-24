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
import java.util.function.Supplier;

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
    private Gyro2023 gyro;
    private VoltageSensor ControlHub_VoltageSensor;
    int arm1_target;
    int arm2_target;
    double arm3_target;
    int level = 0;
    Drive2023 mecanum;
    // todo: write your code here
    public void runOpMode(){
        arm1 = hardwareMap.get(DcMotorEx.class, "Arm1");
        arm2 = hardwareMap.get(DcMotorEx.class, "Arm2");
        arm1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm1.setDirection(DcMotorEx.Direction.REVERSE);
        ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        String [] motorConfig = {"FR","FL","BR","BL"};
        mecanum = new Drive2023(hardwareMap, motorConfig);

        StaticVars vars = new StaticVars(telemetry,"RED");


        mecanum.setMotorMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        mecanum.setMotorMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        gyro = new Gyro2023(hardwareMap, mecanum);
        right = hardwareMap.get(Servo.class, "right");
        arm3 = hardwareMap.get(Servo.class, "arm3");
        left = hardwareMap.get(Servo.class, "left");
        arm1_target = 0;
        arm2_target = 0;
        arm3_target =  0.2;
        left_open  = true;
        right_open = true;
        waitForStart();
        while(must()){
            arm();
            move();
            telemetry.update();
        }


    }
    void arm(){
        telemetry.addData("level", level);
        if(fallingEdge(() -> this.gamepad2.left_bumper)){
            if(left_open){
                left.setPosition(0.5);

                left_open = false;
            }
            else{
                left.setPosition(0.25);
                left_open = true;
            }

        }
        if(fallingEdge(() -> this.gamepad2.right_bumper)){
            if(right_open){
                right.setPosition(0.55);

                right_open = false;
            }
            else{
                right.setPosition(0.35);
                right_open = true;
            }

        }

        if(fallingEdge(() -> this.gamepad2.a)){
            if(level <= 5){
                level++;
            }
        }
        if(fallingEdge(() -> this.gamepad2.b)){
            if(level > 1){
                level--;
            }
        }
        switch (level){
            case 1:{
                arm1_target = 0;
                arm2_target = 175;
                arm3_target =  0.6;
            }
            break;
            case 2:{
                arm1_target = 0;
                arm2_target = 205;
                arm3_target =  0.7;
            }
            break;
            case 3:{
                arm1_target = 0;
                arm2_target = 225;
                arm3_target =  0.7;
            }
            break;
            case 4:{
                arm1_target = 0;
                arm2_target = 250;
                arm3_target =  0.75;
            }
            break;
            case 5:{
                arm1_target = 80;
                arm2_target = 200;
                arm3_target =  0.75;
            }
            break;
            case 6:{
                arm1_target = 90;
                arm2_target = 250;
                arm3_target =  0.85;
            }
            break;
            default:

        }
        if(fallingEdge(() -> this.gamepad2.x)){
            arm3.setPosition(0.2);
            arm3_target = 0.2;
            while(level>=1){
                level--;
                while(must() && arm2.isBusy()){

                }
            }

            arm1_target = 0;
            arm2_target = 0;
            arm3_target = 0.2;

        }

        if(fallingEdge(() -> this.gamepad2.y)){
            arm3_target = 0.45;
            arm2_target = -15;
        }


    }

    void move()
    {
        double limit = 0.7;
        double speed_y = limit*this.gamepad1.left_stick_y;
        double speed_x = -limit *this.gamepad1.left_stick_x;
        double speed_rotate =  - 0.4 *this.gamepad1.right_stick_x;
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
            speed_y = speed_y * 0.3;
            speed_x = speed_x* 0.3;
            speed_rotate = speed_rotate * 0.3;
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

        if(fallingEdge(() -> this.gamepad1.b)){
            gyro.TargetAngle = -90.0;
            lock();
        }
        if(fallingEdge(() -> this.gamepad1.a)){
            gyro.TargetAngle = 90.0;
            lock();
        }
        if(fallingEdge(() -> this.gamepad1.x)){
            gyro.TargetAngle = 45.0;
            lock();

        }
        if(fallingEdge(() -> this.gamepad1.left_bumper)){
            double angle_current = gyro.getOrientation();
            double Quadrant = (Math.floor(angle_current /90));
            
            gyro.TargetAngle = (Quadrant +1)*90;

            lock();

        }
        if(this.gamepad1.left_trigger > 0.3){
            double angle_current = gyro.getOrientation();
            double Quadrant = (Math.floor(angle_current /90));
            gyro.TargetAngle = Quadrant*90;

            lock();
        }
        if(fallingEdge(() -> this.gamepad1.guide))gyro.reset();
    }

    double getPowerForVoltage(double voltage){
        double power = voltage/ControlHub_VoltageSensor.getVoltage();
        telemetry.addData("Motor Power:" , power);
        return power;
    }

    

    public boolean fallingEdge(Supplier<Boolean> button){
        if(button.get()){
            while(button.get()){
                must();
                
            }
            return true;
        }
        return false;
    }
    public boolean must(){
        arm1.setTargetPosition(arm1_target);
        arm2.setTargetPosition(arm2_target);
        arm1.setPower(getPowerForVoltage(3));
        arm2.setPower(getPowerForVoltage(3));
        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("Arm1", arm1.getCurrentPosition());
        telemetry.addData("Arm2", arm2.getCurrentPosition());
        telemetry.addData("Gyro val: ", gyro.getOrientation());
        telemetry.addData("Quadrant", Math.floor(gyro.getOrientation()/90));
        telemetry.addData("Target", gyro.TargetAngle);
        if(!(arm1.isBusy() || arm2.isBusy())){
            double servo_target = gamepad2.right_trigger > 0.3 ? arm3_target +0.3 : arm3_target;
            arm3.setPosition(servo_target);
        }


        return opModeIsActive();
    }
    void lock(){
        do{
            telemetry.update();
        }while(gyro.angularLock() && opModeIsActive() );
        mecanum.setAllPower(0.0);
    }
}