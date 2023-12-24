package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous

public class PlayGround extends LinearOpMode{
    private Blinker expansion_Hub_2;
    private Blinker expansion_Hub_8;
    private DcMotor arm;
    private DcMotor arm_2;
    private DcMotor back_left;
    private DcMotor back_right;
    private DcMotor front_left;
    private DcMotor front_right;
    private Gyroscope imu;
    private Servo servo2;
    private Servo servo;
    public PlayGroundSupport support = new PlayGroundSupport(this);
    // todo: write your code here
    
    
    public void runOpMode(){
        waitForStart();
        
        support.runTillEnd();
    }
}