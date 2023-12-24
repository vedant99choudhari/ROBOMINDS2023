package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;


public class PlayGroundSupport {

    LinearOpMode opmode;
    // todo: write your code here
    PlayGroundSupport(LinearOpMode opmode_temp){
        opmode = opmode_temp;
    }
    void runTillEnd(){
        int counter = 0;
        while(opmode.opModeIsActive()){
        
        opmode.telemetry.addData("Counter",counter);
        opmode.telemetry.update();
        counter++;
    }
    
}
    
}