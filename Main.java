package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.IMU;

@Autonomous

public class Main extends LinearOpMode {
    private DcMotorEx arm1;
    private DcMotorEx arm2;
    private DcMotorEx BL;
    private DcMotorEx BR;
    private ColorSensor COLOURL;
    private ColorSensor COLOURR;
    private Blinker control_Hub;
    private DistanceSensor DISTANCEB;
    private DistanceSensor DISTANCEL;
    private DistanceSensor DISTANCER;
    private Blinker expansion_Hub_2;
    private DcMotorEx FL;
    private DcMotorEx FR;
    private DcMotorEx HANGER;
    private HardwareDevice webcam_1;
    private IMU imu;
    Drive2023 mecanum;
    String field = "RED";
    // todo: write your code here
    
    public void runOpMode(){
        
        arm1 = hardwareMap.get(DcMotorEx.class, "ARM1");
        arm2 = hardwareMap.get(DcMotorEx.class, "ARM2");
        String [] motorConfig = {"FR","FL","BR","BL"};
        mecanum = new Drive2023(hardwareMap, motorConfig);
        mecanum.setMotorMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        StaticVars vars = new StaticVars(telemetry);
        
        
        
        boolean flag = true;
        waitForStart();
        
        while((opModeIsActive() && flag)){
        
            telemetry.addData("key" , "test");
            moveTill(0.0,10.0,false);
            
            flag = false;
            telemetry.update();
        }
        
    }
        /**
     * Moves the robot in the x and y direction
     *
     * @param  x_in      the x value in inches
     * @param  y_in      the y value in inches
     * @param  timedFailSafe     whether to have a time failsafe
     */
    public void moveTill(Double x_in, Double y_in, boolean timedFailSafe){
        int x_enc = ((Number)(x_in * -24.053125)).intValue();
        int y_enc = ((Number)(y_in * -24.053125)).intValue();
        double power = 0.05;
        mecanum.setMotorMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        double displacement = setTargets(x_enc,y_enc);
        double time = displacement / 17;
       mecanum.setMotorMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        mecanum.setAllPower(power);
        //TODO: Implement must()
        //must() for backboard failsafe?
        //now implementation
        while (mecanum.isMotorsBusy() && opModeIsActive() )
        {
            telemetry.addData("FR", mecanum.motors[0].getCurrentPosition());
            telemetry.addData("FL", mecanum.motors[1].getCurrentPosition());
            telemetry.addData("BL", mecanum.motors[2].getCurrentPosition());
            telemetry.addData("BR", mecanum.motors[3].getCurrentPosition());
            telemetry.update();
        }
        mecanum.setAllPower(0); 
        mecanum.setMotorMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
    double setTargets(int target_x , int target_y)
    {
        if(field == "RED")
         {
             target_x = -target_x;
         }
        double angle = Math.toDegrees( Math.atan2(target_y,target_x));
        double target =Math.sqrt(2) * Math.sqrt((target_x*target_x) + (target_y *target_y));
        double diagonal1 = - target * Math.sin(Math.toRadians(angle+45));
        double diagonal2 = target * Math.cos(Math.toRadians(angle+45));
        
        int d1 = (int) Math.round(diagonal1);
        int d2 = (int) Math.round(diagonal2);
        //TODO : Integrate into Drive2023
        mecanum.motors[0].setTargetPosition(d2);
        mecanum.motors[1].setTargetPosition(d1);
        mecanum.motors[2].setTargetPosition(d1);
        mecanum.motors[3].setTargetPosition(d2);
        
        
        return Math.sqrt(target/Math.sqrt(2));
    }

    
}
    
