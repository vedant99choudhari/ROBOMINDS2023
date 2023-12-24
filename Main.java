package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.IMU;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous

public class Main extends LinearOpMode {
    private DcMotorEx arm1;
    private DcMotorEx arm2;
    private DcMotorEx BL;
    private DcMotorEx BR;
    private ColorSensor colourL;
    private ColorSensor colourR;

    private Blinker control_Hub;
    private DistanceSensor distanceB;
    private DistanceSensor distanceFL;
    private DistanceSensor distanceR;
    private DistanceSensor distanceFR;
    private DistanceSensor distanceL;
    private Blinker expansion_Hub_2;
    private DcMotorEx FL;
    private DcMotorEx FR;
    private DcMotorEx HANGER;
    private HardwareDevice webcam_1;
    private IMU imu;
    Drive2023 mecanum;
    String field = "RED";
    Gyro2023 gyro;
    VoltageSensor ControlHub_VoltageSensor;
    private Servo right;
    private Servo left;
    private Servo arm3;
    int arm1_target = 0;
    int arm2_target= 0;
    // todo: write your code here
    DriveFunctions movement;
    public void runOpMode(){
       
       
       
        arm1 = hardwareMap.get(DcMotorEx.class, "Arm1");
        arm2 = hardwareMap.get(DcMotorEx.class, "Arm2");
        
        ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        
        
        
        right = hardwareMap.get(Servo.class, "right");
        arm3 = hardwareMap.get(Servo.class, "arm3");
        left = hardwareMap.get(Servo.class, "left");
        
        String [] motorConfig = {"FR","FL","BR","BL"};
        mecanum = new Drive2023(hardwareMap, motorConfig);
        
        StaticVars vars = new StaticVars(telemetry,"RED");
        
        
        mecanum.setMotorMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        mecanum.setMotorMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        
        distanceFR = hardwareMap.get(DistanceSensor.class,"FRDISTANCE");
        distanceFL = hardwareMap.get(DistanceSensor.class,"FLDISTANCE"); 
        distanceB = hardwareMap.get(DistanceSensor.class,"REARDISTANCE");
        distanceR = hardwareMap.get(DistanceSensor.class,"DISTANCER"); 
        distanceL = hardwareMap.get(DistanceSensor.class,"DISTANCEL"); 
        
        
        right.setPosition(0);
        left.setPosition(0.85);
        
        waitForStart();
        gyro = new Gyro2023(hardwareMap, mecanum);
        gyro.reset();
        
        
        movement = new DriveFunctions(mecanum,gyro,this,hardwareMap);
        boolean flag = true;
        gyro.TargetAngle = 0.0;
        while((opModeIsActive() && flag)){
            arm1.setTargetPosition(arm1_target);
            arm2.setTargetPosition(arm2_target);
            arm1.setPower(getPowerForVoltage(0.95));
            arm2.setPower(getPowerForVoltage(0.95));
            arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm3.setPosition(0.3);
            
            int element = 2;
            movement.moveTill(0d,30d,true);
            
            // if(element == 0){
            //     gyro.TargetAngle = -90.0;
            //     lock();
            //     arm3.setPosition(0.4);
            //     left.setPosition(0.3);
            //     arm3.setPosition(0.3);
            // }else if (element == 1){
            //     arm3.setPosition(0.4);
            //     left.setPosition(0.3);
            //     arm3.setPosition(0.3);
            // }else{
            //     telemetry.addData("Test","Test");
            //     telemetry.update();
            //     gyro.TargetAngle = 90.0;
            //     lock();
            //     arm3.setPosition(0.4);
            //     left.setPosition(0.3);
            //     arm3.setPosition(0.3);
                
            // }
            gyro.TargetAngle =90.0;
            // arm3.setPosition(0.5);
            lock();
            
            movement.moveTill(0d,3d,true);
            
            // arm3.setPosition(0.5);
            left.setPosition(0.3);
            wait(200);
            
            movement.moveTill(0d,-10d,true);
           gyro.TargetAngle =-90.0;
            lock();
            
             arm2.setTargetPosition(160);
            while(arm2.isBusy() && must()){}
            arm3.setPosition(0.55);
            movement.moveTillDistance(11,0,distanceFR);
            if(element == 0){
            movement.moveTillDistance(36,3,distanceR);
            }else if(element == 1){
                movement.moveTillDistance(41,3,distanceR);
            }else{
                movement.moveTillDistance(31,3,distanceR);
            }
            
            flag = false;
            //telemetry.update();
        }
       
        while(must()){
            
        }
    }
    public double getAverageDistance(DistanceSensor sensor){
        double sum = 0;
        for(int i = 1; i<=10; i++){
            sum = sum + sensor.getDistance(DistanceUnit.INCH);     
        }
        return sum/10;
    }
    
    public void wait(int seconds){
        long now = StaticVars.clock.now(TimeUnit.NANOSECONDS);
            while((StaticVars.clock.now(TimeUnit.NANOSECONDS) - now < seconds) && must()){
            telemetry.addData("Elapsed Time",(StaticVars.clock.now(TimeUnit.SECONDS) - now));
            telemetry.addData("Arm2 Encoder: ", arm2.getCurrentPosition());
            }
    }
        /**
     * Moves the robot in the x and y direction
     *
     * @param  x_in      the x value in inches
     * @param  y_in      the y value in inches
     * @param  timedFailSafe     whether to have a time failsafe
     */
    double getPowerForVoltage(double voltage){
        double power = voltage/ControlHub_VoltageSensor.getVoltage();
        telemetry.addData("Motor Power:" , power);
        return power;
    }
    
    public boolean must(){
        telemetry.addData("FR target", mecanum.motors[0].getTargetPosition());
            telemetry.addData("FR position", mecanum.motors[0].getCurrentPosition());
            telemetry.addData("FL target", mecanum.motors[1].getTargetPosition());
            telemetry.addData("FL position", mecanum.motors[1].getCurrentPosition());
            telemetry.addData("BR target", mecanum.motors[2].getTargetPosition());
            telemetry.addData("BR position", mecanum.motors[2].getCurrentPosition());
            telemetry.addData("BL target", mecanum.motors[3].getTargetPosition());
            telemetry.addData("BL position", mecanum.motors[3].getCurrentPosition());
            telemetry.update();
        return opModeIsActive();
    }

    
    void lock(){
        
        do{
            
        }while(gyro.angularLock() && opModeIsActive());
        mecanum.setAllPower(0.0);
    }

    
}
    
