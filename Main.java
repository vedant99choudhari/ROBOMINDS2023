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
    int arm1_target;
    int arm2_target;
    // todo: write your code here
    DriveFunctions movement;
    public void runOpMode(){
       
       
       
        arm1 = hardwareMap.get(DcMotorEx.class, "ARM1");
        arm2 = hardwareMap.get(DcMotorEx.class, "ARM2");
        arm1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm2.setDirection(DcMotorEx.Direction.REVERSE);
        ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        
        right = hardwareMap.get(Servo.class, "right");
        arm3 = hardwareMap.get(Servo.class, "arm3");
        left = hardwareMap.get(Servo.class, "left");
        
        String [] motorConfig = {"FR","FL","BR","BL"};
        mecanum = new Drive2023(hardwareMap, motorConfig);
        
        StaticVars vars = new StaticVars(telemetry,"RED");
        
        
        mecanum.setMotorMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        mecanum.setMotorMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        
        colourL = hardwareMap.get(ColorSensor.class,"COLOURL"); 
        
        colourR = hardwareMap.get(ColorSensor.class,"COLOURR"); 
        
        
        distanceFL = hardwareMap.get(DistanceSensor.class,"DISTANCEFL"); 
        distanceB = hardwareMap.get(DistanceSensor.class,"DISTANCEB");
        distanceR = hardwareMap.get(DistanceSensor.class,"DISTANCER"); 
        
        
        
        right.setPosition(0);
        left.setPosition(0.9);
        
        waitForStart();
        gyro = new Gyro2023(hardwareMap, mecanum);
        
        
        
        movement = new DriveFunctions(mecanum,gyro,this,hardwareMap);
        boolean flag = true;
        
        gyro.TargetAngle = 0.0;
        
        while((opModeIsActive() && flag)){
        
            arm3.setPosition(0.3);
            String element = "LEFT";
            movement.moveTill(0d,25d,true);
            if(element == "RIGHT"){
                gyro.TargetAngle = -90.0;
                lock();
                left.setPosition(0.3);
            }else if (element == "CENTER"){
                movement.moveTill(0d,3d,true);
                left.setPosition(0.3);
                movement.moveTill(0d,-3d,true);
                gyro.TargetAngle = -90.0;
                lock();
            }else{
                gyro.TargetAngle = 90.0;
                lock();
                left.setPosition(0.3);
                gyro.TargetAngle = -90.0;
                lock();
            }
            movement.moveTillDistance(10,0,distanceFL);
            if(element == "RIGHT"){
            movement.moveTillDistance(24,3,distanceR);
            }else if(element == "CENTER"){
                movement.moveTillDistance(30,3,distanceR);
            }else{
                movement.moveTillDistance(36,3,distanceR);
            }
            //First level with right
            arm2_target = 140;
            while (movearm2() && must()){
                telemetry.update();
            
            }
            
            
            arm3.setPosition(0.6);
            movement.moveTillDistance(11,0,distanceFL);
            right.setPosition(0.5);
            
            
            /* for centerstage
            arm2_target = 158;
            movement.moveTillDistance(18,0,distanceFL);
            while (movearm2() && must()){
                telemetry.update();
                
            
            }*/
            /* //For Level 2
            arm2_target = 160;
            while (movearm2() && must()){
                telemetry.update();
            
            }
            
            
            arm3.setPosition(0.58);
            
            movement.moveTillDistance(9.75,0,distanceFL);
            
            right.setPosition(0.5);*/
            //for level 3
            /* 
            arm2_target = 180;
            while (movearm2() && must()){
                telemetry.update();
            }
            telemetry.update();
            //wait(5);
            
            arm3.setPosition(0.6);
            
            movement.moveTillDistance(8.7,0,distanceFL);
            
            right.setPosition(0.5);
            

            */
            /* for level 4
            int counter = 0;
            arm2_target = 200;
            boolean test = movearm2();
            while (test && must()){
                counter++;
                test = movearm2();
                telemetry.addData("count", counter);
                telemetry.addData("bool", test);
                telemetry.addData("opActive", must());
                telemetry.update();
            }
            counter++;
            telemetry.addData("count", counter);
            telemetry.addData("bool", test);
            telemetry.addData("opActive", must());
            telemetry.update();
            arm3.setPosition(0.73);
            movement.moveTillDistance(4.2,0,distanceFL);
            right.setPosition(0.5);
            */
            /* Level 5
            int counter = 0;
            arm1_target = 80;
            arm2_target = 170;
            while ((movearm1() || movearm2())&& must()){
                telemetry.update();
                counter++;
            telemetry.addData("count", counter);
            telemetry.update();
            }
            telemetry.update();
            //wait(5);
            
            arm3.setPosition(0.75);
            movement.moveTillDistance(5.75,0,distanceFL);
            
            right.setPosition(0.5);
            */
            /* Level 6
            int counter = 0;
            arm1_target = 90;
            arm2_target = 180;
            while ((movearm1() || movearm2())&& must()){
                telemetry.update();
                counter++;
            telemetry.addData("count", counter);
            telemetry.update();
            }
            telemetry.update();
            //wait(5);
            
            movement.moveTillDistance(2.7,0,distanceFL);
            arm3.setPosition(0.75);
            wait(1);
            right.setPosition(0.5);
            */
            /*
            movement.moveTillDistance(14,0,distanceFL);
            arm3.setPosition(0.4);
            arm1_target = 0;
            arm2_target = 0;
            while ((movearm1() || movearm2())&& must()){
                telemetry.update();
                counter++;
            telemetry.addData("count", counter);
            telemetry.update();
            
            }*/
            flag = false;
            //telemetry.update();
        }
       
        while(must()){
            telemetry.addData("Distance",distanceFL.getDistance(DistanceUnit.INCH));
            telemetry.addData("Gyro",gyro.getOrientation());
            telemetry.addData("Arm2 Encoder: ", arm2.getCurrentPosition());
            telemetry.update();
        }
    }
    
    public void wait(int seconds){
        long now = StaticVars.clock.now(TimeUnit.SECONDS);
            while((StaticVars.clock.now(TimeUnit.SECONDS) - now < seconds) && must()){
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
        telemetry.addData("Arm angle" , angle);
        double error = arm2_target - angle;
        telemetry.addData("error" , error);
        int multiplier = 1;
        if (error < 0){
            multiplier = -1;
        }
        if(Math.abs(error) < 4){
            arm2.setPower(getPowerForVoltage(0.03) * multiplier);
            telemetry.addData("","Holding");
            return false;
        }
        else{
            arm2.setPower(getPowerForVoltage(0.4)* multiplier);
            telemetry.addData("","Moving");

            return true;
        }
        
    }
    
    double getPowerForVoltage(double voltage){
        double power = voltage/ControlHub_VoltageSensor.getVoltage();
        telemetry.addData("Motor Power:" , power);
        return power;
    }
    
    public boolean must(){
        movearm1();
        movearm2();
        return opModeIsActive();
    }

    
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
    
    void lock(){
        int counter = 0;
        do{
            counter++;
            telemetry.addData("Counter",counter);
            telemetry.update();
        }while(gyro.angularLock() && opModeIsActive());
        mecanum.setAllPower(0.0);
    }

    
}
    
