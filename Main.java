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
    // todo: write your code here
    DriveFunctions movement;
    public void runOpMode(){
       
       
       
        arm1 = hardwareMap.get(DcMotorEx.class, "ARM1");
        arm2 = hardwareMap.get(DcMotorEx.class, "ARM2");
        arm1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        
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
        arm3.setPosition(0.3);
        waitForStart();
        gyro = new Gyro2023(hardwareMap, mecanum);
        
        
        
        movement = new DriveFunctions(mecanum,gyro,this,hardwareMap);
        boolean flag = true;
        
        gyro.TargetAngle = 0.0;
        
        while((opModeIsActive() && flag)){
        
        
            
            lock();
           
        
            movement.moveTill(0d,28d,true);
            gyro.TargetAngle = -90.0;
            left.setPosition(0.3);
            movement.moveTill(0d,-4d,true);
            lock();
            //movement.moveTillDistance(10,0,distanceFL);
            //movement.moveTillDistance(24,3,distanceR);
            while (!(movearm2(158)) && opModeIsActive()){
                telemetry.update();
                
            
            }
            
            
            arm3.setPosition(0.6);
            movement.moveTillDistance(11,0,distanceFL);
            right.setPosition(0.5);
            
            movearm2(158);
            movement.moveTillDistance(18,0,distanceFL);
            while (!(movearm2(158)) && opModeIsActive()){
                telemetry.update();
                
            
            }
            flag = false;
            
            //telemetry.update();
        }
       
        while(opModeIsActive()){
            telemetry.addData("Distance",distanceFL.getDistance(DistanceUnit.INCH));
            telemetry.addData("Gyro",gyro.getOrientation());
            telemetry.addData("Arm2 Encoder: ", arm2.getCurrentPosition());
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
    public boolean movearm2(int target){
        int position = arm2.getCurrentPosition();
        double angle =(position*-1);
        telemetry.addData("Arm position" , position);
        telemetry.addData("Arm angle" , angle);
        telemetry.addData("Arm angle" , angle);
        if(target - angle < 10){
            arm2.setPower(-getPowerForVoltage(0.01));
            return true;
        }
        else{
            arm2.setPower(-getPowerForVoltage(0.6));
            return false;
        }
    }
     
    double getPowerForVoltage(double voltage){
        double power = voltage/ControlHub_VoltageSensor.getVoltage();
        telemetry.addData("Motor Power:" , power);
        telemetry.update();
        return power;
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
    
