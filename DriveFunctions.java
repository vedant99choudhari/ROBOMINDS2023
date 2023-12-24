package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Drive2023;
import org.firstinspires.ftc.teamcode.Gyro2023;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;

public class DriveFunctions {
    Gyro2023 gyro;
    Drive2023 mecanum;
    LinearOpMode opMode;
    DistanceSensor distanceL,distanceB,distanceR;
    
    public DriveFunctions(Drive2023 mecanum_t, Gyro2023 gyro_t,LinearOpMode opMode_t, HardwareMap hardwareMap){
        mecanum = mecanum_t;
        gyro = gyro_t;
        opMode = opMode_t;

    }
    /**
     * Moves the robot in the x and y direction
     *
     * @param  x_in      the x value in inches
     * @param  y_in      the y alue in inches
     * @param  timedFailSafe     whether to have a time failsafe
     */
    public void moveTill(Double x_in, Double y_in, boolean timedFailSafe){
        int x_enc = ((Number)(x_in * 24.053125)).intValue();
        int y_enc = ((Number)(y_in * -24.053125)).intValue();
        double power = 0.07;
        mecanum.setMotorMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        double displacement = setTargets(x_enc,y_enc);
        double time = displacement / 17;
        mecanum.setMotorMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        mecanum.setAllPower(power);
        //TODO: Implement must()
        //must() for backboard failsafe?
        //now implementation
        while (mecanum.isMotorsBusy() && opMode.opModeIsActive() )
        {
            opMode.telemetry.addData("FR", mecanum.motors[0].getCurrentPosition());
            opMode.telemetry.addData("FL", mecanum.motors[1].getCurrentPosition());
            opMode.telemetry.addData("BL", mecanum.motors[2].getCurrentPosition());
            opMode.telemetry.addData("BR", mecanum.motors[3].getCurrentPosition());
            opMode.telemetry.update();
        }
        mecanum.setAllPower(0); 
        mecanum.setMotorMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
    
    
    /**
     * 
     * @param target - Target value for distance sensor
     * @param direction - 
     *      0 - Forward
     *      1 - Right
     *      2 - Back
     *      3 - Left
     * @param sensor
     */
    public void moveTillDistance(double target, int direction, DistanceSensor sensor){
        double d1 = 0;
        double d2 = 0;
        do{
            opMode.telemetry.update();
        }while(gyro.angularLock() && opMode.opModeIsActive());
        
        if(StaticVars.field == "RED"){
            if(direction == 1)direction = 3;
            else if(direction == 3) direction = 1;
        }
        
        
        if (direction==0)
        {
          d1 = -1;
          d2 = -1;
        }
        else if (direction == 1)
        {
           d1 = 1;
           d2 = -1;
        }else if (direction == 2){
            d1 = 1;
           d2 = 1;
        }else{
            d1 = -1;
           d2 = 1;
        }
        
        double reading1 = sensor.getDistance(DistanceUnit.INCH);
        double BaseSpeed = 0.1;
        double MinSpeed = 0.05;
        double MaxSpeed = 0.05;
        double buffer = 0.3;
        double range = 3;
        double speed = 0;
        
        opMode.telemetry.addData("distance",reading1);
        opMode.telemetry.update();
                
        double value;        
        while( Math.abs(reading1-target)>buffer && opMode.opModeIsActive()){
            value = gyro.getOrientation();
           
            if(Math.abs(value) >4){
               gyro.continousA();
            }
            
            reading1 = sensor.getDistance(DistanceUnit.INCH);
            
            if(Math.abs(reading1 - target) >range){
                BaseSpeed = MaxSpeed;
            }else {
             BaseSpeed = 0.01 * Math.abs(reading1 - target);
             if(BaseSpeed < MinSpeed)
             {
                 BaseSpeed = MinSpeed;
             }
            }
            
            if (reading1 >target){
                speed = -1*BaseSpeed;
            }else{
              speed = 1 * BaseSpeed;
            }
               
            mecanum.setPowers(d1*speed,d2*speed,d2*speed,d1*speed);   
            
            opMode.telemetry.addData("distance",reading1);
            opMode.telemetry.update();
            
            
        }     

        reading1 = sensor.getDistance(DistanceUnit.INCH);
        
        mecanum.setPowers(0,0,0,0);
        
        
    }
    
    
    
    double setTargets(int target_x , int target_y)
    {
        if(StaticVars.field == "RED")
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
