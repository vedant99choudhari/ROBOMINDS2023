package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.teamcode.Pid2023;
import com.qualcomm.robotcore.hardware.IMU;


public class Gyro2023 {

    public double TargetAngle = 0.0;
    double value;
    IMU imu;
    Orientation angles;
    Telemetry telemetry  = StaticVars.telemetry;
    Drive2023 mecanum;
    Pid2023 control = new Pid2023();
    public Gyro2023(HardwareMap hardwareMap, Drive2023 drive){
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();
        mecanum = drive;
    }
    //TODO : setmode in anuglar lock?
    boolean angularLock()
    {
     
            value = getOrientation();
            continousA();
             
            
        return Math.abs(value-TargetAngle) > 2 ;
    }
    public double getOrientation() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
        
    }
    
    void continousA()
    {
        mecanum.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
         if(control.compute(value,TargetAngle,StaticVars.clock.seconds())){
               if((TargetAngle- value)> 10){
                control.output = -0.15 * control.output;
                }else{
                control.output = -0.05 * control.output;
            }
        
            mecanum.setPowers(-control.output,control.output,-control.output,control.output);
            StaticVars.telemetry.addData("calculated","%5.2f",control.output );
            StaticVars.telemetry.addData("Angle","%5.2f",value );
                              

    }}

    
    double getDegree(AngleUnit angleUnit, double angle){
     return AngleUnit.DEGREES.fromUnit(angleUnit, angle);   
    }


}
