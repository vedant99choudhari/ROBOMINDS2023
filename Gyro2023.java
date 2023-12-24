package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.teamcode.Pid2023;
public class Gyro2023 {
    double TargetAngle = 0.0;
    double value;
    BNO055IMU imu;
    Orientation angles;
    Telemetry telemetry  = StaticVars.telemetry;
    Drive2023 mecanum;
    Pid2023 control = new Pid2023();
    public Gyro2023(HardwareMap hardwareMap, Drive2023 drive){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        mecanum = drive;
    }
    //TODO : setmode in anuglar lock?
    void angularLock()
    {
     do
        {
            value = getOrientation();
            continousA();
            telemetry.update();
             
            
        }while(Math.abs(value-TargetAngle) > 2 && must());
            
      mecanum.setPowers(0,0,0,0);
    }
    public double getOrientation() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        value = getDegree(angles.angleUnit, angles.firstAngle);
        return value;
    }
    
    void continousA()
    {
         if(control.compute(value,TargetAngle,clock.seconds()))
            {
               if(Math.abs(TargetAngle- value)> 10)
                control.output = 0.4 * control.output;
                else
                control.output = 0.25 * control.output;
            }
            mecanum.setPowers(-control.output,control.output,-control.output,control.output);
            telemetry.addData("calculated","%5.2f",control.output );
            telemetry.addData("Angle","%5.2f",value );
                              

    }

    
    double getDegree(AngleUnit angleUnit, double angle)
    {
     return AngleUnit.DEGREES.fromUnit(angleUnit, angle);   
    }


}
