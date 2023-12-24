package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.Servo;
import java.lang.annotation.Target;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import org.firstinspires.ftc.teamcode.Drive;

@Autonomous(name = "Loop", group = "Auto")

public class Test_loop extends LinearOpMode {
    Drive mecanum;
    ElapsedTime clock;
    Proximity sensor_1,sensor_2;
    Telescopic red,black;
    DistanceSensor distance,distance2;
    double color;
    double color22;
    String field;
    DigitalChannel digitalTouch,digitalTouch2;  
    
    Pid control;
    double TargetAngle;
    double value;
    BNO055IMU imu;
    Orientation angles;


    @Override
    public void runOpMode() {
        field = "BLUE";
        if(field == "BLUE")
        {
        
        red = new Telescopic(hardwareMap,"arm","servo2");
        red.set_min_max(0.2,0.75);
        
        black = new Telescopic(hardwareMap,"arm_2","servo");
        black.set_min_max(1,0.35);
        
        distance  = hardwareMap.get(DistanceSensor.class,"distance"); 
        distance2 = hardwareMap.get(DistanceSensor.class,"distance2");
        
            black.lift.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else
        {
          
        
        red = new Telescopic(hardwareMap,"arm_2","servo");
        
        
        black = new Telescopic(hardwareMap,"arm","servo2");
        red.set_min_max(1,0.35);
        black.set_min_max(0.2,0.75);
        distance = hardwareMap.get(DistanceSensor.class,"distance2"); 
        distance2 = hardwareMap.get(DistanceSensor.class,"distance");    
            red.lift.setDirection(DcMotorSimple.Direction.REVERSE); 
        }
        
        String [] motor_config = {"front_right","front_left","back_right","back_left"};
        mecanum = new Drive(hardwareMap,motor_config);
    
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        control = new Pid();
        control.set(0.3,0.1,0.6);
        
        
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        TargetAngle = 0.00;
        
        
         
        
        
        
        
        clock = new ElapsedTime();
        
        
    
        // Wait for the start button
        telemetry.addData(">", "Press Start to run." );
        telemetry.update();
        
       
        waitForStart();
        
        
    
    
        boolean flag =false;
        
    flag=true;
    //  moveTill_Proximity();  
        
        if(must() && flag)
        {   
            
            
             moveTill_distance2(40,1);
             
           flag =false; 
        }
        
    }
      
    void moveTill_Proximity()
    {
        while(must())
        {
            telemetry.addData("Proximity","%5.2f" ,sensor_1.get_distance());
            telemetry.addData("Proximity 2" ,"%5.2f",sensor_2.get_distance());
            telemetry.update();
            
             angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
           value = getdegree(angles.angleUnit, angles.firstAngle);
           
           if(Math.abs(value) >4)
            {
               continuous_a();
            }
            
            set_power(-0.4);
            
            
            
            if(sensor_1.get_distance() < 6 || sensor_1.get_distance() < 6 )
            {
               break; 
            }
        }
    }
    
    void moveTill_touch()
    {
        double start = clock.seconds();
        while ( must()) 
        {
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
           value = getdegree(angles.angleUnit, angles.firstAngle);
           
           if(Math.abs(value) >4 )
            {
               continuous_a();
               if(clock.seconds() - start > 2)
               {
                   angular_lock();
                   start = clock.seconds();
               }
            }
            
            set_power(0.7);
            
           if (!digitalTouch.getState() || !digitalTouch2.getState())
              break;
        }
        
        set_power(0);
    }
    
    void wait(double time)
    {
         double now = clock.seconds();
            
            while(clock.seconds()-now < time && must())
            {
                
            }
    }
    
    
    void lift()
    {
       red.increment(100);
       black.increment(100);  
    }
    
    void drop()
    {
         red.decrement(101);
         black.decrement(101);
            
           
            
    }
    
    
    boolean must()
    {
        red.move();
        black.move();
        
        
        return opModeIsActive();
    }
     public void move_across(double target, int direction)
    {
        double d1 = 0;
        double d2 = 0;
        
        angular_lock();
        
         if(field == "RED")
        {
          if(direction == 0)
           direction =1;
          else 
           direction = 0;
        }
        
        
        if (direction==0)
        {
          d1 = -1;
          d2 =1;
        }
        else
        {
           d1 = 1;
           d2 = -1;
        }
        
        double reading1 = distance.getDistance(DistanceUnit.INCH);
        double BaseSpeed = 0.1;
        double MinSpeed = 0.15;
        double MaxSpeed = 0.6;
        double buffer = 0.3;
        double range = 3;
        double speed = 0;
        
        telemetry.addData("distance","%5.2f",reading1);
        telemetry.update();
                
                
        while( Math.abs(reading1-target)>buffer && must() && reading1<target)
        {
           angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
           value = getdegree(angles.angleUnit, angles.firstAngle);
           
           if(Math.abs(value) >4)
            {
               continuous_a();
            }
            reading1 = distance.getDistance(DistanceUnit.INCH);
            
            if(Math.abs(reading1 - target) >range)
            {
                BaseSpeed = MaxSpeed;
            }
            
            else 
            {
             BaseSpeed = 0.01 * Math.abs(reading1 - target);
             if(BaseSpeed < MinSpeed)
             {
                 BaseSpeed = MinSpeed;
             }
            }
            
            if (reading1 >target)
            {
                speed = -1*BaseSpeed;
            }
            else
            {
              speed = 1 * BaseSpeed;
            }
               
            mecanum.set(d1*speed,d2*speed,d2*speed,d1*speed);   
            
            telemetry.addData("distance","%5.2f",reading1);
            telemetry.update();
            
            
        }     
            
        reading1 = distance.getDistance(DistanceUnit.INCH);
        
        mecanum.set(0,0,0,0);
        
        
    }
    
    
    
    void angular_lock()
    {
     do
        {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            value = getdegree(angles.angleUnit, angles.firstAngle);
            continuous_a();
            telemetry.update();
             
            
        }while(Math.abs(value-TargetAngle) > 2 && must());
            
      mecanum.set(0,0,0,0);
    }
    
    void continuous_a()
    {
         if(control.compute(value,TargetAngle,clock.seconds()))
            {
               if(Math.abs(TargetAngle- value)> 10)
                control.output = 0.4 * control.output;
                else
                control.output = 0.25 * control.output;
            }
            mecanum.motors[1].setPower(control.output);
            mecanum.motors[3].setPower(control.output);
                
            mecanum.motors[0].setPower(-control.output);
            mecanum.motors[2].setPower(-control.output);
            telemetry.addData("calculated","%5.2f",control.output );
            telemetry.addData("Angle","%5.2f",value );
                              

    }
    public void moveTill_distance2(double target, int direction)
    {
        double d1 = 0;
        double d2 = 0;
        
        angular_lock();
        
         if(field == "RED")
        {
          if(direction == 0)
           direction =1;
          else 
           direction = 0;
        }
        
        
        if (direction==0)
        {
          d1 = 1;
          d2 =-1;
        }
        else
        {
           d1 = -1;
           d2 = 1;
        }
        
        double reading1 = distance2.getDistance(DistanceUnit.INCH);
        double BaseSpeed = 0.1;
        double MinSpeed = 0.15;
        double MaxSpeed = 0.6;
        double buffer = 0.3;
        double range = 3;
        double speed = 0;
        
        telemetry.addData("distance","%5.2f",reading1);
        telemetry.update();
                
                
        while( Math.abs(reading1-target)>buffer && must())
        {
           angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
           value = getdegree(angles.angleUnit, angles.firstAngle);
           
           if(Math.abs(value) >4)
            {
               continuous_a();
            }
            reading1 = distance2.getDistance(DistanceUnit.INCH);
            
            if(Math.abs(reading1 - target) >range)
            {
                BaseSpeed = MaxSpeed;
            }
            
            else 
            {
             BaseSpeed = 0.01 * Math.abs(reading1 - target);
             if(BaseSpeed < MinSpeed)
             {
                 BaseSpeed = MinSpeed;
             }
            }
            
            if (reading1 >target)
            {
                speed = -1*BaseSpeed;
            }
            else
            {
              speed = 1 * BaseSpeed;
            }
               
            mecanum.set(d1*speed,d2*speed,d2*speed,d1*speed);   
            
            telemetry.addData("distance","%5.2f",reading1);
            telemetry.update();
            
            
        }     
            
       
        
        mecanum.set(0,0,0,0);
        
        
    }
     public void moveTill_distance(double target, int direction)
    {
        double d1 = 0;
        double d2 = 0;
        
        angular_lock();
        
         if(field == "RED")
        {
          if(direction == 0)
           direction =1;
          else 
           direction = 0;
        }
        
        
        if (direction==0)
        {
          d1 = 1;
          d2 =-1;
        }
        else
        {
           d1 = -1;
           d2 = 1;
        }
        
        double reading1 = distance.getDistance(DistanceUnit.INCH);
        double BaseSpeed = 0.1;
        double MinSpeed = 0.15;
        double MaxSpeed = 0.6;
        double buffer = 0.3;
        double range = 3;
        double speed = 0;
        
        telemetry.addData("distance","%5.2f",reading1);
        telemetry.update();
                
                
        while( Math.abs(reading1-target)>buffer && must())
        {
           angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
           value = getdegree(angles.angleUnit, angles.firstAngle);
           
           if(Math.abs(value) >4)
            {
               continuous_a();
            }
            reading1 = distance.getDistance(DistanceUnit.INCH);
            
            if(Math.abs(reading1 - target) >range)
            {
                BaseSpeed = MaxSpeed;
            }
            
            else 
            {
             BaseSpeed = 0.01 * Math.abs(reading1 - target);
             if(BaseSpeed < MinSpeed)
             {
                 BaseSpeed = MinSpeed;
             }
            }
            
            if (reading1 >target)
            {
                speed = -1*BaseSpeed;
            }
            else
            {
              speed = 1 * BaseSpeed;
            }
               
            mecanum.set(d1*speed,d2*speed,d2*speed,d1*speed);   
            
            telemetry.addData("distance","%5.2f",reading1);
            telemetry.update();
            
            
        }     
            
        reading1 = distance.getDistance(DistanceUnit.INCH);
        
        mecanum.set(0,0,0,0);
        
        
    }
    
     double set(int target_x , int target_y)
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
        
        mecanum.motors[0].setTargetPosition(d2);
        mecanum.motors[1].setTargetPosition(d1);
        mecanum.motors[2].setTargetPosition(d1);
        mecanum.motors[3].setTargetPosition(d2);
        
        
        return Math.sqrt(target/Math.sqrt(2));
    }
    
    
    
   
    
    double getdegree(AngleUnit angleUnit, double angle)
    {
     return AngleUnit.DEGREES.fromUnit(angleUnit, angle);   
    }
    
    void moveTill(double target_x, double target_y )
    {
      mecanum.mode(1);
      double pwr = 0.5;
      double now = clock.seconds(); 
      int targetx = (int)Math.round(target_x * 58);
      int targety = (int)Math.round(target_y * 58);
      double inch = set(targetx,targety);
      double time = inch / 17;
      mecanum.mode(2);
      set_power(pwr);
      
      while (mecanum.is_busy() && must() && (clock.seconds()-now)<time )
        {
            telemetry.addData("In locking" , 0);          
            telemetry.update();
           
           angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
           value = getdegree(angles.angleUnit, angles.firstAngle);
           
           if(Math.abs(value) >4)
            {
                set_power(0);
                mecanum.mode(3);
                angular_lock();
                set(targetx,targety);
                mecanum.mode(2);
                set_power(pwr);
            }
        }
      
      
        
       mecanum.set(0,0,0,0); 
      mecanum.mode(3);
      
    }
    
    
    void moveTill(double target_x, double target_y ,boolean timed)
    {
      mecanum.mode(1);
      double pwr = 0.5;
      double now = clock.seconds(); 
      int targetx = (int)Math.round(target_x * 58);
      int targety = (int)Math.round(target_y * 58);
      double inch = set(targetx,targety);
      
      double time;
      if(timed)
      time = inch / 17;
      else
      time = inch ;
      mecanum.mode(2);
      set_power(pwr);
      
      while (mecanum.is_busy() && must() && (clock.seconds()-now)<time )
        {
            telemetry.addData("In locking" , 0);          
            telemetry.update();
           
           angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
           value = getdegree(angles.angleUnit, angles.firstAngle);
           
           if(Math.abs(value) >4)
            {
                set_power(0);
                mecanum.mode(3);
                angular_lock();
                set(targetx,targety);
                mecanum.mode(2);
                set_power(pwr);
            }
        }
      
     
        
       mecanum.set(0,0,0,0); 
      mecanum.mode(3);
      
    }

    void set_power(double speed)
    {
       for(int num = 0;num<4; num = num +1)
        {
           mecanum.motors[num].setPower(speed);
        } 
    }
    
    
    double get_block (Proximity color)
    {
        return (color.red() * color.green()) / Math.pow(color.blue(), 2);
    }
   
};
