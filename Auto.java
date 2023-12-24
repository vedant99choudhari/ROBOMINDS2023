package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Autonomous(name = "SkyStones", group = "Auto")

public class Auto extends LinearOpMode {
    Drive mecanum;
    ElapsedTime clock;
    Proximity sensor_1,sensor_2;
    Telescopic red,black;
    DistanceSensor distance;
    String field;
    
    Pid control;
    double TargetAngle;
    double value;
    int skystone;
    BNO055IMU imu;
    Orientation angles;


    @Override
    public void runOpMode() {
        field = "BLUE";
        
        // Connect to motor (Assume standard left wheel)
        // Change the text in quotes to match any motor name on your robot.
        if(field == "BLUE")
        {
        
        sensor_1 = new Proximity("color",hardwareMap);
        sensor_2 = new Proximity("color2",hardwareMap);
        skystone = -1;
        
        red = new Telescopic(hardwareMap,"arm","servo2");
        red.set_min_max(0.2,0.75);
        
        black = new Telescopic(hardwareMap,"arm_2","servo");
        black.set_min_max(1,0.35);
        
        distance = hardwareMap.get(DistanceSensor.class,"distance"); 
        
            black.lift.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else
        {
           sensor_1 = new Proximity("color2",hardwareMap);
        sensor_2 = new Proximity("color",hardwareMap);
        skystone = -1;
        
        red = new Telescopic(hardwareMap,"arm_2","servo");
        
        
        black = new Telescopic(hardwareMap,"arm","servo2");
        red.set_min_max(1,0.35);
        black.set_min_max(0.2,0.75);
        distance = hardwareMap.get(DistanceSensor.class,"distance2"); 
            
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
        
        
        if(must() && flag)
        {   
            
            lift();
            moveTill(0,33,0.6);
            detect();
            if(skystone == 1)
            {
              drop();
              wait(0.5);
              black.grip_close();
            }
            else if(skystone == 2)
            {
              drop();
              wait(0.5);
              red.grip_close();
            }
            else
            {
              drop();
              wait(0.5);
              moveTill(10,0.5,0.5);
              
              angular_lock();
              red.grip_close();
            }
            wait(0.5);
        
            
             moveTill(-13,-13.5,0.9);
            
            
            move_across(45,0);
            
             
              moveTill(-30,0,0.4);
             
             angular_lock();
              red.grip_open();
             black.grip_open();
             
             moveTill(30,0);
             
             
             
            
            
            if(skystone != 3)
            {
                moveTill_distance(11,1);
            }
            else
            {
                moveTill_distance(5,1);
                moveTill(5,0);
            }

              moveTill(0,21,0.5);
             
             
             if(skystone == 1)
             {
              black.grip_close(); 
             }
             else if (skystone == 2)
             {
              
              red.grip_close();  
             }
             else
             {
              red.grip_close();
             }
             wait(0.5);
             
             
             
             moveTill(-15,-17,0.9);
            
             if (field == "RED")
                TargetAngle = -90;
             else
                TargetAngle = 90;
            
            
            angular_lock();
            if (skystone == 3)
                moveTill(0,70,1);
            else
                moveTill(0,60,1);
             
              red.grip_open();
             black.grip_open();
             
             
            moveTill(0,-15);
             
             
             
           
           flag =false; 
        }
        
        
       //  move_across(45,0);
        
        //moveTill_distance(11,1); 
        
        
        // while(must())
        // {
            
        // }
      
    

    }
    
    void detect()
    {
        double v1,v2;
        
        telemetry.addData("In the detect",0);
            telemetry.update();
        
            v1 = sensor_1.get_stone();
            v2 = sensor_2.get_stone();
            
            if(v1<2)
            {
              skystone = 1;
            }
            else if(v2 < 2)
            {
              skystone = 2;
            }
            else
            {
              skystone = 3;
            }
            
            telemetry.addData("In the detect",0);
            telemetry.update();
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
        double MinSpeed = 0.3;
        double MaxSpeed = 0.65;
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
        }while(Math.abs(value-TargetAngle) > 2 && must());
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
    }
    
     public void moveTill_distance(double target, int direction)
    {
        double d1 = 0;
        double d2 = 0;
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
        double MinSpeed = 0.3;
        double MaxSpeed = 0.5;
        double buffer = 0.3;
        double range = 3;
        double speed = 0;
        
        telemetry.addData("distance","%5.2f",reading1);
        telemetry.update();
                
                
        while( Math.abs(reading1-target)>buffer && must())
        {
           angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
           value = getdegree(angles.angleUnit, angles.firstAngle);
           
           if(Math.abs(value-TargetAngle) >4)
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
        double angle =Math.atan2(target_y,target_x) + Math.PI / 4;
        double magnitude = Math.sqrt((target_x*target_x) + (target_y *target_y));
        double target =Math.sqrt(2) * magnitude;
        int d1 = - (int) Math.round (target * Math.sin(angle));
        int d2 =   (int) Math.round (target * Math.cos(angle));
       
        mecanum.motors[0].setTargetPosition(d2);
        mecanum.motors[1].setTargetPosition(d1);
        mecanum.motors[2].setTargetPosition(d1);
        mecanum.motors[3].setTargetPosition(d2);
        
        return Math.sqrt(magnitude);
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
      while (mecanum.is_busy() && must() && (clock.seconds()-now)<time)
        {
           angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
           value = getdegree(angles.angleUnit, angles.firstAngle);
            
            
           if(Math.abs(value-TargetAngle) >4)
            {
                set_power(0);
                mecanum.mode(3);
                angular_lock();
                set(targetx,targety);
                mecanum.mode(2);
                set_power(pwr);
            }
        }
      
      set_power(0);
      mecanum.mode(3);
      
      
    }
    
        void moveTill(double target_x, double target_y ,double pwr)
    {
      mecanum.mode(1);
     
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
           
        //   if(Math.abs(value-TargetAngle) >5)
        //     {
        //         set_power(0);
        //         mecanum.mode(3);
        //         angular_lock();
        //         set(targetx,targety);
        //         mecanum.mode(2);
        //         set_power(pwr);
        //     }
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
   
};
