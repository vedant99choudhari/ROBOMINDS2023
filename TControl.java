
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.Vector;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import org.firstinspires.ftc.teamcode.Drive;

@TeleOp(name = "Manual:Telescopic", group = "Operator")

public class TControl extends LinearOpMode {
    Drive mecanum;
    Extenders red,black;
    ElapsedTime clock;
    Vector <Double> data_log ;
    double last_log;
    boolean state;
    double limit;

    
    
    @Override
    public void runOpMode() {

        // Connect to motor (Assume standard left wheel)
        // Change the text in quotes to match any motor name on your robot.
        String [] motor_config = {"front_right","front_left","back_right","back_left"};
        mecanum = new Drive(hardwareMap,motor_config);
        limit = 0.95;
        clock = new ElapsedTime();
        red = new Extenders(hardwareMap,"arm_2","servo",950);
        data_log = new Vector<Double>();
        state = true;
        black = new Extenders(hardwareMap,"arm","servo2",900);
        red.set_min_max(1,0.3);
        black.set_min_max(0.1,0.75);
        last_log = 0;
       
        
        
       
        DcMotorSimple.Direction left_direction = DcMotorSimple.Direction.REVERSE;
        red.lift.setDirection(left_direction); 
        
        red.grip_motion(true);
        red.grip_motion(false);
        red.grip_motion(true);
        black.grip_motion(true);
        black.grip_motion(false);
        black.grip_motion(true);
        
       
        // Wait for the start button
        telemetry.addData(">", "Press Start to run." );
        
        telemetry.update();
        
        waitForStart();
        
    
    
        while(opModeIsActive())
        {
          check();
          move();
          
          record(this.gamepad1.b);
         
          if(this.gamepad1.y)
          {
                for(int x =0 ;x<data_log.size() ; x++)
                   telemetry.addData("value","%5.2f",data_log.get(x));
        
                telemetry.update(); 
          }
          
        }
        
        
       
        mecanum.set(0,0,0,0);

    }
    
    void move()
    {
            double speed_y =- limit*this.gamepad1.left_stick_y;
            double speed_x = limit *this.gamepad1.left_stick_x;
            double speed_rotate = this.gamepad1.right_stick_x/2;
            
            if(this.gamepad1.right_trigger > 0.3)
            {
                speed_y = speed_y/2;
                speed_x = speed_x/2;
                speed_rotate = speed_rotate /2;
            }
            else if(this.gamepad1.right_bumper)
            {
                speed_y = speed_y/3;
                speed_x = speed_x/3;
                speed_rotate = speed_rotate /3;
            }

            double angle = Math.toDegrees( Math.atan2(speed_y,speed_x));
            double speed = Math.sqrt(2) * Math.sqrt((speed_y*speed_y) + (speed_x *speed_x));

            double diagonal1 = -speed * Math.sin(Math.toRadians(angle+45));
            double diagonal2 = speed * Math.cos(Math.toRadians(angle+45));
            
            
            if(Math.abs(speed_rotate) > 0.1)
            {
            mecanum.set(speed_rotate,-speed_rotate,speed_rotate,-speed_rotate); 
            }
            else
            {
                mecanum.set(diagonal2, diagonal1, diagonal1, diagonal2);
            }
            red.move();
            black.move();
            

    }
  
    void check()
    {
        int value = 50;
        // telemetry.addData("the red value is","%d",black.target);
        // telemetry.addData("the black value is","%d",red.target);
        // telemetry.addData("the red position is","%d",black.print());
        // telemetry.addData("the black position is","%d",red.print());
        // telemetry.update();
        
        if(this.gamepad2.left_trigger > 0.0)
         {
          
           red.change(value);
         }
        if(this.gamepad2.left_bumper)
        {
            red.change(-value);
        }
        
        if(this.gamepad2.right_trigger > 0.0)
        {
            black.change(value);
        }
        
        if(this.gamepad2.right_bumper)
        {
            black.change(-value);
        }
        
        red.grip_motion(this.gamepad2.dpad_down);
        
        black.grip_motion(this.gamepad2.a);
        
        
        black.lift_motion(this.gamepad2.x);
        red.lift_motion(this.gamepad2.x);
        
        black.lift_max(this.gamepad2.y);
        red.lift_max(this.gamepad2.y);
        
        
        if(this.gamepad2.right_stick_y > 0.4)
        {
                 black.decrement(1);
        }
        
        
        if(this.gamepad2.right_stick_y < -0.4)
        {
           
                 black.increment(1);
        }
        
         if(this.gamepad2.left_stick_y > 0.4)
        {
            
                 red.decrement(1);
        }
        
        
        if(this.gamepad2.left_stick_y < - 0.4)
        {
                 red.increment(1);
        }
        
        if(this.gamepad2.right_stick_button)
        {
             black.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
             black.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
             black.target =0;
        }
        
        if(this.gamepad2.left_stick_button)
        {
             red.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
             red.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
             red.target =0;
        }
        
    }
    
    void record(boolean input)
    {
        
        if (input)
        {
            if(state)
            {
                return;
            }
            else
            {
                if(clock.seconds()-last_log > 1)
                {
                state = true;
                last_log = clock.seconds();
                 data_log.addElement(last_log);
                }
            }
        }
        else
        {
            state =false;
            return;
        }
    }
}
