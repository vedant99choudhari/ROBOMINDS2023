/*
Copyright 2020 FIRST Tech Challenge Team 3101

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp()

public class Test extends OpMode {
    /* Declare OpMode members. */
    
        Telescopic red,black;
        Proximity color,color2;
        DistanceSensor distance;
        DigitalChannel digitalTouch,digitalTouch2;
    

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        red = new Telescopic(hardwareMap,"arm","servo");
        black = new Telescopic(hardwareMap,"arm","servo");
        color= new Proximity("color",hardwareMap);
        color2 = new Proximity("color2",hardwareMap);
        distance = hardwareMap.get(DistanceSensor.class, "distance");
        digitalTouch = hardwareMap.get(DigitalChannel.class, "touch");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        digitalTouch2 = hardwareMap.get(DigitalChannel.class, "touch2");
        digitalTouch2.setMode(DigitalChannel.Mode.INPUT);
        
        
        
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        
        
        
         red.flip();
      //   wait(0.5);
        // red.grip_motion(true);
        
       
        // black.grip_motion(true);
        // black.grip_motion(false);
        // black.grip_motion(true);
        lift();
        drop();
        telemetry.addData("color:","%5.2f" ,color.hue());
        telemetry.addData("color2:","%5.2f" ,color2.hue());
        telemetry.addData("distance:","%5.2f" ,distance.getDistance(DistanceUnit.INCH));
        telemetry.addData("Touch1:",digitalTouch.getState());
        telemetry.addData("Touch2:",digitalTouch2.getState());
        telemetry.update();
        
        
        
        
        
        
        
      
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

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
    //  boolean must()
    // {
        
    //     red.move();
    //     black.move();
    //     return opModeIsActive();
    // }
    //  void wait(double time)
    // {
    //      double now = clock.seconds();
            
    //         while(clock.seconds()-now < time && must())
    //         {
                
    //         }
    // }
    
}
