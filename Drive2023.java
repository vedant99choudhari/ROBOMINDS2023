package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import com.qualcomm.robotcore.hardware.HardwareDevice;


//Class for controlling the drive of the robot.
public class Drive2023 {
    // Array to store all the motors
    public DcMotorImplEx[] motors;

    /**
     * Constructor for Drive2023
     * 
     * Gets motor objects from hardwareMap by the keys given in config. Adds these objects to class variable motors for easy access.
     * @param hardwareMap
     * @param config
     */   
    public Drive2023(HardwareMap hardwareMap,String[] config)
    {
        motors = new DcMotorImplEx[4];
        for (int i=0; i<4; i++){
            motors[i]=hardwareMap.get(DcMotorImplEx.class, config[i]);
        }
        correctDirection();
        setPIDCoefficients();
        setMode(0);
    }
    /**
     * Sets the correct direction for the motors. (This is needed because the direction for the left hand motors are flipped tdue to thier placement in the hardware)
     *
     * @param  None
     * @return None
     */
    public void correctDirection() {
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[3].setDirection(DcMotorSimple.Direction.REVERSE);
     }

     /**
      * Sets the mode of the motors based on the provided integer value.
      *
      * @param  id  Internal Id for motor modes:
      *            0 - sets all motors to ZeroPowerBehavior
      *            1 - set sall motors to STOP_AND_RESET_ENCODER mode
      *            2 - sets all motors to RUN_TO_POSITION mode
      *            3 - sets all motors to RUN_USING_ENCODER mode
      */
     public void setMode(int id){
        DcMotor.RunMode modeToBeSet =  DcMotor.RunMode.STOP_AND_RESET_ENCODER;
        if(id==0){
            for(int i =0 ;i <4 ; i++)
            {
                motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
        }else{
            if(id==1){
                modeToBeSet = DcMotor.RunMode.STOP_AND_RESET_ENCODER;
            }
            else if(id==2){
                modeToBeSet = DcMotor.RunMode.RUN_TO_POSITION;
            }
            else if(id==3){
                modeToBeSet = DcMotor.RunMode.RUN_USING_ENCODER;
            }
            for(int i =0 ;i <4 ; i++)
            {
                motors[i].setMode(modeToBeSet);
            }

        }
    }
    
    /**
      * Sets the mode of the motors based on the provided integer value.
      *
      * @param  id  Internal Id for motor modes:
      *            0 - sets all motors to ZeroPowerBehavior
      *            1 - set sall motors to STOP_AND_RESET_ENCODER mode
      *            2 - sets all motors to RUN_TO_POSITION mode
      *            3 - sets all motors to RUN_USING_ENCODER mode
      */
     public void setMotorMode(DcMotor.RunMode mode){
        for(int i =0 ;i <4 ; i++)
        {
            motors[i].setMode(mode);
        }
    }
    
    
    /**
     * Initializes the PID controller and sets the PID coefficients(Kp,Ki,Kd) for all the motors.
     * These coefficients are used by rev's run_to_position motor mode
     *
     */
    public void setPIDCoefficients()
    {
        double kP = 0.3;
        double kI = 0.04;
        double kD = 0.7;
        PIDCoefficients coefficients = new PIDCoefficients(kP,kI,kD);
        for (int i=0;i<4;i++)
        {
        motors[i].setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION,coefficients );
        }    
    }
    /**
     * Sets the power of all motors to the specified value.
     * 
     * @param  power  the power to set for all motors
     */
    public void setAllPower(double power)
    {
        for (int i=0;i<4;i++)
        {
        motors[i].setPower(power);
        }    

    }
    
    /**
     * Sets the power of each individual motor in the robot.
     *
     * @param  fr  the power for the front right motor
     * @param  fl  the power for the front left motor
     * @param  br  the power for the back right motor
     * @param  bl  the power for the back left motor
     */
    public void setPowers(double fr,double fl,double bl,double br)
    {
        motors[0].setPower(fr);
        motors[1].setPower(fl);
        motors[2].setPower(bl);
        motors[3].setPower(br);
    }


    /**
     * Checks if any of the motors are currently busy.
     *
     * @return true if any of the motors are busy, false otherwise
     */
    boolean isMotorsBusy()
    {
        return motors[0].isBusy() ||motors[1].isBusy() ||motors[2].isBusy() ||motors[3].isBusy() ;
    }
    
}


