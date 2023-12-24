package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Objects;

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
        double now = StaticVars.clock.seconds();
        double power = 0.5;

        mecanum.setMotorMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        double displacement = setTargets(x_enc,y_enc);
        double time =timedFailSafe ? displacement / 140 : 2;
        // StaticVars.telemetry.speak(String.format("starting movetill, time required is %f",time));
        mecanum.setMotorMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        mecanum.setAllPower(power);
        //TODO: Implement must()
        //must() for backboard failsafe?
        //now implementation
        while (mecanum.isMotorsBusy() && ((Main)opMode).must() && ((StaticVars.clock.seconds()-now)<time))
        {

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
        double MinSpeed = 0.07;
        double MaxSpeed = 0.15;
        double buffer = 0.3;
        double range = 3;
        double speed = 0;

        opMode.telemetry.addData("distance",reading1);
        opMode.telemetry.update();

        double value;
        while( Math.abs(reading1-target)>buffer && ((Main)opMode).must()){
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
        opMode.telemetry.addData("distance",reading1);
        mecanum.setPowers(0,0,0,0);


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
    public void moveTillApriltag(double target, int direction,int tag, AprilTagProcessor processor){
        double d1 = 0;
        double d2 = 0;

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

        Double distanceX = getXDistance(tag, processor);

        if (distanceX == Double.NaN)
            return;

        double reading1 = distanceX;
        double BaseSpeed = 0.1;
        double MinSpeed = 0.03;
        double MaxSpeed = 0.04;
        double buffer = 0.3;
        double range = 3;
        double speed = 0;

        opMode.telemetry.addData("distance",reading1);
        opMode.telemetry.update();

        double value;
        while( Math.abs(reading1-target)>buffer && ((Main)opMode).must()){
            value = gyro.getOrientation();

            if(Math.abs(value) >4){
                gyro.continousA();
            }
            
            reading1 = getXDistance(tag, processor);
            
            if (reading1 == Double.NaN) {
                mecanum.setPowers(0,0,0,0);
                return;
            }
            opMode.telemetry.addData("distance",reading1);
            

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

        reading1 = getXDistance(tag, processor);
        //opMode.telemetry.addData("distance",reading1);
        mecanum.setPowers(0,0,0,0);


    }


    private static Double getXDistance(int tag, AprilTagProcessor processor) {
        ArrayList<AprilTagDetection> detections = processor.getDetections();
        for (AprilTagDetection detection : detections) {
            if(detection.metadata != null){
                if(detection.id == tag){
                    return detection.ftcPose.x;
                }
            }
        }
        return -1d;
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


        return target/Math.sqrt(2);
    }

}
