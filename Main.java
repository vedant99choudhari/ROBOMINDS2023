package org.firstinspires.ftc.teamcode;

import android.util.Size;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import java.util.List;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

@Autonomous

public class Main extends LinearOpMode {
    private static final int RESOLUTION_WIDTH = 640;
    private static final int RESOLUTION_HEIGHT = 480;
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
    private DistanceSensor distanceFR;
    private DistanceSensor distanceL;
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
    int arm1_target = 0;
    int arm2_target= 0;
    // todo: write your code here
    DriveFunctions movement;
    private AprilTagProcessor myAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

    public void runOpMode(){



        arm1 = hardwareMap.get(DcMotorEx.class, "Arm1");
        arm2 = hardwareMap.get(DcMotorEx.class, "Arm2");
        ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        right = hardwareMap.get(Servo.class, "right");
        arm3 = hardwareMap.get(Servo.class, "arm3");
        left = hardwareMap.get(Servo.class, "left");

        String [] motorConfig = {"FR","FL","BR","BL"};
        // TODO: 15-12-2023 remove the init code from drive
        mecanum = new Drive2023(hardwareMap, motorConfig);
        StaticVars vars = new StaticVars(telemetry,"RED");
        mecanum.setMotorMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        mecanum.setMotorMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        distanceFR = hardwareMap.get(DistanceSensor.class,"FRDISTANCE");
        distanceFL = hardwareMap.get(DistanceSensor.class,"FLDISTANCE");
        distanceB = hardwareMap.get(DistanceSensor.class,"REARDISTANCE");
        distanceR = hardwareMap.get(DistanceSensor.class,"DISTANCER");
        distanceL = hardwareMap.get(DistanceSensor.class,"DISTANCEL");


        right.setPosition(0);
        left.setPosition(0.9);


        initCamera();

        waitForStart();
        gyro = new Gyro2023(hardwareMap, mecanum);
        gyro.reset();


        movement = new DriveFunctions(mecanum,gyro,this,hardwareMap);
        boolean flag = true;
        gyro.TargetAngle = 0.0;
        while((opModeIsActive() && flag)){
            arm1.setTargetPosition(arm1_target);
            arm2.setTargetPosition(arm2_target);
            arm1.setPower(getPowerForVoltage(0.95));
            arm2.setPower(getPowerForVoltage(0.95));
            arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm3.setPosition(0.2);

            TeamElement element = TeamElement.CENTER;
            
            if(element == TeamElement.RIGHT)
              movement.moveTill(24d,0d,true);  
            
            movement.moveTill(0d,30d,true);
            if(element == TeamElement.LEFT ||element == TeamElement.RIGHT ){
                gyro.TargetAngle =90.0;
                lock();

                arm3.setPosition(0.4);
                movement.moveTill(0d,2d,false);
                left.setPosition(0.3);

                movement.moveTill(0d,-10d,false);
            }else if(element == TeamElement.CENTER){
                arm3.setPosition(0.4);
                
                left.setPosition(0.3);
                //movement.moveTill(3d,0d,false);
            }

           
            gyro.TargetAngle =-90.0;
            lock();
            
           
            
            switch(element){
                case LEFT:
                    movement.moveTillApriltag(1.5,3,5,myAprilTagProcessor);
                    break;
                case CENTER:
                    movement.moveTillApriltag(1.5,3,6,myAprilTagProcessor);
                    break;
                default:
                    movement.moveTillApriltag(-1.5,1,5,myAprilTagProcessor);
                    break;
            }
            
            
            arm2.setTargetPosition(165); 
            movement.moveTillDistance(8,0,distanceFR);
            arm3.setPosition(0.6);
            wait(1000000000);
            right.setPosition(0.6);





            flag = false;

        }
        

        while(must()){
          
             
            break;


        }
    }

    private void initCamera() {
        // Create a VisionPortal.Builder and set attributes related to the camera.
        VisionPortal.Builder myVisionPortalBuilder = new VisionPortal.Builder();
        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        myVisionPortalBuilder.setCameraResolution(new Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT));

        myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
        // Create a VisionPortal by calling build.
        VisionPortal myVisionPortal = myVisionPortalBuilder.build();
    }

    public double getAverageDistance(DistanceSensor sensor){
        double sum = 0;
        for(int i = 1; i<=10; i++){
            sum = sum + sensor.getDistance(DistanceUnit.INCH);
        }
        return sum/10;
    }

    public void wait(int seconds){
        long now = StaticVars.clock.now(TimeUnit.NANOSECONDS);
        while((StaticVars.clock.now(TimeUnit.NANOSECONDS) - now < seconds) && must()){
            telemetry.addData("Elapsed Time",(StaticVars.clock.now(TimeUnit.SECONDS) - now));
            telemetry.addData("Arm2 Encoder: ", arm2.getCurrentPosition());
        }
    }
    /**
     * Moves the robot in the x and y direction
     *
     * @param  x_in      the x value in inches
     * @param  y_in      the y value in inches
     * @param  timedFailSafe     whether to have a time failsafe
     */
    double getPowerForVoltage(double voltage){
        double power = voltage/ControlHub_VoltageSensor.getVoltage();
        telemetry.addData("Motor Power:" , power);
        return power;
    }
    
     /**
   * Display info (using telemetry) for a recognized AprilTag.
   */
  private void telemetryAprilTag() {
    List<AprilTagDetection> myAprilTagDetections;
    AprilTagDetection myAprilTagDetection;

    // Get a list of AprilTag detections.
    myAprilTagDetections = myAprilTagProcessor.getDetections();
    telemetry.addData("# AprilTags Detected", JavaUtil.listLength(myAprilTagDetections));
    // Iterate through list and call a function to display info for each recognized AprilTag.
    for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
      myAprilTagDetection = myAprilTagDetection_item;
      // Display info about the detection.
      telemetry.addLine("");
      if (myAprilTagDetection.metadata != null) {
        telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") " + myAprilTagDetection.metadata.name);
        telemetry.addLine("XYZ " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.x, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.y, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.z, 6, 1) + "  (inch)");
        telemetry.addLine("PRY " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.pitch, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.roll, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.yaw, 6, 1) + "  (deg)");
        telemetry.addLine("RBE " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.range, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.bearing, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.elevation, 6, 1) + "  (inch, deg, deg)");
      } else {
        telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") Unknown");
        telemetry.addLine("Center " + JavaUtil.formatNumber(myAprilTagDetection.center.x, 6, 0) + "" + JavaUtil.formatNumber(myAprilTagDetection.center.y, 6, 0) + " (pixels)");
      }
    }
    telemetry.addLine("");
    telemetry.addLine("key:");
    telemetry.addLine("XYZ = X (Right), Y (Forward), Z (Up) dist.");
    telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
    telemetry.addLine("RBE = Range, Bearing & Elevation");
  }

    public boolean must(){
        /*telemetry.addData("FR target", mecanum.motors[0].getTargetPosition());
        telemetry.addData("FR position", mecanum.motors[0].getCurrentPosition());
        telemetry.addData("FL target", mecanum.motors[1].getTargetPosition());
        telemetry.addData("FL position", mecanum.motors[1].getCurrentPosition());
        telemetry.addData("BR target", mecanum.motors[2].getTargetPosition());
        telemetry.addData("BR position", mecanum.motors[2].getCurrentPosition());
        telemetry.addData("BL target", mecanum.motors[3].getTargetPosition());
        telemetry.addData("BL position", mecanum.motors[3].getCurrentPosition());*/
        telemetryAprilTag();
        telemetry.update();
        return opModeIsActive();
    }


    void lock(){

        do{

        }while(gyro.angularLock() && opModeIsActive());
        mecanum.setAllPower(0.0);
    }


}
    
