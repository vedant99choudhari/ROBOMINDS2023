package org.firstinspires.ftc.teamcode;

import android.util.Size;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "cameraTest1 (Blocks to Java)")
public class cameraTest1 extends LinearOpMode {

  /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    VisionPortal.Builder myVisionPortalBuilder;
    boolean USE_WEBCAM;
    int RESOLUTION_WIDTH;
    int RESOLUTION_HEIGHT;
    VisionPortal myVisionPortal;
    boolean lastX;
    int frameCount;
    long capReqTime;
    boolean x;

    
    RESOLUTION_WIDTH = 160;
    RESOLUTION_HEIGHT = 120;
    // Create a VisionPortal.Builder and set attributes related to the camera.
    myVisionPortalBuilder = new VisionPortal.Builder();
    myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
    myVisionPortalBuilder.setCameraResolution(new Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT));
    // CustomProcessor customProcessor = new CustomProcessor(telemetry);
    MatDetect detection = new MatDetect();
    myVisionPortalBuilder.addProcessor(detection);
    // Create a VisionPortal by calling build.
    myVisionPortal = myVisionPortalBuilder.build();
    
    while (!isStopRequested()) {
      telemetry.addData("object detected", detection.count >1000);
      telemetry.update();
    }
  }
}
