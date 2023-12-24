package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.android.*;
import org.opencv.imgproc.Imgproc;
import android.graphics.*;


public class CustomProcessor implements VisionProcessor{
  Telemetry telemetry;
  int executionCount = 0;
  long value;
  
  CustomProcessor(Telemetry telemetry){
    this.telemetry = telemetry;
  }
  
  public java.lang.Object processFrame(org.opencv.core.Mat image,long captureTime){
    org.opencv.core.Mat out = new org.opencv.core.Mat();
    Imgproc.cvtColor(image, out, Imgproc.COLOR_RGB2HSV);
    Core.inRange(out, new Scalar(90,0,0),new Scalar(150, 255, 255), out);
    
    Bitmap bmp=Bitmap.createBitmap(out.cols(),out.rows(),Bitmap.Config.ARGB_8888);
    Utils.matToBitmap(out , bmp, false);
    long count = 0;
    for(int rowIndex = 0; rowIndex<bmp.getWidth();rowIndex++){
      for(int columnIndex = 0; columnIndex<bmp.getHeight(); columnIndex++){
        int colour = bmp.getPixel(rowIndex , columnIndex);
        if(colour==-1){
          count++;
        }
      }
    }
    
    executionCount++;
    telemetry.addData("Execution count", executionCount);
    telemetry.addData("Execution pixelCount", count);
    telemetry.addData("Detected:", count>1000);
    telemetry.update();
    
    
    return bmp;
  }
  
  public void init(int width,int height ,org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration calibration){
    
  }

  public void onDrawFrame(android.graphics.Canvas canvas,int width,int height,float scale,float density,java.lang.Object bmp){
    
    
    
    Paint paint = new Paint();
    paint.setStyle(Paint.Style.FILL);
    paint.setColor(Color.WHITE);
    Bitmap bmp1= (Bitmap) bmp;
    canvas.drawBitmap(bmp1,0,0,paint);
  }  
}