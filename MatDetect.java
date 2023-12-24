package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.android.*;
import org.opencv.imgproc.Imgproc;
import android.graphics.*;


public class MatDetect implements VisionProcessor{
  
  public long count;
  public long width;
  public long height;
  

  
  public java.lang.Object processFrame(org.opencv.core.Mat image,long captureTime){
    
    int rowCount = image.rows();
    int columnCount = image.cols();
   
    
    
    count = 0;
    for(int rowIndex = 0; rowIndex<rowCount;rowIndex++){
      for(int columnIndex = 0; columnIndex<columnCount; columnIndex++){
        Scalar color = new Scalar(image.get(rowIndex, columnIndex));
        double blueColor = color.val[2];
        double greenColor = color.val[1];
        double redColor = color.val[0];
        if(blueColor > 70 && blueColor<255 && (greenColor<100) && (redColor<100) ){
          count = count +1;
        }
      }
    }
    
   return image;
    
  }
  
  public void init(int width,int height ,org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration calibration){
    
  }

  public void onDrawFrame(android.graphics.Canvas canvas,int width,int height,float scale,float density,java.lang.Object bmp){
    
    Paint paint = new Paint();
    paint.setStyle(Paint.Style.STROKE);
    paint.setColor(Color.WHITE);
    canvas.drawRect(50 *scale, 35*scale,70*scale,55 *scale,paint);
    
    canvas.drawRect(110 *scale, 35*scale,130*scale,55 *scale,paint);
  }  
}