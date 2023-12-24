package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.TeamElement.CENTER;
import static org.firstinspires.ftc.teamcode.TeamElement.LEFT;
import static org.firstinspires.ftc.teamcode.TeamElement.RIGHT;

import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;


public class MatDetect implements VisionProcessor{

  public long count;
  public int centerCount;
  public int rightCount;

  public TeamElement getPosition(){
    if(centerCount > 30){
      return CENTER;
    }

    if(rightCount > 30){
      return RIGHT;
    }

    return LEFT;
  }


  public java.lang.Object processFrame(org.opencv.core.Mat image,long captureTime){

    count = 0;
    centerCount = getPixelCountFor(image,CENTER);
    rightCount = getPixelCountFor(image, RIGHT);
    return image;

  }

  private int getPixelCountFor(Mat image, TeamElement area) {
    int count = 0;
    for(int rowIndex = area.topX; rowIndex< area.bottomX;rowIndex++){
      for(int columnIndex = area.topY; columnIndex< area.bottomY; columnIndex++){
        Scalar color = new Scalar(image.get(columnIndex, rowIndex));
        double blueColor = color.val[2];
        double greenColor = color.val[1];
        double redColor = color.val[0];
        if(redColor > 10 && (greenColor<100) && (blueColor<100) ){
          count = count +1;
        }
      }
    }
    return count;
  }


  public void init(int width,int height ,org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration calibration){

  }

  public void onDrawFrame(android.graphics.Canvas canvas,int width,int height,float scale,float density,java.lang.Object bmp){

    Paint paint = new Paint();
    paint.setStyle(Paint.Style.STROKE);
    paint.setColor(Color.WHITE);
    canvas.drawRect(CENTER.topX *scale, CENTER.topY*scale,CENTER.bottomX*scale,CENTER.bottomY *scale,paint);
    canvas.drawRect(RIGHT.topX *scale, RIGHT.topY*scale,RIGHT.bottomX*scale,RIGHT.bottomY*scale,paint);
  }
}