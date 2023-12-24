package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Pid2023 {
    
public double SampleTime,ki,kp,kd,outputSum,outMax,outMin,output,lastInput,lastTime;
Pid2023()
{
 SampleTime =0.004;
 ki=0.01;
 kp=0.07;
 kd=0.17;
 outputSum = 0;
 outMax=0.8;
 outMin=-0.8;
 output=0;
 lastInput = 0;
 lastTime=0;//get TIme;
 
}
void clear_error()
{
 outputSum =0;
}
 
 boolean compute(double input,double setpoint,double now)
{
   
   
   double timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
      
      double error = setpoint - input;
      double dInput = (input - lastInput);
      outputSum+= (ki * error);
      
     

      /*Add Proportional on Measurement*/
      outputSum-= kp * dInput;

      if(outputSum > outMax) outputSum= outMax;
      else if(outputSum < outMin) outputSum= outMin;

      /*Add Proportional on Error*/
     
      output = kp * error;
      

      /*Compute Rest of PID Output*/
      output += outputSum - kd * dInput;

        if(output > outMax) output = outMax;
      else if(output < outMin) output = outMin;
       

      /*Remember some variables for next time*/
      lastInput = input;
      lastTime = now;
        return true;
   }
   else return false;
}
void set(double p,double i ,double d)
{
  kp=p;
  ki=i;
  kd=d;
}
 
double absolute(double value)
{
 if(value<0)
  return -value;
 else
  return value;
}
}
