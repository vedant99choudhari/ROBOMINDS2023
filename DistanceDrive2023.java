// package org.firstinspires.ftc.teamcode;
// 
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.PIDCoefficients;
// import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
// import com.qualcomm.robotcore.hardware.HardwareMap;
// import com.qualcomm.robotcore.hardware.DcMotorImplEx;
// 
// 
// import org.firstinspires.ftc.teamcode.Drive2023;
// import org.firstinspires.ftc.teamcode.Gyro2023;
// import com.qualcomm.robotcore.hardware.HardwareDevice;
// 
// import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
// import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
// import com.qualcomm.hardware.bosch.BNO055IMU;
// 
// public class DistanceDrive2023 {
//     Gyro2023 gyro;
//     Drive2023 mecanum;
//     public DistanceDrive2023(Drive2023 mecanum_input, Gyro2023 gyro_input){
//        mecanum = mecanum_input;
//        gyro = gyro_input;
//     }
//     /**
//      * Moves the robot in the x and y direction
//      *
//      * @param  x_in      the x value in inches
//      * @param  y_in      the y alue in inches
//      * @param  timedFailSafe     whether to have a time failsafe
//      */
//     public void moveTill(double x_in, double y_in, boolean timedFailSafe){
//         double x_enc = x_in * 58;
//         double y_enc = y_in * 58;
//         double power = 0.5;
//         mecanum.setMode(1);
//         double displacement = setTargets(x_enc,y_enc);
//         double time = displacement / 17;
//         mecanum.setMode(2);
//         mecanum.setALL(power);
//         //TODO: Implement must()
//         //must() for backboard failsafe?
//         //now implementation
//         while (mecanum.isMotorsBusy() && must() && (clock.seconds()-now)<time)
//         {
//             telemetry.addData("In locking" , 0);          
//             telemetry.update();
//             
//             double value = gyro.getOrientation();
//             
//             if(Math.abs(value) >4)
//             
//                 mecanum.setALL(0);
//                 mecanum.setMode(3);
//                 gyro.angularLock();
//                 setTargets(x_enc,y_enc);
//                 mecanum.setMode(2);
//                 mecanum.setALL(power);
//         }
//         mecanum.setAll(0); 
//         mecanum.setMode(3);
//     }
//     double setTargets(int target_x , int target_y)
//     {
//         if(field == "RED")
//          {
//              target_x = -target_x;
//          }
//         double angle = Math.toDegrees( Math.atan2(target_y,target_x));
//         double target =Math.sqrt(2) * Math.sqrt((target_x*target_x) + (target_y *target_y));
//         double diagonal1 = - target * Math.sin(Math.toRadians(angle+45));
//         double diagonal2 = target * Math.cos(Math.toRadians(angle+45));
//         
//         int d1 = (int) Math.round(diagonal1);
//         int d2 = (int) Math.round(diagonal2);
//         //TODO : Integrate into Drive2023
//         mecanum.motors[0].setTargetPosition(d2);
//         mecanum.motors[1].setTargetPosition(d1);
//         mecanum.motors[2].setTargetPosition(d1);
//         mecanum.motors[3].setTargetPosition(d2);
//         
//         
//         return Math.sqrt(target/Math.sqrt(2));
//     }
// 
// }
// 