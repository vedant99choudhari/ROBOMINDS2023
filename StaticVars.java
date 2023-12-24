package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.util.ElapsedTime;


public class StaticVars {
    public static Telemetry telemetry;
    public static ElapsedTime clock = new ElapsedTime();
    public static String field;
    StaticVars(Telemetry telemetry_t, String field_t){
        telemetry = telemetry_t;
        field = field_t;
    }
}