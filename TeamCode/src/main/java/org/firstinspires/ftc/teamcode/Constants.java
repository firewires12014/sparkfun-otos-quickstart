package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;

/** Central place for tunable/static constants. */
@Config
public final class Constants {
    private Constants() {
    }



    public static double LIFT1_UP = .2;
    public static double LIFT2_UP = .76;
    public static double LIFT1_DOWN = 1;
    public static double LIFT2_DOWN = 0;
    // Shooter PID/FF
    public static double SHOOTER_KP = 0.5;
    public static double SHOOTER_KD = 0.0;
    public static double SHOOTER_KV = 0.0004;


    // Shooter control
    public static int SHOOTER_VELOCITY = 1150;

    // Hood limits
    public static float HOOD_UPPER_LIMIT = 0.24f;
    public static float HOOD_MIDDLE_LIMIT = 0.12f;
    public static float HOOD_LOWER_LIMIT = 0.0f;

    // Define your range (180 degrees total = -90 to +90)
    public static double MAX_RAD = Math.toRadians(721.67);
    public static  double MIN_RAD = Math.toRadians(-695);
    public static double TICKS_PER_REVOLUTION = 145.1; // Example for 5203 Series motor
    public static double TURRET_GEAR_RATIO = 1.0;     // Ratio between motor and turret
    public static double TICKS_TO_RADIANS = (2 * Math.PI) / (TICKS_PER_REVOLUTION * TURRET_GEAR_RATIO);

    // Vision/auto offsets
    public static int BLUE_CLOSE_X_OFFSET = 12;
    public static int BLUE_CLOSE_Y_OFFSET = -14;

    public static int RED_CLOSE_X_OFFSET = 12;
    public static int RED_CLOSE_Y_OFFSET = 14;

    public static double TRIGGER_OPEN = 0.06;
    public static double TRIGGER_CLOSE = .45;

    public static double TRANSFER_SPEED = 1;

    // Shooter limits
    public static final double MIN_RPM = 900;
    public static final double MAX_RPM = 1765;
    public static final double MIN_HOOD = 0.000;
    public static final double MAX_HOOD = 0.24;
}
