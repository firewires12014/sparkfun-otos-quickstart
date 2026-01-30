package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

/** Central place for tunable/static constants. */
@Config
public final class Constants {
    private Constants() {
    }

    public static double LIFT1_UP = 1;
    public static double LIFT1_DOWN = 0;
    public static double LIFT2_UP = 1;
    public static double LIFT2_DOWN = 0;
    // Shooter PID/FF
    public static double SHOOTER_KP = 0.5;
    public static double SHOOTER_KD = 0.0;
    public static double SHOOTER_KV = 0.0004;
    public static boolean TUNE_SHOOTER = false;

    // Shooter control
    public static int SHOOTER_VELOCITY = 1800;
    public static double SHOOTER_VELOCITY_RAMP_RATE = 1500.0;

    // Hood limits
    public static float HOOD_UPPER_LIMIT = 0.87f;
    public static float HOOD_MIDDLE_LIMIT = 0.25f;
    public static float HOOD_LOWER_LIMIT = 0.0f;
    public static float HOOD_RED_AUTO = 0.38f;

    // Define your range (180 degrees total = -90 to +90)
    public static double MAX_RAD = Math.toRadians(721.67);
    public static  double MIN_RAD = Math.toRadians(-695);
    public static double BUFFER = Math.toRadians(5); // Slow down 5 degrees before the stop
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

    // Intake
    public static double AUTO_INTAKE_TIME = 4;

}
