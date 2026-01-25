package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

/** Central place for tunable/static constants. */
@Config
public final class Constants {
    private Constants() {
    }

    // Shooter PID/FF
    public static double SHOOTER_KP = 0.5;
    public static double SHOOTER_KD = 0.0;
    public static double SHOOTER_KV = 0.0004;
    public static boolean TUNE_SHOOTER = false;

    // Shooter control
    public static int SHOOTER_VELOCITY = 1800;
    public static double SHOOTER_VELOCITY_RAMP_RATE = 1500.0;

    // Hood limits
    public static float HOOD_UPPER_LIMIT = 0.35f;
    public static float HOOD_MIDDLE_LIMIT = 0.3f;
    public static float HOOD_LOWER_LIMIT = 0.0f;

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
