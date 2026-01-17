package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.MecanumDrive;

/**
 * Minimal Road Runner autonomous template.
 *
 * - Demonstrates importing Road Runner (Actions/ActionBuilder), Hardware, and Constants.
 * - Use this as a starting point for new autos.
 */
@Autonomous(name = "templateAuto", group = "Templates")
public class templateAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware map (non-RR mechanisms)
        Hardware robot = new Hardware(hardwareMap);

        // Road Runner drive + starting pose
        Pose2d startPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Example: reference constants (replace with your own)
        int xOffset = Constants.BLUE_CLOSE_X_OFFSET;
        int yOffset = Constants.BLUE_CLOSE_Y_OFFSET;

        // Build example actions
        Action goToPoint = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(24 + xOffset, 0 + yOffset))
                .build();

        Action returnHome = drive.actionBuilder(new Pose2d(24 + xOffset, 0 + yOffset, 0))
                .strafeTo(new Vector2d(0, 0))
                .build();

        // Optional: do any mechanism pre-start setup here
        // robot.hood.setPosition(Constants.HOOD_LOWER_LIMIT);

        waitForStart();
        if (isStopRequested()) return;

        // Run actions (sequential)
        Actions.runBlocking(new SequentialAction(
                goToPoint,
                returnHome
        ));

        // Example: stop mechanisms at end
        robot.intake.setPower(0);
        robot.transfer.setPower(0);
    }
}
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

/** Central place for tunable/static constants. */
@Config
public final class Constants {
    private Constants() {}

    // Shooter PID/FF
    public static double SHOOTER_KP = 0.5;
    public static double SHOOTER_KD = 0.0;
    public static double SHOOTER_KV = 0.0004;
    public static boolean TUNE_SHOOTER = false;

    // Shooter control
    public static int SHOOTER_VELOCITY = 1900;
    public static double SHOOTER_VELOCITY_RAMP_RATE = 1500.0;

    // Hood limits
    public static float HOOD_UPPER_LIMIT = 0.7f;
    public static float HOOD_LOWER_LIMIT = 0.0f;

    // Vision/auto offsets
    public static int BLUE_CLOSE_X_OFFSET = 12;
    public static int BLUE_CLOSE_Y_OFFSET = -14;
}

