package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

@Config
public class Vision {
    // Require at least one fiducial in view before trusting Limelight botpose.
    public static int MIN_FIDUCIALS_FOR_POSE = 1;

    // Only trust pose estimates when at least one alliance-specific AprilTag ID is visible.
    public static String TRUSTED_BLUE_TAG_IDS_CSV = "20";
    public static String TRUSTED_RED_TAG_IDS_CSV  = "24";

    // Limelight turret-centering settings.
    public static double SERVO_MIN = 0.034;
    public static double SERVO_CENTER = 0.495;
    public static double SERVO_MAX = 0.68;
    public static double KP = 0.0001;
    public static double MAX_STEP_PER_LOOP = 0.0012;
    public static double DEADBAND_DEGREES = 0.5;
    public static double DIRECTION = 1.0;

    private final Limelight3A limelight;
    private final Servo turret;
    private Double latestTx = null;
    private double servoPosition;

    public Vision(Limelight3A limelight, Servo turret) {
        this.limelight = limelight;
        this.turret = turret;

        // Keep center inside [min, max] in case constants are tuned out-of-range.
        servoPosition = Range.clip(SERVO_CENTER, SERVO_MIN, SERVO_MAX);
        turret.setPosition(servoPosition);
    }

    public void start() {
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    public Double getLatestTx() {
        return latestTx;
    }

    public double getTurretServoPosition() {
        return servoPosition;
    }

    public boolean updatePoseAndAimFromLimelight(Drive drive, boolean isBlueAlliance) {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            latestTx = null;
            return false;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.size() < MIN_FIDUCIALS_FOR_POSE) {
            latestTx = null;
            return false;
        }

        if (!containsTrustedTag(fiducials, isBlueAlliance)) {
            latestTx = null;
            return false;
        }

        latestTx = result.getTx();
//        aimTurretFromTx(latestTx);

        Pose3D botpose = result.getBotpose();
        if (botpose == null) {
            latestTx = null;
            return false;
        }

        double xIn = DistanceUnit.INCH.fromMeters(botpose.getPosition().x);
        double yIn = DistanceUnit.INCH.fromMeters(botpose.getPosition().y);
        double headingRad = botpose.getOrientation().getYaw(AngleUnit.RADIANS);

//        drive.setPose(new Pose2d(xIn, yIn, headingRad));
        return true;
    }

    private void aimTurretFromTx(double tx) {
        if (Math.abs(tx) <= DEADBAND_DEGREES) {
            return;
        }

        double step = DIRECTION * KP * tx;
        step = Range.clip(step, -MAX_STEP_PER_LOOP, MAX_STEP_PER_LOOP);
        servoPosition += step;
        servoPosition = Range.clip(servoPosition, SERVO_MIN, SERVO_MAX);
        turret.setPosition(servoPosition);
    }

    private boolean containsTrustedTag(List<LLResultTypes.FiducialResult> fiducials, boolean isBlueAlliance) {
        String trustedCsv = isBlueAlliance ? TRUSTED_BLUE_TAG_IDS_CSV : TRUSTED_RED_TAG_IDS_CSV;
        Set<Integer> trustedIds = parseTrustedTagIds(trustedCsv);
        if (trustedIds.isEmpty()) {
            return true;
        }

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (trustedIds.contains(fiducial.getFiducialId())) {
                return true;
            }
        }
        return false;
    }

    private Set<Integer> parseTrustedTagIds(String csv) {
        Set<Integer> ids = new HashSet<>();
        if (csv == null || csv.trim().isEmpty()) {
            return ids;
        }

        String[] parts = csv.split(",");
        for (String part : parts) {
            try {
                ids.add(Integer.parseInt(part.trim()));
            } catch (NumberFormatException ignored) {
                // Skip malformed entries so dashboard tuning is forgiving.
            }
        }
        return ids;
    }
}