package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.SwyftLiDarSensor;

public class Sensors {

    public static double[] rightOffset = new double[]{5.151, -4.753};
    public static double[] backOffset = new double[]{3.889, -6.014};


    SwyftLiDarSensor rightLiDAR;
    SwyftLiDarSensor backLiDAR;

    Rev2mDistanceSensor frontDistance;

    /**
     * Constructor for the Sensors class
     * @param hardwareMap
     */
    public Sensors(HardwareMap hardwareMap) {

        rightLiDAR = new SwyftLiDarSensor(hardwareMap, "rightLidar", SwyftLiDarSensor.OperatingConfiguration.Short);
        backLiDAR = new SwyftLiDarSensor(hardwareMap, "backLidar", SwyftLiDarSensor.OperatingConfiguration.Short);
    }

    /**
     * Get the distance from the back LiDAR sensor
     * @return
     */
    public double getBack() {
        return backLiDAR.getDistance();
    }

    /**
     * Get the distance from the right LiDAR sensor
     * @return
     */
    public double getRight() {
        return rightLiDAR.getDistance();
    }

    /**
     * Get the distance from the front distance sensor
     * @return
     */
    public double getFront() {
        return getFrontDistance(DistanceUnit.INCH);
    }

    /**
     * Get the distance from the front distance sensor
     * @param distanceUnit
     * @return
     */
    public double getFrontDistance(DistanceUnit distanceUnit) {
        return frontDistance.getDistance(distanceUnit);
    }

    public Pose2d getSpecimenPosition (Pose2d old) {
        double y = -72+getBack() - Sensors.backOffset[1];
        double x = 72-getRight() - Sensors.rightOffset[0];
        Vector2d newPose = new Vector2d(x,y);
        return new Pose2d(newPose, old.heading.toDouble());

        //return new Pose2d(newPose, Math.toRadians(90));
    }

    public Pose2d getSpecimenRightPosition (Pose2d old) {
        double x = 72-getRight() - Sensors.rightOffset[0];
        Vector2d newPose = new Vector2d(x,old.position.y);

        return new Pose2d(newPose, old.heading.toDouble());
        //return new Pose2d(newPose, Math.toRadians(90));
    }

}
