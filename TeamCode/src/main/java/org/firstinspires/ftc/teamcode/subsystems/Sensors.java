package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.SwyftLiDarSensor;

public class Sensors {

    double[] rightOffset = new double[]{6.1811, 5.82677};
    double[] backOffset = new double[]{3.85826, 7.874};

    Servo leftLED;
    Servo rightLED;

    SwyftLiDarSensor rightLiDAR;
    SwyftLiDarSensor backLiDAR;

    Rev2mDistanceSensor frontDistance;

    /**
     * Constructor for the Sensors class
     * @param hardwareMap
     */
    public Sensors(HardwareMap hardwareMap) {
        leftLED = hardwareMap.get(Servo.class, "leftLED");
        rightLED = hardwareMap.get(Servo.class, "rightLED");

        rightLiDAR = new SwyftLiDarSensor(hardwareMap, "rightLidar", SwyftLiDarSensor.OperatingConfiguration.Default);
        backLiDAR = new SwyftLiDarSensor(hardwareMap, "backLidar", SwyftLiDarSensor.OperatingConfiguration.Default);
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
}
