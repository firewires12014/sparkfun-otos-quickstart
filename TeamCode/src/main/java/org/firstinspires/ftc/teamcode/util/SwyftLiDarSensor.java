package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SwyftLiDarSensor {

    private final AnalogInput sensor;

    public enum OperatingConfiguration {
        Short,
        Default,
        Long
    }

    private final double SHORT_MULTIPLIER = 32.50930976;
    private final double DEFAULT_MULTIPLIER = 48.78136376;
    private final double LONG_MULTIPLIER = 76.85612461;

    private final double SHORT_ADJUSTMENT = -2.695384202;
    private final double DEFUALT_ADJUSTMENT = -4.985354503;
    private final double LONG_ADJUSTMENT = -9.925949725;


    private OperatingConfiguration mode = OperatingConfiguration.Default;

    public SwyftLiDarSensor(HardwareMap hardwareMap, String sensorName) {
        sensor = hardwareMap.get(AnalogInput.class, sensorName);
    }

    public SwyftLiDarSensor(HardwareMap hardwareMap, String sensorName, OperatingConfiguration mode) {
        sensor = hardwareMap.get(AnalogInput.class, sensorName);
        this.mode = mode;
    }

    public double getRawVoltage() {
        return sensor.getVoltage();
    }

    public double getShortDistance() {
        return (getRawVoltage() * SHORT_MULTIPLIER) + SHORT_ADJUSTMENT;
    }

    public double getDefaultDistance() {
        return (getRawVoltage() * DEFAULT_MULTIPLIER) + DEFUALT_ADJUSTMENT;
    }

    public double getLongDistance() {
        return (getRawVoltage() * LONG_MULTIPLIER) + LONG_ADJUSTMENT;
    }

    // returned in Inches, TODO: DistanceUnit parameters to allow for different returned units
    public double getDistance() {
        switch (mode) {
            case Short:
                return getShortDistance();
            case Long:
                return getLongDistance();
            default:
                return getDefaultDistance();
        }
    }
}