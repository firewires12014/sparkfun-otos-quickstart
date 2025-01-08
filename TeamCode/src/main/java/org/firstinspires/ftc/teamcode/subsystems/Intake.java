package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.ActionUtil;
@Config

public class Intake {
    public DcMotorEx extension;
    public DcMotorEx spin;
    public Servo down;
    public Servo lock;
    public RevColorSensorV3 downSensor;
    RevColorSensorV3 forward;

    public static double targetPosition;
    public static double kp = 0;
    public static double manualPower = 0;
    public static double fourbarUp = 1;
    public static double fourbarDown = 0;
    public static double submerisbleBarDistance = 15;

    public Intake(HardwareMap hardwareMap) {
        extension = hardwareMap.get(DcMotorEx.class, "intakeExtension");
        spin = hardwareMap.get(DcMotorEx.class, "intakeSpin");
        down = hardwareMap.get(Servo.class, "intakeDown");
//        lock = hardwareMap.get(Servo.class, "intakeLock");
        downSensor = hardwareMap.get(RevColorSensorV3.class, "intakeDownSensor");
        //forward = hardwareMap.get(RevColorSensorV3.class, "intakeForward");
    }

    public void update () {
        double output = (targetPosition - extension.getCurrentPosition()) * kp;
        extension.setPower(output + manualPower);

    }

    public Action intakeOn () {
        return new ActionUtil.DcMotorExPowerAction(spin,0.7);
    }

    public Action intakeOff () {
        return new ActionUtil.DcMotorExPowerAction(spin, 0);
    }

    public Action fourbarOut () {
        return new ActionUtil.ServoPositionAction(down, fourbarDown);
    }

    public Action fourbarIn () {
        return new ActionUtil.ServoPositionAction(down, fourbarUp);
    }

    public Action manualControl (Gamepad gamepad) {
        return new ActionUtil.RunnableAction(()-> {

            double temperary = kp;
            kp = 0;
            manualPower = -gamepad.left_stick_y;

           return  gamepad.left_stick_y != 0;
        });
    }



}
