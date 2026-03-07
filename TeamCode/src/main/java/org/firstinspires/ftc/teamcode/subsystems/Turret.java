package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware;

@Config
public class Turret extends Hardware {
    public Turret(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    public double angle = 0;
    public static double joystickReductionFactor = 0.9;

    public static double hardLeft = 0.34; //-45
    public static double hardRight = .68; //47
    public static double middle = .495; // 0

    public double lerp (double inputAngle) {
        return 0.0021393 * inputAngle + 0.497156;
        //return 0.0108726 * inputAngle + .482752;
    }

    public void setAngle (double angle) {
        this.angle = angle;
        turret.setPosition(lerp(angle));
    }

    public void increment(double joystick) {
        this.angle += joystick * joystickReductionFactor;

        if (angle < -45) {
            this.angle = -45;
        } else if (angle > 47) {
            this.angle = 47;
        }

        turret.setPosition(lerp(this.angle));
    }


//    public void middle() {
//        turret.setPosition(0.0);
//    }
//
//
//    public void rotateLeft() {
//        turret.setPosition(-.25);
//    }
//
//    public void rotateRight() {
//        turret.setPosition(.25);
//    }
//
}
