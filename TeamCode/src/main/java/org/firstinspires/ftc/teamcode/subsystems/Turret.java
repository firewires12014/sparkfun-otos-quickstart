package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware;

@Config
public class Turret extends Hardware {
    public Turret(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    public void stop() {
        turret.setPower(0.0);
    }


    public void rotateLeft() {
        turret.setPower(-.25);
    }

    public void rotateRight() {
        turret.setPower(.25);
    }

}
