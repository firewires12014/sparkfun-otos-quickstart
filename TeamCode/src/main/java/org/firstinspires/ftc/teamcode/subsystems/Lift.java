package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.PIDFController;
@Config
public class Lift {
    public DcMotorEx lift;
    public DigitalChannel limitSwitch;

    PIDFController controller;
    PIDFController.PIDCoefficients coefficients = new PIDFController.PIDCoefficients();

    public static double liftPosition = 0;

    public static double kP = 0.0065;
    public static double kI = 0;
    public static double kD = 0;

    public static double ff = 0;

    public Lift(HardwareMap hardwareMap) {

        lift = hardwareMap.get(DcMotorEx.class, "lift");
        //limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");
        coefficients.kP = 0;
        coefficients.kI = 0;
        coefficients.kD = 0;
        controller = new PIDFController(coefficients);

        lift.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void update () {
        coefficients.kP = kP;
        coefficients.kI = kI;
        coefficients.kD = kD;
        controller = new PIDFController(coefficients);
        controller.targetPosition = liftPosition;
        double output = kP * (liftPosition - lift.getCurrentPosition());
        lift.setPower(output);


    }

    public Action getHangReady () {
        return new InstantAction(()-> {liftPosition = 100;});
    }

    public Action setLiftPosition (double liftPosition) {return new InstantAction(()-> {this.liftPosition = liftPosition;});}
}
