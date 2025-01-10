package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.ActionUtil;
import org.firstinspires.ftc.teamcode.util.PIDFController;

@Config
public class Hang {
    public DcMotorEx hangmotor1;
    public Servo ptoLeft;
    public Servo ptoRight;

    public static double ptoRightActive = 1;
    public static double ptoLeftActive = 1;
    public static double ptoRightInactive = 0;
    public static double ptoLeftInactive = 0;

    public static double targetPosition;
    public static double kp = 0.0018;
    public static double manualPower = 0;
    public static double hangOutPosition = 3000;
    public static double hangInPosition = 100;


    public static double hangPosition = 0;

    public PIDFController controller = new PIDFController(coefficients);

    public static PIDFController.PIDCoefficients coefficients = new PIDFController.PIDCoefficients();

    public Hang(HardwareMap hardwareMap) {
        hangmotor1 = hardwareMap.get(DcMotorEx.class, "hang");
        ptoLeft = hardwareMap.get(Servo.class, "ptoLeft");
        ptoRight = hardwareMap.get(Servo.class, "ptoRight");

        hangmotor1.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void update() {


    }

    public Action getHangReady() {
        return new InstantAction(() -> {
            hangPosition = 100;
        });
    }

    public Action engagePto() {
        return new ParallelAction(
                new ActionUtil.ServoPositionAction(ptoLeft, ptoLeftActive),
                new ActionUtil.ServoPositionAction(ptoRight, ptoRightActive));
    }
    public Action hangOut () {targetPosition = hangOutPosition;
        return new ActionUtil.RunnableAction(()-> {
            double output = (targetPosition - hangmotor1.getCurrentPosition()) * kp;
            hangmotor1.setPower(output);
            return Math.abs(targetPosition - hangmotor1.getCurrentPosition())> 50;
        });
    }

    public Action hangIn() {targetPosition = hangInPosition;
        return new ActionUtil.RunnableAction(()-> {
            double output = (targetPosition - hangmotor1.getCurrentPosition()) * kp;
            hangmotor1.setPower(output);
            return Math.abs(targetPosition - hangmotor1.getCurrentPosition())> 50;
        });
    }

        public Action disengagePto () {
            return new ParallelAction(
                    new ActionUtil.ServoPositionAction(ptoLeft, ptoLeftInactive),
                    new ActionUtil.ServoPositionAction(ptoRight, ptoRightInactive));
        }


    }

