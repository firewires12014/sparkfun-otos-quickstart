package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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


    public static double hangPosition = 0;

    public PIDFController controller = new PIDFController(coefficients);

    public static PIDFController.PIDCoefficients coefficients = new PIDFController.PIDCoefficients();

    public Hang(HardwareMap hardwareMap) {
        hangmotor1 = hardwareMap.get(DcMotorEx.class, "hang");
        ptoLeft = hardwareMap.get(Servo.class, "ptoLeft");
        ptoRight = hardwareMap.get(Servo.class, "ptoRight");
    }

    public void update() {
        controller.targetPosition = hangPosition;
        controller.update(hangmotor1.getCurrentPosition());
        double output = controller.update(hangmotor1.getCurrentPosition());

        hangmotor1.setPower(output);
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
        public Action disengagePto () {
            return new ParallelAction(
                    new ActionUtil.ServoPositionAction(ptoLeft, ptoLeftInactive),
                    new ActionUtil.ServoPositionAction(ptoRight, ptoRightInactive));
        }


    }

