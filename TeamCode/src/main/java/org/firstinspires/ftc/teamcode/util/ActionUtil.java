package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.Callable;

public class ActionUtil {
   public static class DcMotorExPowerAction implements Action {
      double power;
      DcMotorEx motor;

      public DcMotorExPowerAction(DcMotorEx motor, double power) {
         this.power = power;
         this.motor = motor;
      }

      @Override
      public boolean run(TelemetryPacket packet) {
         motor.setPower(power);
         return false;
      }
   }

   public static class RunnableTimedAction implements Action {
      Callable<Boolean> action;
      ElapsedTime timer = new ElapsedTime();
      double offset = 0;
      boolean startTimer = false;

      public RunnableTimedAction(double offset, Callable<Boolean> action) {
         this.action = action;
         this.offset = offset;
      }

      @Override
      public boolean run(TelemetryPacket packet) {
         try {
            if (!startTimer) {
               timer.reset();
               startTimer = true;
            }

            return action.call() && offset > timer.seconds();
         } catch (Exception e) {
            throw new RuntimeException(e);
         }
      }
   }

   public static class RunnableAction implements Action {
      Callable<Boolean> action;

      public RunnableAction(Callable<Boolean> action) {
         this.action = action;
      }

      @Override
      public boolean run(TelemetryPacket packet) {
         try {
            return action.call();
         } catch (Exception e) {
            throw new RuntimeException(e);
         }
      }
   }

   public static Action Offset(double offset, Action firstAction, Action secondAction) {
      return new ParallelAction(
               firstAction,
              new SequentialAction(
                      new SleepAction(offset),
                      secondAction
              )
      );
   }

   public static Action Delay(double offset, Action action) {
      return new SequentialAction(
              new SleepAction(offset),
              action
      );
   }

   public static class ServoPositionAction implements Action {
      double position;
      Servo servo;

      public ServoPositionAction(Servo servo, double position) {
         this.servo = servo;
         this.position = position;
      }

      @Override
      public boolean run(TelemetryPacket packet) {
         servo.setPosition(position);
         return false;
      }
   }

   public static boolean compareDouble(double a, double b) {
      return Math.abs(a - b) < 0.001;
   }
}