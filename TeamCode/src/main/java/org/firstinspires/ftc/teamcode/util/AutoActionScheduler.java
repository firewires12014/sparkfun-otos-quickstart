package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import java.util.LinkedList;
import java.util.Queue;

public class AutoActionScheduler {
   final Queue<Action> actions = new LinkedList<>();
   final FtcDashboard dash = FtcDashboard.getInstance();
   final Canvas canvas = new Canvas();
   final Runnable pidUpdate;

   public AutoActionScheduler(Runnable pidUpdate) {
      this.pidUpdate = pidUpdate;
   }

   public void addAction(Action action) {
      actions.add(action);
   }

   public void run() {
      while (actions.peek() != null && !Thread.currentThread().isInterrupted()) {
         TelemetryPacket packet = new TelemetryPacket();
         packet.fieldOverlay().getOperations().addAll(canvas.getOperations());

         pidUpdate.run();

         boolean running = actions.peek().run(packet);
         dash.sendTelemetryPacket(packet);

         if (!running) {
            actions.remove();
            if (actions.peek() != null) {
               actions.peek().preview(canvas);
            }
         }
      }
   }

   public void clearActions() {
      actions.clear();
   }

   public boolean isBusy() {
      return !actions.isEmpty();
   }

   public Action currentAction() {
      return actions.peek();
   }

   /**
    * Wait until one of the condition(s) are met,
    * each separate boolean condition is treated as an OR
    * to utilize and AND simply in one condition bind it with parenthesis
    * @param conditions any number of condition(s) that when true proceed to the next line
    */
   public void await(boolean... conditions) {
      boolean contactedCondition = false;

      while (!contactedCondition) {
         for (boolean condition : conditions) {
            contactedCondition |= condition;
         }

         pidUpdate.run();
      }
   }
}