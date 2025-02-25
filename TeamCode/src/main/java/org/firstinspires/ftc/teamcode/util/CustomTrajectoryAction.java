package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TimeTrajectory;

import org.firstinspires.ftc.teamcode.MecanumDrive;

// TODO: Goal to be able to run actions with negative time prior to the end of a trajectory action
//       Movement actions should be done at the bottom/top of the auto to extract the data from them
//       Then use custom action to process them

public class CustomTrajectoryAction implements Action {
    private final Action rawAction;

    public double durationMs;
    public double displacementIn;
    public Pose2d startPose;
    public Pose2d endPose;

    public CustomTrajectoryAction(Action action) throws InterruptedException {
        this.rawAction = action;
        MecanumDrive.FollowTrajectoryAction castedAction = null;

        SequentialAction s = (SequentialAction) action;

        castedAction = (MecanumDrive.FollowTrajectoryAction) s.getInitialActions().get(0);

//        if (action.getClass().equals(MecanumDrive.FollowTrajectoryAction.class)) { // test if this works
//            castedAction = (MecanumDrive.FollowTrajectoryAction) action;
//        } else {
//            throw new InterruptedException("Wrong type of action, must be Trajectory Action");
//        }

        process(castedAction);
    }

    private void process(MecanumDrive.FollowTrajectoryAction action) {
        TimeTrajectory t = action.timeTrajectory;
        this.durationMs = t.duration; // Check validity
        this.displacementIn = t.path.length();

        // AFAIK 0 represents position, 1 position & velocity, 2 position, velocity & acceleration
        this.endPose = t.path.end(0).value();
        this.startPose = t.path.begin(0).value();
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        return rawAction.run(telemetryPacket);
    }

    @Override
    public void preview(@NonNull Canvas c) {
        rawAction.preview(c);
    }

    // Just in this class for conveniences sake
    public static Action OffsetAction(double offset, CustomTrajectoryAction driveAction, Action otherAction) {
        return new ParallelAction(
                driveAction,
                new SequentialAction(
                        new SleepAction(driveAction.durationMs + offset),
                        otherAction
                )
        );
    }
}