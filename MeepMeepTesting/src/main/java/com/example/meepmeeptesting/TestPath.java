package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class TestPath {
    public static void main(String[] args) {
        Pose2d start = new Pose2d(7, -64, Math.toRadians(90));
        Pose2d scorePreload = new Pose2d(new Vector2d(-6, -27), Math.toRadians(90));
        Pose2d backupFromSub = new Pose2d(new Vector2d(-6, -40), Math.toRadians(90));
        Pose2d intakeSample1 = new Pose2d(new Vector2d(28.96, -39.15), Math.toRadians(36.61));

        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15).build();

        myBot.runAction(myBot.getDrive().actionBuilder(start)
                        .splineToLinearHeading(scorePreload, scorePreload.heading)
                        //.setTangent(Math.toRadians(-45))
                        .strafeToLinearHeading(backupFromSub.position, backupFromSub.heading)
                        .splineToLinearHeading(intakeSample1, intakeSample1.heading)
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}