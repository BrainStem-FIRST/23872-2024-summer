package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;



public class MeepMeepTesting {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();


        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(12, 62, Math.toRadians(90)))
                            .setTangent(Math.toRadians(0))
                            .splineTo(new Vector2d(14, 30), Math.toRadians(180))
                        .setTangent(Math.toRadians(90))
                      .splineTo(new Vector2d(14, 55), Math.toRadians(90))
                       .setTangent(Math.toRadians(0))
                       .splineTo(new Vector2d(62, 38), Math.toRadians(-90))
                   //     .setTangent(Math.toRadians(-90))
                  //      .strafeTo(new Vector2d(58, -65))
                        .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)

                .addEntity(myBot)
                .start();

    }
}