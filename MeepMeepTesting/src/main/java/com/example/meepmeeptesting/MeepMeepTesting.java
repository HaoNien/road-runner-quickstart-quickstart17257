package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12)
//                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(36, 62, Math.toRadians(270)))
//
//                        .splineTo(new Vector2d(38,40),Math.toRadians(300))
//
//                        //.strafeTo(new Vector2d(-45, 50))
//                        //.strafeTo(new Vector2d(-45, 12))
//
//
//
//                        //.turn(Math.toRadians(90))
//
//                        .build());
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-10, 60, Math.toRadians(270)))

                        .forward(26)
                        .back(20)
                        //.forward(26)
                        //.splineTo(new Vector2d(-38,40),Math.toRadians(90))
                        //.splineToConstantHeading(new Vector2d(-38,40),Math.toRadians(90))
                        .turn(Math.toRadians(-45))
                        .splineTo(new Vector2d(-29,41),Math.toRadians(220))
                        .turn(Math.toRadians(-60))
                        .turn(Math.toRadians(60))

                        .strafeTo(new Vector2d(-32,41))
                        .strafeTo(new Vector2d(-10, 35))
                        //.strafeRight()
                //.strafeTo(new Vector2d(-45, 50))
                //.strafeTo(new Vector2d(-45, 12))



                //.turn(Math.toRadians(90))

                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}