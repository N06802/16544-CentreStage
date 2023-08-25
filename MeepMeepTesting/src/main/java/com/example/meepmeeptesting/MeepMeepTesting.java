package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 10.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(0,0, Math.toRadians(0)))
                                .lineTo(new Vector2d(24,0)).build()
                );

        RoadRunnerBotEntity bot2 = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(360), Math.toRadians(370), 10.5)
                        .followTrajectorySequence(drive ->
                                 drive.trajectorySequenceBuilder(new Pose2d(10.69, 48, Math.toRadians(270.00)))
                                .splineToConstantHeading(new Vector2d(10.69, 13.34), Math.toRadians(270.00))
                                .splineToConstantHeading(new Vector2d(33.97, 13.72), Math.toRadians(-88.57))
                                .splineToSplineHeading(new Pose2d(34.53, -11.07, Math.toRadians(183.18)), Math.toRadians(183.18))
                                .splineToSplineHeading(new Pose2d(-12.58, -10.88, Math.toRadians(179.77)), Math.toRadians(179.77))
                                .splineToSplineHeading(new Pose2d(-11.45, 13.34, Math.toRadians(88.61)), Math.toRadians(88.61))
                                .splineTo(new Vector2d(10.45, 13.88), Math.toRadians(270))
                                .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot2)
                .start();
    }
}