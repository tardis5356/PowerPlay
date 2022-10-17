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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 66, Math.toRadians(180)))
                                .lineToConstantHeading(new Vector2d(-36, 0)) // scoreX2
                                .lineToConstantHeading(new Vector2d(-36, 3)) // strafeFromX2
                                .splineToConstantHeading(new Vector2d(-60, 12), Math.toRadians(180)) // strafeFromX2
                                .lineToConstantHeading(new Vector2d(-60, 36)) // strafePrepW3
                                .lineToConstantHeading(new Vector2d(-7.5,36)) // goTowardsW3
                                .splineToSplineHeading(new Pose2d(-7.5, 31, Math.toRadians(135)), Math.toRadians(0)) // goTowardsW3
                                .lineToLinearHeading(new Pose2d(-7.5, 36, Math.toRadians(180))) // resetRotation
                                .lineToConstantHeading(new Vector2d(-60, 36)) // goToA2
                                .lineToConstantHeading(new Vector2d(-60, 12)) // collectCone2

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}