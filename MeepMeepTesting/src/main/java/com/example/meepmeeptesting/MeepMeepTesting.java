package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
//        RoadRunnerBotEntity myBot = new customBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(526), Math.toRadians(180), 12.13)
                .setDimensions(10.6, 11.3)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 66, Math.toRadians(180)))
                                .lineToConstantHeading(new Vector2d(-36, 0)) // score preload on x2

                                .lineToConstantHeading(new Vector2d(-36, 3)) // strafes to in front of cones
                                .splineToConstantHeading(new Vector2d(-60, 12), Math.toRadians(180)) // spline to cones

                                .lineToConstantHeading(new Vector2d(-60, 36)) // strafe from cones to a2 for forward movement to high junction

                                .lineToConstantHeading(new Vector2d(-24,36)) // forwards towards w3
                                .splineToSplineHeading(new Pose2d(-7.5, 31, Math.toRadians(135)), Math.toRadians(310)) // spline to w3

                                .splineToSplineHeading(new Pose2d(-36, 36, Math.toRadians(180)), Math.toRadians(-135)) // spline to w3
//                                .splineToSplineHeading(new Pose2d(-35.6,36.8, Math.toRadians(-152)), Math.toRadians(-172))
                                .splineToSplineHeading(new Pose2d(-62.8,24, Math.toRadians(-112)), Math.toRadians(-112))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}