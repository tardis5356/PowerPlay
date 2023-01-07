package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(750);
         final Pose2d red_StartPos = new Pose2d(41, 64, Math.toRadians(90));

       final Pose2d red_PreloadPolePos = new Pose2d(27.25, 4.5, Math.toRadians(20));

      final Pose2d red_MainPolePos = new Pose2d(5, 22.5, Math.toRadians(-32));

        final Pose2d red_StackFarWaypointPos = new Pose2d(38, 11, Math.toRadians(0)); // x -58
        final Pose2d red_StackCloseWaypointPos = new Pose2d(50, 11, Math.toRadians(0)); // x -58
        final Pose2d red_StackPos = new Pose2d(58, 11, Math.toRadians(0)); // x -58

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
//        RoadRunnerBotEntity myBot = new customBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(526), Math.toRadians(180), 12.13)
                .setDimensions(10.6, 11.3)
                .followTrajectorySequence(drive ->
                              drive.trajectorySequenceBuilder(red_StartPos)
                                      .lineTo(new Vector2d(40, 63))
                                      .lineToConstantHeading(new Vector2d(36, 7))
                                      .lineToConstantHeading(new Vector2d(36, 14))
                                      .splineToLinearHeading(red_PreloadPolePos, Math.toRadians(250))

                                      .back(-0.10)
                                      .splineToSplineHeading(red_StackFarWaypointPos, Math.toRadians(90))//180

//                                      .back(-0.10)
//                                      .splineToSplineHeading(red_StackFarWaypointPos, Math.toRadians(0))//180

                                      .back(-0.10)
                                      .lineTo(red_StackPos.vec())

                                      .back(-0.10)
                                      .lineTo(red_StackCloseWaypointPos.vec())

                                      .back(0.10)
                                      .splineToSplineHeading(red_MainPolePos, Math.toRadians(122))//32

//                                        //deliver preload
//                                        .setReversed(true)
//                                        .splineTo(new Vector2d(-36, 20), Math.toRadians(270))
//                                        .splineToLinearHeading(new Pose2d(-30, 4, Math.toRadians(145)), Math.toRadians(340))
//                                        .setReversed(false)
//
//                                        //pick up first from stack
//                                        .back(-0.10)
//                                        .splineToSplineHeading(new Pose2d(-60, 12, Math.toRadians(180)), Math.toRadians(180))
//
//
////                            .strafeTo(new Vector2d(-60.35, 21.44))
////                            .splineToSplineHeading(new Pose2d(-8.11, 26.89, Math.toRadians(150.26)), Math.toRadians(337.52))
////                            .back(-0.10)
////                            .splineToSplineHeading(new Pose2d(-62.78, 21.05, Math.toRadians(-125.61)), Math.toRadians(244.69))
//                                        //cycle
//                                        .back(0.10)
//                                        .splineToSplineHeading(new Pose2d(-6, 20, Math.toRadians(212)), Math.toRadians(32))
//                                        .waitSeconds(0.2)
//                                        .back(-0.10)
//                                        .splineToSplineHeading(new Pose2d(-60, 12, Math.toRadians(180)), Math.toRadians(180))
//                                        .waitSeconds(0.2)
//
//                                        //cycle
//                                        .back(0.10)
//                                        .splineToSplineHeading(new Pose2d(-6, 20, Math.toRadians(212)), Math.toRadians(32))
//                                        .waitSeconds(0.2)
//                                        .back(-0.10)
//                                        .splineToSplineHeading(new Pose2d(-60, 12, Math.toRadians(180)), Math.toRadians(180))
//                                        .waitSeconds(0.2)
//
//                                        //cycle
//                                        .back(0.10)
//                                        .splineToSplineHeading(new Pose2d(-6, 20, Math.toRadians(212)), Math.toRadians(32))
//                                        .waitSeconds(0.2)
//                                        .back(-0.10)
//                                        .splineToSplineHeading(new Pose2d(-60, 12, Math.toRadians(180)), Math.toRadians(180))
//                                        .waitSeconds(0.2)
//
//                                        //cycle
//                                        .back(0.10)
//                                        .splineToSplineHeading(new Pose2d(-6, 20, Math.toRadians(212)), Math.toRadians(32))
//                                        .waitSeconds(0.2)
//                                        .back(-0.10)
//                                        .splineToSplineHeading(new Pose2d(-60, 12, Math.toRadians(180)), Math.toRadians(180))
//                                        .waitSeconds(0.2)
//
//                                        //cycle
//                                        .back(0.10)
//                                        .splineToSplineHeading(new Pose2d(-6, 20, Math.toRadians(212)), Math.toRadians(32))
//                                        .waitSeconds(0.2)
//                                        .back(-0.10)
//                                        .splineToSplineHeading(new Pose2d(-60, 12, Math.toRadians(180)), Math.toRadians(180))
//                                        .waitSeconds(0.2)
//
//                                        //cycle
//                                        .back(0.10)
//                                        .splineToSplineHeading(new Pose2d(-6, 20, Math.toRadians(212)), Math.toRadians(32))
//                                        .waitSeconds(0.2)
//                                        .back(-0.10)
//                                        .splineToSplineHeading(new Pose2d(-60, 12, Math.toRadians(180)), Math.toRadians(180))
//                                        .waitSeconds(0.2)


                                        .build()
                );

/*//                                .lineToConstantHeading(new Vector2d(-36, 0)) // score preload on x2
//
//                                .lineToConstantHeading(new Vector2d(-36, 3)) // strafes to in front of cones
//                                .splineToConstantHeading(new Vector2d(-60, 12), Math.toRadians(180)) // spline to cones
//
//                                .lineToConstantHeading(new Vector2d(-60, 20)) // strafe from cones to a2 for forward movement to high junction
//
////                                .lineToConstantHeading(new Vector2d(-24,36)) // forwards towards w3
////                                .splineToSplineHeading(new Pose2d(-7.5, 31, Math.toRadians(135)), Math.toRadians(310)) // spline to w3
//                                .splineToSplineHeading(new Pose2d(-7.5, 31, Math.toRadians(135)), Math.toRadians(300)) // spline to w3
//                                .setReversed(true)
//                                .splineToSplineHeading(new Pose2d(-36, 36, Math.toRadians(315)), Math.toRadians(180)) // spline to w3
//                                .splineToSplineHeading(new Pose2d(-60,20, Math.toRadians(-112)), Math.toRadians(45))
//                                .setReversed(false)
//                .back(-0.10)
//                .splineToSplineHeading(new Pose2d(-62.65, 11.92, Math.toRadians(-180)), Math.toRadians(180))
//                .lineTo(new Vector2d(-25.39, 13.51))
//                .splineToSplineHeading(new Pose2d(-4.70, 19.30, Math.toRadians(-147.95)), Math.toRadians(34.70))
//                .back(-0.10)
//                .splineToSplineHeading(new Pose2d(-25.17, 13.43, Math.toRadians(-178.46)), Math.toRadians(-178.23))
//                .lineTo(new Vector2d(-62.80, 12.22))*/

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}