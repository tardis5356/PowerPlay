package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(750);
        final Pose2d blue_StartPos = new Pose2d(-30.5, 64, Math.toRadians(90));
        final Pose2d blue_PreloadPolePos = new Pose2d(-25, 7, Math.toRadians(140));//-35//130
        final Pose2d blue_MedPreloadPolePos = new Pose2d(-25.5, 33, Math.toRadians(140));//-35//130
        final Pose2d blue_MedPolePos = new Pose2d(-30.5, 21, Math.toRadians(220)); //-30.5, 21
        final Pose2d blue_StackWaypointPos = new Pose2d(-50, 13, Math.toRadians(180));
        final Pose2d blue_StackPos = new Pose2d(-59.5, 13, Math.toRadians(180));//-63 //(-59.5, 12)

        final Pose2d red_StartPos = new Pose2d(40.5, 64, Math.toRadians(90));
        final Pose2d red_PreloadPolePos = new Pose2d(32, 4, Math.toRadians(40)); //(32, 2)
        final Pose2d red_MedPreloadPolePos = new Pose2d(28, 29, Math.toRadians(90)); // (30, 16.5) //332
        final Pose2d red_MedPolePos = new Pose2d(33, 19, Math.toRadians(328)); // (30, 16.5) //332 // (32, 20)  //TODO: -32?
        final Pose2d red_StackWaypointPos = new Pose2d(50, 12, Math.toRadians(0)); // (50, 9)
        final Pose2d red_StackPos = new Pose2d(62.5, 12, Math.toRadians(0)); // (62.5, 9)

        RoadRunnerBotEntity blueBot = new DefaultBotBuilder(meepMeep)
//        RoadRunnerBotEntity myBot = new customBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(90), Math.toRadians(90), 9.43)
                .setDimensions(12, 12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(blue_StartPos)
                                .setReversed(true)
                                // original
//                                .lineTo(new Vector2d(-31, 63))
//                                .splineToConstantHeading(new Vector2d(-33, 40), Math.toRadians(270))
//                                .splineToLinearHeading(blue_PreloadPolePos, Math.toRadians(340))
//                                .splineToConstantHeading(new Vector2d(-33, 40), Math.toRadians(270))

                                // single spline high
//                                .lineTo(new Vector2d(-34, 62))
//                                .lineTo(new Vector2d(-34, 60))
//                                .splineTo(blue_PreloadPolePos.vec(), Math.toRadians(300))
//                                .setReversed(false)

                                // single spline med
                                .lineTo(new Vector2d(-33, 62))
                                .lineTo(new Vector2d(-33, 61))
                                .splineTo(blue_MedPreloadPolePos.vec(), Math.toRadians(300))
                                .setReversed(false)

                                .back(-0.10)
                                // original
//                                .splineToLinearHeading(blue_StackWaypointPos, Math.toRadians(180))

                                // med
                                .lineToLinearHeading(new Pose2d(-40, 15, Math.toRadians(210)))
                                .splineToSplineHeading(blue_StackWaypointPos, Math.toRadians(180))


                                .back(-0.10)
                                .lineTo(blue_StackPos.vec())

                                .back(-0.10)
                                .lineTo(blue_StackWaypointPos.vec())

                                .back(0.10)
                                .splineToSplineHeading(blue_MedPolePos, Math.toRadians(50))

                                .back(-0.10)
                                .splineToSplineHeading(blue_StackWaypointPos, Math.toRadians(180))

                                .back(0.10)
                                .splineToSplineHeading(blue_MedPolePos, Math.toRadians(50))

                                .build()
                );

        RoadRunnerBotEntity redBot = new DefaultBotBuilder(meepMeep)
//        RoadRunnerBotEntity myBot = new customBotBuilder(meepMeep)

                .setColorScheme(new ColorSchemeRedDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(90), Math.toRadians(90), 9.43)
                .setDimensions(12, 12)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(red_StartPos)
                                        .setReversed(true)
                                        // original
//                                        .lineTo(new Vector2d(35, 60))
//                                        .splineToConstantHeading(new Vector2d(33, 20), Math.toRadians(270)) // (36, 7)
//                                        .splineToLinearHeading(red_PreloadPolePos, Math.toRadians(200))
//                                        .splineToLinearHeading(red_PreloadPolePos, Math.toRadians(230))

                                        // single spline high
//                                        .lineTo(new Vector2d(38, 62))
//                                        .lineTo(new Vector2d(38, 60))
//                                        .splineTo(red_PreloadPolePos.vec(), Math.toRadians(230))

                                        // single spline med
                                        .lineTo(new Vector2d(38, 62))
                                        .lineTo(new Vector2d(38, 60))
                                        .splineTo(red_MedPreloadPolePos.vec(), Math.toRadians(230))

                                        .setReversed(false)

                                        .back(-0.10)
                                        // original
//                                        .splineToLinearHeading(red_StackWaypointPos, Math.toRadians(0)) //TODO: test 90 end tangent

                                        // med
                                        .lineToLinearHeading(new Pose2d(32, 20, Math.toRadians(330)))
                                        .splineToSplineHeading(red_StackWaypointPos, Math.toRadians(0))

                                        .back(-0.10)
                                        .lineTo(red_StackPos.vec())

//                                        .back(-0.10)
//                                        .lineTo(red_StackWaypointPos.vec())

                                        .back(0.10)
                                        .splineToSplineHeading(red_MedPolePos, Math.toRadians(148))

                                        .back(-0.10)
                                        .splineToSplineHeading(red_StackWaypointPos, Math.toRadians(0))

                                        .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(blueBot)
                .addEntity(redBot)
                .start();
    }
}