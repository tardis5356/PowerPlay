package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);
        final Pose2d blue_StartPos = new Pose2d(-30.5, 64, Math.toRadians(90));
        final Pose2d blue_PreloadPolePos = new Pose2d(-26, 6, Math.toRadians(140));//-35//130
        final Pose2d blue_MainPolePos = new Pose2d(-5, 22.5, Math.toRadians(212));
        final Pose2d blue_MedPolePos = new Pose2d(-29, 22, Math.toRadians(212));
        final Pose2d blue_StackFarWaypointPos = new Pose2d(-38, 11, Math.toRadians(180));
        final Pose2d blue_StackCloseWaypointPos = new Pose2d(-50, 12, Math.toRadians(180));
        final Pose2d blue_StackPos = new Pose2d(-59, 12, Math.toRadians(180));//-63

        //red auto positions
        final Pose2d red_StartPos = new Pose2d(41, 64, Math.toRadians(90));
        final Pose2d red_PreloadPolePos = new Pose2d(27.25, 4.5, Math.toRadians(40));
        final Pose2d red_MainPolePos = new Pose2d(5, 22.5, Math.toRadians(302));
        final Pose2d red_MedPolePos = new Pose2d(30, 20, Math.toRadians(332));
        final Pose2d red_StackFarWaypointPos = new Pose2d(38, 11, Math.toRadians(0));
        final Pose2d red_StackCloseWaypointPos = new Pose2d(50, 11, Math.toRadians(0));
        final Pose2d red_StackPos = new Pose2d(58, 11, Math.toRadians(0));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
//        RoadRunnerBotEntity myBot = new customBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(90), Math.toRadians(90), 9.43)
                .setDimensions(12, 12)
                .followTrajectorySequence(drive ->
                                // RED
//                        drive.trajectorySequenceBuilder(red_StartPos)
//                                .setReversed(true)
////                        .lineTo(new Vector2d(36, 54))
////                        .lineTo(new Vector2d(36, 7), SampleMecanumDrive_R2V2.getVelocityConstraint(62, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
////                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
////                        .lineTo(new Vector2d(36, 20))
//                                .lineTo(new Vector2d(40, 63))
//                                .splineToConstantHeading(new Vector2d(36, 7), Math.toRadians(270))
//                                .lineTo(new Vector2d(36, 14))
//                                .splineToLinearHeading(red_PreloadPolePos, Math.toRadians(200))
//                                .setReversed(false)
//
//                                .back(-0.10)
//                                .splineToLinearHeading(red_StackCloseWaypointPos, Math.toRadians(0))
//
//                                .back(-0.10)
//                                .lineTo(red_StackPos.vec())
//
//                                .back(-0.10)
//                                .lineTo(red_StackCloseWaypointPos.vec())
//
//
////                                //score cone (MEDIUM)
//                                .back(0.10)
//                                .splineToSplineHeading(red_MedPolePos, Math.toRadians(150))
//
//                                .back(-0.10)
////                                .splineToSplineHeading(red_StackCloseWaypointPos, Math.toRadians(0))
//                                .splineToSplineHeading(red_StackPos, Math.toRadians(0))
////                                //score cone (MEDIUM)
//                                .back(0.10)
//                                .splineToSplineHeading(red_MedPolePos, Math.toRadians(150))
//
//                                .back(-0.10)
////                                .splineToSplineHeading(red_StackCloseWaypointPos, Math.toRadians(0))
//                                .splineToSplineHeading(red_StackPos, Math.toRadians(0))
////                                //score cone (MEDIUM)
//                                .back(0.10)
//                                .splineToSplineHeading(red_MedPolePos, Math.toRadians(150))
//
//                                .back(-0.10)
////                                .splineToSplineHeading(red_StackCloseWaypointPos, Math.toRadians(0))
//                                .splineToSplineHeading(red_StackPos, Math.toRadians(0))
////                                //score cone (MEDIUM)
//                                .back(0.10)
//                                .splineToSplineHeading(red_MedPolePos, Math.toRadians(150))
//
//                                .back(-0.10)
////                                .splineToSplineHeading(red_StackCloseWaypointPos, Math.toRadians(0))
//                                .splineToSplineHeading(red_StackPos, Math.toRadians(0))

//                                .setReversed(true)
////                                .lineTo(new Vector2d(36, 54))
////                                .strafeLeft(0.1)
//                                .lineTo(new Vector2d(40.6, 63))
//                                .splineToConstantHeading(new Vector2d(36, 7), Math.toRadians(270))
////                                .lineTo(new Vector2d(36, 7))
//                                .lineTo(new Vector2d(36, 20))
//                                .splineToLinearHeading(red_PreloadPolePos, Math.toRadians(230))
//                                .setReversed(false)
//
//                                .back(-0.10)
//                                .splineToLinearHeading(red_StackCloseWaypointPos, Math.toRadians(0))
//
//                                // grab cone
//                                .back(-0.10)
//                                .lineTo(red_StackPos.vec())
//
//                                .back(-0.10)
//                                .lineTo(red_StackCloseWaypointPos.vec())
//
//                                //score cone (HIGH)
////                                .back(0.10)
////                                .splineToSplineHeading(red_MainPolePos, Math.toRadians(122))
////                                .back(-0.10)
////                                .splineToSplineHeading(red_StackCloseWaypointPos, Math.toRadians(0))
//
//
//                                .lineToLinearHeading(new Pose2d(60, 15, Math.toRadians(270)))


                                //BLUE
                                drive.trajectorySequenceBuilder(blue_StartPos)
                                        .setReversed(true)
                                        .lineTo(new Vector2d(-31, 63))
//                                        .splineToConstantHeading(new Vector2d(-33, 20), Math.toRadians(270))
//                                        .splineToLinearHeading(blue_PreloadPolePos, Math.toRadians(340))
                                        .splineToLinearHeading(blue_PreloadPolePos, Math.toRadians(300))
                                        .setReversed(false)
//                                        .setReversed(true)
////                        .splineTo(new Vector2d(-36, 20), Math.toRadians(270))
//                                        .lineTo(new Vector2d(-36, 54))
//                                        .lineTo(new Vector2d(-36, 7))
//                                        .lineTo(new Vector2d(-36, 20))
//                                        .splineToLinearHeading(blue_PreloadPolePos, Math.toRadians(340))
//                                        .setReversed(false)
//                                        .back(-0.10)
//                                        .splineToLinearHeading(blue_StackCloseWaypointPos, Math.toRadians(180))
//
//                                        //grab cone
//                                        .back(-0.10)
//                                        .lineTo(blue_StackPos.vec())
//                                        .back(-0.10)
//                                        .lineTo(blue_StackCloseWaypointPos.vec())
//
//                                        //score cone
//                                        .back(0.10)
//                                        .splineToSplineHeading(blue_MedPolePos, Math.toRadians(32))
//                                        .back(-0.10)
//                                        .splineToSplineHeading(blue_StackCloseWaypointPos, Math.toRadians(180))
//
//                                        // park
//                                        .lineToLinearHeading(new Pose2d(-12, 18, Math.toRadians(90)))

//                                        //grab cone
//                                        .back(-0.10)
//                                        .lineTo(blue_StackPos.vec())
//                                        .back(-0.10)
//                                        .lineTo(blue_StackCloseWaypointPos.vec())
//
//                                        //score cone
//                                        .back(0.10)
//                                        .splineToSplineHeading(blue_MedPolePos, Math.toRadians(32))
//                                        .back(-0.10)
//                                        .splineToSplineHeading(blue_StackCloseWaypointPos, Math.toRadians(180))
//
//                                        //grab cone
//                                        .back(-0.10)
//                                        .lineTo(blue_StackPos.vec())
//                                        .back(-0.10)
//                                        .lineTo(blue_StackCloseWaypointPos.vec())
//
//                                        //score cone
//                                        .back(0.10)
//                                        .splineToSplineHeading(blue_MedPolePos, Math.toRadians(32))
//                                        .back(-0.10)
//                                        .splineToSplineHeading(blue_StackCloseWaypointPos, Math.toRadians(180))
//
//                                        //grab cone
//                                        .back(-0.10)
//                                        .lineTo(blue_StackPos.vec())
//                                        .back(-0.10)
//                                        .lineTo(blue_StackCloseWaypointPos.vec())
//
//                                        //score cone
//                                        .back(0.10)
//                                        .splineToSplineHeading(blue_MedPolePos, Math.toRadians(32))
//                                        .back(-0.10)
//                                        .splineToSplineHeading(blue_StackCloseWaypointPos, Math.toRadians(180))
//
//                                        //grab cone
//                                        .back(-0.10)
//                                        .lineTo(blue_StackPos.vec())
//                                        .back(-0.10)
//                                        .lineTo(blue_StackCloseWaypointPos.vec())
//
//                                        //score cone
//                                        .back(0.10)
//                                        .splineToSplineHeading(blue_MedPolePos, Math.toRadians(32))
//                                        .back(-0.10)
//                                        .splineToSplineHeading(blue_StackCloseWaypointPos, Math.toRadians(180))

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