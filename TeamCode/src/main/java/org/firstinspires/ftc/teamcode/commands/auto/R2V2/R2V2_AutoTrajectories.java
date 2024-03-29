package org.firstinspires.ftc.teamcode.commands.auto.R2V2;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.DriveConstants_R2V2;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive_R2V2;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class R2V2_AutoTrajectories {
//    public static final Pose2d blue_StartPos = new Pose2d(-30.5, 64, Math.toRadians(90)); 2023-03-04
    public static final Pose2d blue_StartPos = new Pose2d(-40.5, 64, Math.toRadians(90));
    public static final Pose2d blue_MedPreloadPolePos = new Pose2d(-27, 31, Math.toRadians(140));//-35//130
    public static final Pose2d blue_MedPolePos = new Pose2d(-29.5, 21.5, Math.toRadians(220)); //-30.5, 21
    public static final Pose2d blue_StackPos = new Pose2d(-61.5, 11.5, Math.toRadians(180));//13.5//-63 //(-59.5, 12)

    public static final Pose2d red_StartPos = new Pose2d(40.5, 64, Math.toRadians(90));
    public static final Pose2d red_MedPreloadPolePos = new Pose2d(30, 29, Math.toRadians(40)); // (30, 16.5) //332 //90
    public static final Pose2d red_MedPolePos = new Pose2d(30.5, 18, Math.toRadians(328)); // (30, 16.5) //332 // (32, 20)  //TODO: -32?
    public static final Pose2d red_StackPos = new Pose2d(62, 10, Math.toRadians(0)); // (62.5, 9)

    // DPERECATED POSITIONS
    public static final Pose2d blue_PreloadPolePos = new Pose2d(-25, 7, Math.toRadians(140));//-35//130
    public static final Pose2d blue_MainPolePos = new Pose2d(-5, 22.5, Math.toRadians(212));
    public static final Pose2d blue_StackFarWaypointPos = new Pose2d(-38, 11, Math.toRadians(180));
    public static final Pose2d blue_StackWaypointPos = new Pose2d(-50, 13, Math.toRadians(180));

    public static final Pose2d red_PreloadPolePos = new Pose2d(32, 4, Math.toRadians(40)); //(32, 2)
    public static final Pose2d red_MainPolePos = new Pose2d(5, 22.5, Math.toRadians(302)); //
    public static final Pose2d red_StackFarWaypointPos = new Pose2d(38, 11, Math.toRadians(0));
    public static final Pose2d red_StackWaypointPos = new Pose2d(50, 12, Math.toRadians(0)); // (50, 9)

    //deprecated trajectories
//    public static TrajectorySequence blue_StartToPreloadPoleWaypoint, blue_PreloadPoleWaypointToPreloadPole, blue_PreloadPoleToStackWaypoint,   blue_StackWaypointToStack,    red_PreloadPoleToStackWaypoint,  red_StackWaypointToStack,
//    red_StackCloseWaypointToMedPole, red_MedPoleToStack, red_StartToPreloadPoleWaypoint, red_PreloadPoleWaypointToPreloadPole;

    //1+4 trajectories
    public static TrajectorySequence blue_StartToPreloadPole, blue_PreloadPoleToStackWaypoint, blue_StackWaypointToStack, blue_StackToStackWaypoint, blue_StackWaypointToMedPole, blue_MedPoleToStackWaypoint, blue_MedPreloadPoleToStack, blue_MedPoleToStack, blue_MedPoleToStackSlow;
    public static TrajectorySequence red_StartToPreloadPole, red_PreloadPoleToStackWaypoint, red_StackWaypointToStack, red_StackToStackWaypoint, red_StackWaypointToMedPole, red_MedPoleToStack, red_MedPreloadPoleToStack, red_MedPoleToStackSlow;

    public static TrajectorySequence red_MedPoleToStackWaypoint;

    //experimental 1+5 trajectories
    public static TrajectorySequence blue_StackToMedPole, blue_StartToMedPreloadPole, blue_MedPreloadPoleToStackWaypoint, red_StackToMedPole, red_StartToMedPreloadPole, red_MedPreloadPoleToStackWaypoint;

    //old high pole trajectories
    public static TrajectorySequence  blue_MainPoleToStackWaypoint, blue_StackWaypointToMainPole;
    public static TrajectorySequence red_MainPoleToStackWaypoint, red_StackWaypointToMainPole;


    public static void generateTrajectories(SampleMecanumDrive_R2V2 drive) {
        // 🟦🟦🟦 BLUE AUTO COMMANDS 🟦🟦🟦
        blue_StartToMedPreloadPole =
                drive.trajectorySequenceBuilder(blue_StartPos)
                        .setReversed(true)
//                        .lineTo(new Vector2d(-33, 62), SampleMecanumDrive_R2V2.getVelocityConstraint(86, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
//                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
//                        .lineTo(new Vector2d(-33, 60), SampleMecanumDrive_R2V2.getVelocityConstraint(86, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
//                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
//                        .splineTo(blue_MedPreloadPolePos.vec(), Math.toRadians(300), SampleMecanumDrive_R2V2.getVelocityConstraint(40, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
//                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .lineTo(new Vector2d(-38, 62), SampleMecanumDrive_R2V2.getVelocityConstraint(86, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .lineTo(new Vector2d(-38, 60), SampleMecanumDrive_R2V2.getVelocityConstraint(86, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .splineTo(blue_MedPreloadPolePos.vec(), Math.toRadians(310), SampleMecanumDrive_R2V2.getVelocityConstraint(40, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .setReversed(false)
                        .build();

        blue_MedPreloadPoleToStackWaypoint =
                drive.trajectorySequenceBuilder(blue_MedPreloadPolePos)
                        .back(-0.10)
                        .lineToLinearHeading(new Pose2d(-35, 15, Math.toRadians(200)), SampleMecanumDrive_R2V2.getVelocityConstraint(30, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .splineToSplineHeading(blue_StackWaypointPos, Math.toRadians(180), SampleMecanumDrive_R2V2.getVelocityConstraint(60, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .build();

        blue_MedPreloadPoleToStack =
                drive.trajectorySequenceBuilder(blue_MedPreloadPolePos)
                        .back(-0.10)
                        .lineToLinearHeading(new Pose2d(-35, 15, Math.toRadians(200)), SampleMecanumDrive_R2V2.getVelocityConstraint(50, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .splineToSplineHeading(blue_StackPos, Math.toRadians(180), SampleMecanumDrive_R2V2.getVelocityConstraint(60, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .build();

        blue_StackToMedPole =
                drive.trajectorySequenceBuilder(blue_StackPos)
                        .back(0.10)
                        .splineToSplineHeading(blue_MedPolePos, Math.toRadians(32), SampleMecanumDrive_R2V2.getVelocityConstraint(40, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(30))
                        .build();

        blue_MedPoleToStack =
                drive.trajectorySequenceBuilder(blue_MedPolePos)
                        .back(-0.10)
                        .splineToSplineHeading(blue_StackPos, Math.toRadians(180), SampleMecanumDrive_R2V2.getVelocityConstraint(35, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .build();

        blue_MedPoleToStackSlow =
                drive.trajectorySequenceBuilder(blue_MedPolePos)
                        .back(-0.10)
                        .splineToSplineHeading(blue_StackPos, Math.toRadians(180), SampleMecanumDrive_R2V2.getVelocityConstraint(25, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .build();


        // 🟥🟥🟥 red auto commands 🟥🟥🟥
        red_StartToMedPreloadPole =
                drive.trajectorySequenceBuilder(red_StartPos)
                        .setReversed(true)
                        .lineTo(new Vector2d(38, 62), SampleMecanumDrive_R2V2.getVelocityConstraint(86, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .lineTo(new Vector2d(38, 60), SampleMecanumDrive_R2V2.getVelocityConstraint(86, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .splineTo(red_MedPreloadPolePos.vec(), Math.toRadians(230), SampleMecanumDrive_R2V2.getVelocityConstraint(40, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .setReversed(false)
                        .build();

        red_MedPreloadPoleToStackWaypoint =
                drive.trajectorySequenceBuilder(red_MedPreloadPolePos)
                        .back(-0.10)
                        .lineToLinearHeading(new Pose2d(35, 15, Math.toRadians(340)), SampleMecanumDrive_R2V2.getVelocityConstraint(30, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .splineToSplineHeading(red_StackWaypointPos, Math.toRadians(0), SampleMecanumDrive_R2V2.getVelocityConstraint(60, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .build();

        red_MedPreloadPoleToStack =
                drive.trajectorySequenceBuilder(red_MedPreloadPolePos)
                        .back(-0.10)
                        .lineToLinearHeading(new Pose2d(35, 15, Math.toRadians(340)), SampleMecanumDrive_R2V2.getVelocityConstraint(50, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .splineToSplineHeading(red_StackPos, Math.toRadians(0), SampleMecanumDrive_R2V2.getVelocityConstraint(60, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .build();

        red_StackToMedPole =
                drive.trajectorySequenceBuilder(red_StackPos)
                        .back(0.10)
                        .splineToSplineHeading(red_MedPolePos, Math.toRadians(148), SampleMecanumDrive_R2V2.getVelocityConstraint(40, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(30))
                        .build();

        red_MedPoleToStack =
                drive.trajectorySequenceBuilder(red_MedPolePos)
                        .back(-0.10)
                        .splineToSplineHeading(red_StackPos, Math.toRadians(0), SampleMecanumDrive_R2V2.getVelocityConstraint(35, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .build();

        red_MedPoleToStackSlow =
                drive.trajectorySequenceBuilder(red_MedPolePos)
                        .back(-0.10)
                        .splineToSplineHeading(red_StackPos, Math.toRadians(0), SampleMecanumDrive_R2V2.getVelocityConstraint(25, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .build();


        // 🟦⬜🟦 BLUE EXPERIMENTAL/OLD  🟦⬜🟦
        blue_StartToPreloadPole =
                drive.trajectorySequenceBuilder(blue_StartPos)
                        .setReversed(true)
                        .lineTo(new Vector2d(-31, 63), SampleMecanumDrive_R2V2.getVelocityConstraint(30, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .splineToConstantHeading(new Vector2d(-33, 20), Math.toRadians(270), SampleMecanumDrive_R2V2.getVelocityConstraint(86, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .splineToLinearHeading(blue_PreloadPolePos, Math.toRadians(340), SampleMecanumDrive_R2V2.getVelocityConstraint(50, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .setReversed(false)
                        .build();

        blue_PreloadPoleToStackWaypoint =
                drive.trajectorySequenceBuilder(blue_PreloadPolePos)
                        .back(-0.10)
                        .splineToLinearHeading(blue_StackWaypointPos, Math.toRadians(180), SampleMecanumDrive_R2V2.getVelocityConstraint(60, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .build();

        blue_StackToStackWaypoint =
                drive.trajectorySequenceBuilder(blue_StackPos)
                        .back(-0.10)
                        .lineTo(blue_StackWaypointPos.vec(), SampleMecanumDrive_R2V2.getVelocityConstraint(40, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
//                        .lineTo(blue_StackFarWaypointPos.vec())
                        .build();

        blue_StackWaypointToMedPole = //HERE
                drive.trajectorySequenceBuilder(blue_StackWaypointPos)
                        .back(0.10)
                        .splineToSplineHeading(blue_MedPolePos, Math.toRadians(30), SampleMecanumDrive_R2V2.getVelocityConstraint(20, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .build();

        blue_MainPoleToStackWaypoint =
                drive.trajectorySequenceBuilder(blue_MainPolePos)
                        .back(-0.10)
                        .splineToSplineHeading(blue_StackFarWaypointPos, Math.toRadians(180), SampleMecanumDrive_R2V2.getVelocityConstraint(30, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .build();

        blue_StackWaypointToStack =
                drive.trajectorySequenceBuilder(blue_StackWaypointPos)
                        .back(-0.10)
                        .lineTo(blue_StackPos.vec(), SampleMecanumDrive_R2V2.getVelocityConstraint(40, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .build();

        blue_MedPoleToStackWaypoint =
                drive.trajectorySequenceBuilder(blue_MedPolePos)
                        .back(-0.10)
                        .splineToSplineHeading(blue_StackWaypointPos, Math.toRadians(180), SampleMecanumDrive_R2V2.getVelocityConstraint(86, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .build();



        // 🟥⬜🟥 RED EXPERIMENTAL/OLD 🟥⬜🟥
        red_StartToPreloadPole =
                drive.trajectorySequenceBuilder(red_StartPos)
                        .setReversed(true)
                        .lineTo(new Vector2d(40, 63), SampleMecanumDrive_R2V2.getVelocityConstraint(30, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .splineToConstantHeading(new Vector2d(33, 20), Math.toRadians(270), SampleMecanumDrive_R2V2.getVelocityConstraint(86, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL)) // (36, 7)
                        .splineToLinearHeading(red_PreloadPolePos, Math.toRadians(250), SampleMecanumDrive_R2V2.getVelocityConstraint(50, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL)) //TODO: test 200 end tangent
                        .setReversed(false)
                        .build();

        red_PreloadPoleToStackWaypoint =
                drive.trajectorySequenceBuilder(red_PreloadPolePos)
                        .back(-0.10)
                        .splineToLinearHeading(red_StackWaypointPos, Math.toRadians(0), SampleMecanumDrive_R2V2.getVelocityConstraint(60, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL)) //TODO: test 90 end tangent
                        .build();
        red_StackToStackWaypoint =
                drive.trajectorySequenceBuilder(red_StackPos)
                        .back(-0.10)
                        .lineTo(red_StackWaypointPos.vec(), SampleMecanumDrive_R2V2.getVelocityConstraint(40, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
//                        .lineTo(blue_StackFarWaypointPos.vec())
                        .build();

        red_StackWaypointToMedPole =
                drive.trajectorySequenceBuilder(red_StackWaypointPos)
                        .back(0.10)
                        .splineToSplineHeading(red_MedPolePos, Math.toRadians(148), SampleMecanumDrive_R2V2.getVelocityConstraint(60, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .build();

        red_StackWaypointToMainPole =
                drive.trajectorySequenceBuilder(red_StackWaypointPos)
                        .back(0.10)
                        .splineToSplineHeading(red_MainPolePos, Math.toRadians(122), SampleMecanumDrive_R2V2.getVelocityConstraint(30, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .build();

        red_MainPoleToStackWaypoint =
                drive.trajectorySequenceBuilder(red_MainPolePos)
                        .back(-0.10)
                        .splineToSplineHeading(red_StackFarWaypointPos, Math.toRadians(180), SampleMecanumDrive_R2V2.getVelocityConstraint(30, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .build();


        red_MedPoleToStackWaypoint =
                drive.trajectorySequenceBuilder(red_MedPolePos)
                        .back(-0.10)
                        .splineToSplineHeading(red_StackWaypointPos, Math.toRadians(0), SampleMecanumDrive_R2V2.getVelocityConstraint(86, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .build();
        red_StackWaypointToStack =
                drive.trajectorySequenceBuilder(red_StackWaypointPos)
                        .back(-0.10)
                        .lineTo(red_StackPos.vec(), SampleMecanumDrive_R2V2.getVelocityConstraint(40, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .build();

        // OLD/DEPRECATED TRAJECTORIES
/*


//        blue_PreloadPoleToStack =
//                drive.trajectorySequenceBuilder(blue_PreloadPolePos)
//                        .back(-0.10)
//                        .splineToLinearHeading(blue_StackPos, Math.toRadians(180), SampleMecanumDrive_R2V2.getVelocityConstraint(60, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
//                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
//                        .build();



//        blue_StartToPreloadPoleWaypoint =
//                drive.trajectorySequenceBuilder(blue_StartPos)
//                        .setReversed(true)
//                        .lineTo(new Vector2d(-31, 63), SampleMecanumDrive_R2V2.getVelocityConstraint(30, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
//                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
//                        .splineToConstantHeading(new Vector2d(-33, 20), Math.toRadians(270), SampleMecanumDrive_R2V2.getVelocityConstraint(86, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
//                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
//                        .setReversed(false)
//                        .build();
//        blue_StartToPreloadPoleWaypoint =
//                drive.trajectorySequenceBuilder(blue_StartPos)
//                        .setReversed(true)
//                        .lineTo(new Vector2d(-31, 63), SampleMecanumDrive_R2V2.getVelocityConstraint(30, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
//                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
//                        .splineToConstantHeading(new Vector2d(-33, 7), Math.toRadians(270), SampleMecanumDrive_R2V2.getVelocityConstraint(60, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
//                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
//                        .setReversed(false)
//                        .build();

//        blue_PreloadPoleWaypointToPreloadPole =
//                drive.trajectorySequenceBuilder(blue_StartToPreloadPoleWaypoint.end())
//                        .setReversed(true)
////                        .lineTo(new Vector2d(-33, 15), SampleMecanumDrive_R2V2.getVelocityConstraint(30, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
////                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
//                        .splineToLinearHeading(blue_PreloadPolePos, Math.toRadians(340), SampleMecanumDrive_R2V2.getVelocityConstraint(50, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
//                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
//
/////
//                        .setReversed(false)
//                        .build();
//
//        blue_PreloadPoleToStackWaypoint =
//                drive.trajectorySequenceBuilder(blue_PreloadPoleWaypointToPreloadPole.end())
//                        .back(-0.10)
//                        .splineToLinearHeading(blue_StackFarWaypointPos, Math.toRadians(180), SampleMecanumDrive_R2V2.getVelocityConstraint(30, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
//                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
//                        .build();



//        blue_StackWaypointToStack =
//                drive.trajectorySequenceBuilder(blue_StackFarWaypointPos)
//                        .back(-0.10)
//                        .lineTo(blue_StackPos.vec(), SampleMecanumDrive_R2V2.getVelocityConstraint(10, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
//                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
//                        .build();

//        blue_StackWaypointToMainPole =
//                drive.trajectorySequenceBuilder(blue_StackCloseWaypointPos)
//                        .back(0.10)
//                        .splineToSplineHeading(blue_MainPolePos, Math.toRadians(32))
//                        .build();
//        red_StartToPreloadPoleWaypoint =
//                drive.trajectorySequenceBuilder(red_StartPos)
//                        .setReversed(true)
//                        .lineTo(new Vector2d(40, 63), SampleMecanumDrive_R2V2.getVelocityConstraint(30, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
//                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
//                        .splineToConstantHeading(new Vector2d(36, 20), Math.toRadians(270), SampleMecanumDrive_R2V2.getVelocityConstraint(86, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
//                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
//                         .setReversed(false)
//                        .build();
//        red_PreloadPoleWaypointToPreloadPole =
//                drive.trajectorySequenceBuilder(red_StartToPreloadPoleWaypoint.end())
//                        .setReversed(true)
////                        .lineTo(new Vector2d(36, 14), SampleMecanumDrive_R2V2.getVelocityConstraint(30, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
////                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
//                        .splineToLinearHeading(red_PreloadPolePos, Math.toRadians(250), SampleMecanumDrive_R2V2.getVelocityConstraint(50, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
//                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL)) //TODO: test 200 end tangent
//                        .setReversed(false)
//                        .build();
//
//
//        red_PreloadPoleToStackWaypoint =
//                drive.trajectorySequenceBuilder(red_PreloadPolePos)
//                        .back(-0.10)
//                        .splineToLinearHeading(red_StackFarWaypointPos, Math.toRadians(90), SampleMecanumDrive_R2V2.getVelocityConstraint(30, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
//                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
//                        .build();

//



//        red_MedPoleToStack =
//                drive.trajectorySequenceBuilder(red_MedPolePos)
//                        .back(-0.10)
//                        .splineToSplineHeading(red_StackPos, Math.toRadians(180), SampleMecanumDrive_R2V2.getVelocityConstraint(30, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
//                                SampleMecanumDrive_R2V2.getAccelerationConstraint(20)) //TODO: test stack to pole
//                        .build();
////
//        red_StackWaypointToStack =
//                drive.trajectorySequenceBuilder(red_StackFarWaypointPos)
//                        .back(-0.10)
//                        .lineTo(red_StackPos.vec(), SampleMecanumDrive_R2V2.getVelocityConstraint(10, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
//                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
//                        .build();


//
//

//        red_StackCloseWaypointToMedPole =
//                drive.trajectorySequenceBuilder(red_StackCloseWaypointPos)
//                        .back(0.10)
//                        .splineToSplineHeading(red_MedPolePos, Math.toRadians(180), SampleMecanumDrive_R2V2.getVelocityConstraint(30, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
//                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
//                        .build();*/

    }

}