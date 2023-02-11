package org.firstinspires.ftc.teamcode.commands.auto.R2V2;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.DriveConstants_R2V2;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive_R2V2;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class R2V2_AutoTrajectories {

    /*
        _StartToPreloadPole
        _PreloadPoleToStack
        x5 {
            _StackToMainPole 
            _MainPoleToStack
        }
    */

    public static final Pose2d blue_StartPos = new Pose2d(-30.5, 64, Math.toRadians(90));
    public static final Pose2d blue_PreloadPolePos = new Pose2d(-26, 6, Math.toRadians(140));//-35//130
    public static final Pose2d blue_MainPolePos = new Pose2d(-5, 22.5, Math.toRadians(212));
    public static final Pose2d blue_MedPolePos = new Pose2d(-29, 22, Math.toRadians(212));
    public static final Pose2d blue_StackFarWaypointPos = new Pose2d(-38, 11, Math.toRadians(180));
    public static final Pose2d blue_StackCloseWaypointPos = new Pose2d(-50, 12, Math.toRadians(180));
    public static final Pose2d blue_StackPos = new Pose2d(-59, 12, Math.toRadians(180));//-63

    //red auto positions
//    public static final Pose2d red_StartPos = new Pose2d(41, 64, Math.toRadians(90));
//
//    public static final Pose2d red_PreloadPolePos = new Pose2d(31, 4, Math.toRadians(20));
//
//    public static final Pose2d red_MainPolePos = new Pose2d(4, 21, Math.toRadians(32));//32
//
//    //public static final Pose2d red_StackFarWaypointPos = new Pose2d(38, 14, Math.toRadians(0)); x -58
//    public static final Pose2d red_StackFarWaypointPos = new Pose2d(38, 14, Math.toRadians(0));
//    public static final Pose2d red_StackCloseWaypointPos = new Pose2d(50, 14, Math.toRadians(0)); // x -58
//    public static final Pose2d red_StackPos = new Pose2d(56, 14, Math.toRadians(0)); // x -58

    public static final Pose2d red_StartPos = new Pose2d(40.5, 64, Math.toRadians(90));
    public static final Pose2d red_PreloadPolePos = new Pose2d(32, 2, Math.toRadians(40));
    public static final Pose2d red_MainPolePos = new Pose2d(5, 22.5, Math.toRadians(302));
    public static final Pose2d red_MedPolePos = new Pose2d(30, 16.5, Math.toRadians(332));
    public static final Pose2d red_StackFarWaypointPos = new Pose2d(38, 11, Math.toRadians(0));
    public static final Pose2d red_StackCloseWaypointPos = new Pose2d(50, 9, Math.toRadians(0));
    public static final Pose2d red_StackPos = new Pose2d(62.5, 9, Math.toRadians(0));

    public static TrajectorySequence blue_StartToPreloadPole, blue_StartToPreloadPoleWaypoint, blue_PreloadPoleWaypointToPreloadPole, blue_PreloadPoleToStackWaypoint, blue_PreloadPoleToStackCloseWaypoint, blue_MainPoleToStackWaypoint, blue_MedPoleToStackWaypoint, blue_StackWaypointToStack, blue_StackCloseWaypointToStack, blue_StackToStackWaypoint, blue_StackWaypointToMainPole,  blue_StackWaypointToMedPole,
    red_StartToPreloadPole, red_PreloadPoleToStackWaypoint, red_MainPoleToStackWaypoint, red_StackWaypointToStack, red_StackToStackWaypoint, red_StackWaypointToMainPole, red_MedPoleToStackWaypoint, red_PreloadPoleToStackCloseWaypoint, red_StackCloseWaypointToStack,
    red_StackCloseWaypointToMedPole, red_StackWaypointToMedPole, red_MedPoleToStack, red_StartToPreloadPoleWaypoint, red_PreloadPoleWaypointToPreloadPole;


    public static void generateTrajectories(SampleMecanumDrive_R2V2 drive) {
        //TODO: remove this step from other programs
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

        blue_StartToPreloadPoleWaypoint =
                drive.trajectorySequenceBuilder(blue_StartPos)
                        .setReversed(true)
                        .lineTo(new Vector2d(-31, 63), SampleMecanumDrive_R2V2.getVelocityConstraint(30, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .splineToConstantHeading(new Vector2d(-33, 20), Math.toRadians(270), SampleMecanumDrive_R2V2.getVelocityConstraint(86, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .setReversed(false)
                        .build();
//        blue_StartToPreloadPoleWaypoint =
//                drive.trajectorySequenceBuilder(blue_StartPos)
//                        .setReversed(true)
//                        .lineTo(new Vector2d(-31, 63), SampleMecanumDrive_R2V2.getVelocityConstraint(30, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
//                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
//                        .splineToConstantHeading(new Vector2d(-33, 7), Math.toRadians(270), SampleMecanumDrive_R2V2.getVelocityConstraint(60, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
//                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
//                        .setReversed(false)
//                        .build();

        blue_PreloadPoleWaypointToPreloadPole =
                drive.trajectorySequenceBuilder(blue_StartToPreloadPoleWaypoint.end())
                        .setReversed(true)
//                        .lineTo(new Vector2d(-33, 15), SampleMecanumDrive_R2V2.getVelocityConstraint(30, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
//                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .splineToLinearHeading(blue_PreloadPolePos, Math.toRadians(340), SampleMecanumDrive_R2V2.getVelocityConstraint(50, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))

///
                        .setReversed(false)
                        .build();

        blue_PreloadPoleToStackWaypoint =
                drive.trajectorySequenceBuilder(blue_PreloadPoleWaypointToPreloadPole.end())
                        .back(-0.10)
                        .splineToLinearHeading(blue_StackFarWaypointPos, Math.toRadians(180), SampleMecanumDrive_R2V2.getVelocityConstraint(30, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .build();


        blue_PreloadPoleToStackCloseWaypoint =
                drive.trajectorySequenceBuilder(blue_PreloadPolePos)
                        .back(-0.10)
                        .splineToLinearHeading(blue_StackCloseWaypointPos, Math.toRadians(180), SampleMecanumDrive_R2V2.getVelocityConstraint(60, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .build();

        blue_MainPoleToStackWaypoint =
                drive.trajectorySequenceBuilder(blue_MainPolePos)
                        .back(-0.10)
                        .splineToSplineHeading(blue_StackFarWaypointPos, Math.toRadians(180), SampleMecanumDrive_R2V2.getVelocityConstraint(30, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .build();


        blue_MedPoleToStackWaypoint =
                drive.trajectorySequenceBuilder(blue_MedPolePos)
                        .back(-0.10)
                        .splineToSplineHeading(blue_StackCloseWaypointPos, Math.toRadians(180), SampleMecanumDrive_R2V2.getVelocityConstraint(60, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .build();

        blue_StackWaypointToStack =
                drive.trajectorySequenceBuilder(blue_StackFarWaypointPos)
                        .back(-0.10)
                        .lineTo(blue_StackPos.vec(), SampleMecanumDrive_R2V2.getVelocityConstraint(10, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .build();

        blue_StackCloseWaypointToStack =
                drive.trajectorySequenceBuilder(blue_StackCloseWaypointPos)
                        .back(-0.10)
                        .lineTo(blue_StackPos.vec(), SampleMecanumDrive_R2V2.getVelocityConstraint(20, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .build();

        blue_StackToStackWaypoint =
                drive.trajectorySequenceBuilder(blue_StackPos)
                        .back(-0.10)
                        .lineTo(blue_StackCloseWaypointPos.vec(), SampleMecanumDrive_R2V2.getVelocityConstraint(20, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
//                        .lineTo(blue_StackFarWaypointPos.vec())
                        .build();

//        blue_StackWaypointToMainPole =
//                drive.trajectorySequenceBuilder(blue_StackCloseWaypointPos)
//                        .back(0.10)
//                        .splineToSplineHeading(blue_MainPolePos, Math.toRadians(32))
//                        .build();
        blue_StackWaypointToMedPole =
                drive.trajectorySequenceBuilder(blue_StackCloseWaypointPos)
                        .back(0.10)
                        .splineToSplineHeading(blue_MedPolePos, Math.toRadians(32), SampleMecanumDrive_R2V2.getVelocityConstraint(60, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .build();

        //red auto commands
        red_StartToPreloadPole =
                drive.trajectorySequenceBuilder(red_StartPos)
                        .setReversed(true)
//                        .lineTo(new Vector2d(36, 54))
//                        .lineTo(new Vector2d(36, 7), SampleMecanumDrive_R2V2.getVelocityConstraint(62, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
//                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
//                        .lineTo(new Vector2d(36, 20))
                        .lineTo(new Vector2d(40, 63), SampleMecanumDrive_R2V2.getVelocityConstraint(30, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .splineToConstantHeading(new Vector2d(36, 7), Math.toRadians(270), SampleMecanumDrive_R2V2.getVelocityConstraint(30, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .lineTo(new Vector2d(36, 14), SampleMecanumDrive_R2V2.getVelocityConstraint(30, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .splineToLinearHeading(red_PreloadPolePos, Math.toRadians(250), SampleMecanumDrive_R2V2.getVelocityConstraint(20, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL)) //TODO: test 200 end tangent
                        .setReversed(false)
                        .build();
//

        red_StartToPreloadPoleWaypoint =
                drive.trajectorySequenceBuilder(red_StartPos)
                        .setReversed(true)
                        .lineTo(new Vector2d(40, 63), SampleMecanumDrive_R2V2.getVelocityConstraint(30, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .splineToConstantHeading(new Vector2d(36, 20), Math.toRadians(270), SampleMecanumDrive_R2V2.getVelocityConstraint(86, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                         .setReversed(false)
                        .build();
        red_PreloadPoleWaypointToPreloadPole =
                drive.trajectorySequenceBuilder(red_StartToPreloadPoleWaypoint.end())
                        .setReversed(true)
//                        .lineTo(new Vector2d(36, 14), SampleMecanumDrive_R2V2.getVelocityConstraint(30, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
//                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .splineToLinearHeading(red_PreloadPolePos, Math.toRadians(250), SampleMecanumDrive_R2V2.getVelocityConstraint(50, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL)) //TODO: test 200 end tangent
                        .setReversed(false)
                        .build();
//
//
        red_PreloadPoleToStackWaypoint =
                drive.trajectorySequenceBuilder(red_PreloadPolePos)
                        .back(-0.10)
                        .splineToLinearHeading(red_StackFarWaypointPos, Math.toRadians(90), SampleMecanumDrive_R2V2.getVelocityConstraint(30, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .build();

        red_PreloadPoleToStackCloseWaypoint =
                drive.trajectorySequenceBuilder(red_PreloadPolePos)
                        .back(-0.10)
                        .splineToLinearHeading(red_StackCloseWaypointPos, Math.toRadians(0), SampleMecanumDrive_R2V2.getVelocityConstraint(60, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL)) //TODO: test 90 end tangent
                        .build();
//
        red_MainPoleToStackWaypoint =
                drive.trajectorySequenceBuilder(red_MainPolePos)
                        .back(-0.10)
                        .splineToSplineHeading(red_StackFarWaypointPos, Math.toRadians(180), SampleMecanumDrive_R2V2.getVelocityConstraint(30, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .build();

        red_MedPoleToStackWaypoint =
                drive.trajectorySequenceBuilder(red_MedPolePos)
                        .back(-0.10)
                        .splineToSplineHeading(red_StackCloseWaypointPos, Math.toRadians(180), SampleMecanumDrive_R2V2.getVelocityConstraint(60, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .build();

        red_MedPoleToStack =
                drive.trajectorySequenceBuilder(red_MedPolePos)
                        .back(-0.10)
                        .splineToSplineHeading(red_StackPos, Math.toRadians(180), SampleMecanumDrive_R2V2.getVelocityConstraint(30, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(20)) //TODO: test stack to pole
                        .build();
//
        red_StackWaypointToStack =
                drive.trajectorySequenceBuilder(red_StackFarWaypointPos)
                        .back(-0.10)
                        .lineTo(red_StackPos.vec(), SampleMecanumDrive_R2V2.getVelocityConstraint(10, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .build();

        red_StackCloseWaypointToStack =
                drive.trajectorySequenceBuilder(red_StackCloseWaypointPos)
                        .back(-0.10)
                        .lineTo(red_StackPos.vec(), SampleMecanumDrive_R2V2.getVelocityConstraint(20, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .build();
//
        red_StackToStackWaypoint =
                drive.trajectorySequenceBuilder(red_StackPos)
                        .back(-0.10)
                        .lineTo(red_StackCloseWaypointPos.vec(), SampleMecanumDrive_R2V2.getVelocityConstraint(20, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
//                        .lineTo(blue_StackFarWaypointPos.vec())
                        .build();
//
        red_StackWaypointToMainPole =
                drive.trajectorySequenceBuilder(red_StackCloseWaypointPos)
                        .back(0.10)
                        .splineToSplineHeading(red_MainPolePos, Math.toRadians(122), SampleMecanumDrive_R2V2.getVelocityConstraint(30, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .build();

        red_StackCloseWaypointToMedPole =
                drive.trajectorySequenceBuilder(red_StackCloseWaypointPos)
                        .back(0.10)
                        .splineToSplineHeading(red_MedPolePos, Math.toRadians(180), SampleMecanumDrive_R2V2.getVelocityConstraint(30, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .build();

        red_StackWaypointToMedPole =
                drive.trajectorySequenceBuilder(red_StackCloseWaypointPos)
                        .back(0.10)
                        .splineToSplineHeading(red_MedPolePos, Math.toRadians(180), SampleMecanumDrive_R2V2.getVelocityConstraint(60, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                                SampleMecanumDrive_R2V2.getAccelerationConstraint(DriveConstants_R2V2.MAX_ACCEL))
                        .build();
    }

}