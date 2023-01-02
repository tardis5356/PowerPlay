package org.firstinspires.ftc.teamcode.commands.auto.barney;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.DriveConstants_Barney;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive_Barney;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class Barney_AutoTrajectories {

    /*
        _StartToPreloadPole
        _PreloadPoleToStack
        x5 {
            _StackToMainPole 
            _MainPoleToStack
        }
    */

    public static final Pose2d blue_StartPos = new Pose2d(-36, 64, Math.toRadians(90));
    public static final Pose2d blue_StartPos2 = new Pose2d(36, 64, Math.toRadians(90));


    public static final Pose2d blue_PreloadPolePos = new Pose2d(-31, 6, Math.toRadians(160));
//    public static final Pose2d red_PreloadPolePos = new Pose2d(-41, 4, Math.toRadians(20));

    public static final Pose2d blue_MainPolePos = new Pose2d(-9, 21, Math.toRadians(212));

    public static final Pose2d blue_StackFarWaypointPos = new Pose2d(-38, 14, Math.toRadians(180)); // x -58
    public static final Pose2d blue_StackCloseWaypointPos = new Pose2d(-50, 14, Math.toRadians(180)); // x -58
    public static final Pose2d blue_StackPos = new Pose2d(-57, 15, Math.toRadians(180)); // x -58
    //keep changing stack pos number, not sure what the correct number should be

    //red trajectories
    public static final Pose2d red_StartPos = new Pose2d(36, 64, Math.toRadians(90));
//    public static final Pose2d red_StartPos2 = new Pose2d(-36, 64, Math.toRadians(90));


    public static final Pose2d red_PreloadPolePos = new Pose2d(31, 4, Math.toRadians(20));

    public static final Pose2d red_MainPolePos = new Pose2d(4, 21, Math.toRadians(-32));

    public static final Pose2d red_StackFarWaypointPos = new Pose2d(38, 14, Math.toRadians(0)); // x -58
    public static final Pose2d red_StackCloseWaypointPos = new Pose2d(50, 14, Math.toRadians(0)); // x -58
    public static final Pose2d red_StackPos = new Pose2d(56, 14, Math.toRadians(0)); // x -58


    public static TrajectorySequence blue_StartToPreloadPole, blue_PreloadPoleToStackWaypoint, blue_MainPoleToStackWaypoint, blue_StackWaypointToStack, blue_StackToStackWaypoint, blue_StackWaypointToMainPole,

    red_StartToPreloadPole, red_PreloadPoleToStackWaypoint, red_MainPoleToStackWaypoint, red_StackWaypointToStack, red_StackToStackWaypoint, red_StackWaypointToMainPole;

    public static void generateTrajectories(SampleMecanumDrive_Barney drive) {
        blue_StartToPreloadPole =
                drive.trajectorySequenceBuilder(blue_StartPos)
                        .setReversed(true)
//                        .splineTo(new Vector2d(-36, 20), Math.toRadians(270))
                        .lineTo(new Vector2d(-36, 7))
                        .lineTo(new Vector2d(-36, 20))
                        .splineToLinearHeading(blue_PreloadPolePos, Math.toRadians(340), SampleMecanumDrive_Barney.getVelocityConstraint(30, DriveConstants_Barney.MAX_ANG_VEL, DriveConstants_Barney.TRACK_WIDTH),
                                SampleMecanumDrive_Barney.getAccelerationConstraint(DriveConstants_Barney.MAX_ACCEL))
                        .setReversed(false)
                        .build();

        blue_PreloadPoleToStackWaypoint =
                drive.trajectorySequenceBuilder(blue_PreloadPolePos)
                        .back(-0.10)
                        .splineToLinearHeading(blue_StackFarWaypointPos, Math.toRadians(180))
                        .build();

        blue_MainPoleToStackWaypoint =
                drive.trajectorySequenceBuilder(blue_MainPolePos)
                        .back(-0.10)
                        .splineToSplineHeading(blue_StackFarWaypointPos, Math.toRadians(180))
                        .build();

        blue_StackWaypointToStack =
                drive.trajectorySequenceBuilder(blue_StackFarWaypointPos)
                        .back(-0.10)
                        .lineTo(blue_StackPos.vec(), SampleMecanumDrive_Barney.getVelocityConstraint(10, DriveConstants_Barney.MAX_ANG_VEL, DriveConstants_Barney.TRACK_WIDTH),
                                SampleMecanumDrive_Barney.getAccelerationConstraint(DriveConstants_Barney.MAX_ACCEL))
                        .build();

        blue_StackToStackWaypoint =
                drive.trajectorySequenceBuilder(blue_StackPos)
                        .back(-0.10)
                        .lineTo(blue_StackCloseWaypointPos.vec(), SampleMecanumDrive_Barney.getVelocityConstraint(10, DriveConstants_Barney.MAX_ANG_VEL, DriveConstants_Barney.TRACK_WIDTH),
                                SampleMecanumDrive_Barney.getAccelerationConstraint(DriveConstants_Barney.MAX_ACCEL))
//                        .lineTo(blue_StackFarWaypointPos.vec())
                        .build();

        blue_StackWaypointToMainPole =
                drive.trajectorySequenceBuilder(blue_StackCloseWaypointPos)
                        .back(0.10)
                        .splineToSplineHeading(blue_MainPolePos, Math.toRadians(32))
                        .build();


        ////////////////////////////////////////red trajectories//////////////////////////////////////////////////////////////
        red_StartToPreloadPole =
                drive.trajectorySequenceBuilder(red_StartPos)
                        .setReversed(true)
//                        .splineTo(new Vector2d(-36, 20), Math.toRadians(270))
                        .lineTo(new Vector2d(36, 7))
                        .lineTo(new Vector2d(36, 20))
                        .splineToLinearHeading(red_PreloadPolePos, Math.toRadians(250), SampleMecanumDrive_Barney.getVelocityConstraint(30, DriveConstants_Barney.MAX_ANG_VEL, DriveConstants_Barney.TRACK_WIDTH),
                                SampleMecanumDrive_Barney.getAccelerationConstraint(DriveConstants_Barney.MAX_ACCEL))
                        .setReversed(false)
                        .build();


        red_PreloadPoleToStackWaypoint =
                drive.trajectorySequenceBuilder(red_PreloadPolePos)
                        .back(-0.10)
                        .splineToLinearHeading(red_StackFarWaypointPos, Math.toRadians(90))
                        .build();

        red_MainPoleToStackWaypoint =
                drive.trajectorySequenceBuilder(red_MainPolePos)
                        .back(-0.10)
                        .splineToSplineHeading(red_StackFarWaypointPos, Math.toRadians(180))
                        .build();

        red_StackWaypointToStack =
                drive.trajectorySequenceBuilder(red_StackFarWaypointPos)
                        .back(-0.10)
                        .lineTo(red_StackPos.vec(), SampleMecanumDrive_Barney.getVelocityConstraint(10, DriveConstants_Barney.MAX_ANG_VEL, DriveConstants_Barney.TRACK_WIDTH),
                                SampleMecanumDrive_Barney.getAccelerationConstraint(DriveConstants_Barney.MAX_ACCEL))
                        .build();

        red_StackToStackWaypoint =
                drive.trajectorySequenceBuilder(red_StackPos)
                        .back(-0.10)
                        .lineTo(red_StackCloseWaypointPos.vec(), SampleMecanumDrive_Barney.getVelocityConstraint(10, DriveConstants_Barney.MAX_ANG_VEL, DriveConstants_Barney.TRACK_WIDTH),
                                SampleMecanumDrive_Barney.getAccelerationConstraint(DriveConstants_Barney.MAX_ACCEL))
//                        .lineTo(blue_StackFarWaypointPos.vec())
                        .build();

        red_StackWaypointToMainPole =
                drive.trajectorySequenceBuilder(red_StackCloseWaypointPos)
                        .back(0.10)
                        .splineToSplineHeading(red_MainPolePos, Math.toRadians(122))
                        .build();
    }

}