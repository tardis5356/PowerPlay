package org.firstinspires.ftc.teamcode.commands.auto;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive_Barney;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class BarneyAutoTrajectories {

    /*
        _StartToPreloadPole
        _PreloadPoleToStack
        x5 {
            _StackToMainPole 
            _MainPoleToStack
        }
    */

    public static final Pose2d blue_StartPos = new Pose2d(-36, 64, Math.toRadians(90));

    public static final Pose2d blue_PreloadPolePos = new Pose2d(-32, 3, Math.toRadians(160));

    public static final Pose2d blue_MainPolePos = new Pose2d(-8, 18, Math.toRadians(212));

    public static final Pose2d blue_StackWaypointPos = new Pose2d(-50, 15, Math.toRadians(180)); // x -58
    public static final Pose2d blue_StackPos = new Pose2d(-58, 15, Math.toRadians(180)); // x -58

    public static TrajectorySequence blue_StartToPreloadPole, red_StartToPreloadPole, blue_PreloadPoleToStack, red_PreloadPoleToStack, blue_MainPoleToStack, red_PoleToStack, blue_StackToMainPole, red_StackToPole;

    public static void generateTrajectories(SampleMecanumDrive_Barney drive) {
        blue_StartToPreloadPole =
                drive.trajectorySequenceBuilder(blue_StartPos)
                        .setReversed(true)
                        .splineTo(new Vector2d(-36, 20), Math.toRadians(270))
                        .splineToLinearHeading(blue_PreloadPolePos, Math.toRadians(340))
                        .setReversed(false)
                        .build();

        blue_PreloadPoleToStack =
                drive.trajectorySequenceBuilder(blue_PreloadPolePos)
                        .back(-0.10)
                        .splineToSplineHeading(blue_StackWaypointPos, Math.toRadians(180))
                        .waitSeconds(0.5)
                        .splineToSplineHeading(blue_StackPos, Math.toRadians(180))
                        .build();

        blue_MainPoleToStack =
                drive.trajectorySequenceBuilder(blue_MainPolePos)
                        .back(-0.10)
                        .splineToSplineHeading(blue_StackWaypointPos, Math.toRadians(180))
                        .waitSeconds(0.5)
                        .splineToSplineHeading(blue_StackPos, Math.toRadians(180))
                        .build();

        blue_StackToMainPole =
                drive.trajectorySequenceBuilder(blue_StackPos)
                        .back(0.10)
                        .splineToSplineHeading(blue_MainPolePos, Math.toRadians(32))
                        .build();
    }

}