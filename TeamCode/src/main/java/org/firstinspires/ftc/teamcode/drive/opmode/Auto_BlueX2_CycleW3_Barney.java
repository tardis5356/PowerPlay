package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive_Barney;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class Auto_BlueX2_CycleW3_Barney extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive_Barney drive = new SampleMecanumDrive_Barney(hardwareMap);


        waitForStart();

        if (isStopRequested()) return;
        //A2 starting
        Pose2d startPose = new Pose2d(-36, 66, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        /*
        scoreX2 - score on x2
        X2ToCones - move from x2 (middle high junction) to a3 (cone stack)
        conesToA2 - strafe from a3 (cone stack) to a2 (next to cones)
        A2ToW3 - forwards from a2 (next to cones) to w3 (high junction)
        W3ToA3 - spline from w3 (high junction) to a3 (cone stack)
        park1
        park2
        park3
         */

        Trajectory scoreX2 = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-36, 0)) // score preload on x2
                .build();

        Trajectory X2ToCones = drive.trajectoryBuilder(scoreX2.end())
                .lineToConstantHeading(new Vector2d(-36, 3)) // strafes to in front of cones
                .splineToConstantHeading(new Vector2d(-60, 12), Math.toRadians(180)) // spline to cones
                .build();

        Trajectory conesToA2 = drive.trajectoryBuilder(X2ToCones.end())
                .lineToConstantHeading(new Vector2d(-60, 36)) // strafe from cones to a2 for forward movement to high junction
                .build();

        Trajectory A2ToW3 = drive.trajectoryBuilder(conesToA2.end())
                .lineToConstantHeading(new Vector2d(-24,36)) // forwards towards w3
                .splineToSplineHeading(new Pose2d(-7.5, 31, Math.toRadians(135)), Math.toRadians(310)) // spline to w3
                .build();

        Trajectory W3ToA3 = drive.trajectoryBuilder(A2ToW3.end())
                .splineToSplineHeading(new Pose2d(-7.5, 31, Math.toRadians(135)), Math.toRadians(310)) // spline to w3
                .build();

        Trajectory A3ToW3 = drive.trajectoryBuilder(conesToA2.end())
                .lineToConstantHeading(new Vector2d(-24,36)) // forwards towards w3
                .splineToSplineHeading(new Pose2d(-7.5, 31, Math.toRadians(135)), Math.toRadians(310)) // spline to w3
                .build();


        drive.followTrajectory(scoreX2);
//        drive.followTrajectory(X2ToCones);
//        drive.followTrajectory(conesToA2);
//        drive.followTrajectory(A2ToW3);
//        drive.followTrajectory(W3ToA3);
//        drive.followTrajectory(A3ToW3);

        /*
        scoreX2 - score on x2
        X2ToCones - move from x2 (middle high junction) to a3 (cone stack)
        conesToA2 - strafe from a3 (cone stack) to a2 (next to cones)
        A2ToW3 - forwards from a2 (next to cones) to w3 (high junction)
        W3ToA3 - spline from w3 (high junction) to a3 (cone stack)
        A3ToW3 - spline from  a3 (cone stack) to w3 (high junction)
        park1
        park2
        park3
         */

        sleep(2000);

    }
}