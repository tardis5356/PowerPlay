package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive_Barney;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Timer;

@Autonomous(group = "drive")
public class test_1_5_Barney extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();
    double fullAutoTime = 30, cycleTime = 5, parkTime = 2;

    @Override


    public void runOpMode() {
        SampleMecanumDrive_Barney drive = new SampleMecanumDrive_Barney(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;
         //A2 starting
            Pose2d startPose = new Pose2d(-36, 66, Math.toRadians(90));
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

            TrajectorySequence deliverPreload = drive.trajectorySequenceBuilder(startPose)
                    .setReversed(true)
                    .splineTo(new Vector2d(-36, 20), Math.toRadians(270))
                    .splineToLinearHeading(new Pose2d(-32, 8, Math.toRadians(160)), Math.toRadians(340))
                    .setReversed(false)
                    .build();

            TrajectorySequence getFirstCone = drive.trajectorySequenceBuilder(deliverPreload.end())
                    .back(-0.10)
                    .splineToSplineHeading(new Pose2d(-55, 17, Math.toRadians(180)), Math.toRadians(180))
                    .build();

            TrajectorySequence cycle = drive.trajectorySequenceBuilder(getFirstCone.end())
                    .back(0.10)
                    .splineToSplineHeading(new Pose2d(-9, 23, Math.toRadians(212)), Math.toRadians(32))
                    .back(-0.10)
                    .splineToSplineHeading(new Pose2d(-55, 17, Math.toRadians(180)), Math.toRadians(180))
                    .build();
            TrajectorySequence parkZone2 = drive.trajectorySequenceBuilder(cycle.end())
                    .splineToSplineHeading(new Pose2d(-36, 12, Math.toRadians(180)), Math.toRadians(180))
                    .build();


        while (runtime.seconds() < fullAutoTime) {

            drive.followTrajectorySequence(deliverPreload);
            drive.followTrajectorySequence(getFirstCone);

            while (runtime.seconds() < fullAutoTime - cycleTime - parkTime) {

                drive.followTrajectorySequence(cycle);
            }

            drive.followTrajectorySequence(parkZone2);
            requestOpModeStop();

        }




    }

}

