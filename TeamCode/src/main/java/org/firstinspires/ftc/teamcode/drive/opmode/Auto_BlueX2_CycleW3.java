package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class Auto_BlueX2_CycleW3 extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        waitForStart();

        if (isStopRequested()) return;
        //A2 starting
        Pose2d startPose = new Pose2d(-36, 66, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        Trajectory scoreX2 = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-36, 0)) // scoreX2
                .build();

        Trajectory strafeFromX2 = drive.trajectoryBuilder(scoreX2.end())
                .lineToConstantHeading(new Vector2d(-36, 3)) // strafeFromX2
                .splineToConstantHeading(new Vector2d(-60, 12), Math.toRadians(180)) // strafeFromX2
                .build();

//        Trajectory collectCone = drive.trajectoryBuilder(strafeFromX2.end())
//                //.splineToConstantHeading(new Vector2d(-36, 12), Math.toRadians(180))
//                .splineToConstantHeading(new Vector2d(-60, 12), Math.toRadians(180))
//                .build();

        Trajectory strafePrepW3 = drive.trajectoryBuilder(strafeFromX2.end())
                .lineToConstantHeading(new Vector2d(-60, 36)) // strafePrepW3
                .build();

        Trajectory goTowardsW3 = drive.trajectoryBuilder(strafePrepW3.end())
                .lineToConstantHeading(new Vector2d(-7.5,36)) // goTowardsW3
                .splineToSplineHeading(new Pose2d(-7.5, 31, Math.toRadians(135)), Math.toRadians(0)) // goTowardsW3
                .build();

//        Trajectory scoreW3 = drive.trajectoryBuilder(goTowardsW3.end())
//                .lineToLinearHeading(new Pose2d(-7.5, 31, Math.toRadians(135)))
//                .build();

        Trajectory resetRotation = drive.trajectoryBuilder(goTowardsW3.end())
                .lineToLinearHeading(new Pose2d(-7.5, 36, Math.toRadians(180))) // resetRotation
                .build();

        Trajectory goToA2 = drive.trajectoryBuilder(resetRotation.end())
                .lineToConstantHeading(new Vector2d(-60, 36)) // goToA2
                .build();

        Trajectory collectCone2 = drive.trajectoryBuilder(goToA2.end())
                .lineToConstantHeading(new Vector2d(-60, 12)) // collectCone2
                .build();
//        Trajectory scoreW3 = drive.trajectoryBuilder(strafePrepW3.end())
//                .splineToLinearHeading(new Pose2d(-7.5,31, Math.toRadians(135)), Math.toRadians(90))
//                .build();

//        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
//                .strafeLeft(10)
//                .build();
//
//        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
//                .strafeRight(36)
//                .build();


        drive.followTrajectory(scoreX2);
        drive.followTrajectory(strafeFromX2);
        //drive.followTrajectory(collectCone);
        drive.followTrajectory(strafePrepW3);
        drive.followTrajectory(goTowardsW3);
        //drive.followTrajectory(scoreW3);
        drive.followTrajectory(resetRotation);
        drive.followTrajectory(goToA2);
        drive.followTrajectory(collectCone2);
        drive.followTrajectory(strafePrepW3);
        drive.followTrajectory(goTowardsW3);
        //drive.followTrajectory(scoreW3);

//        drive.followTrajectory(traj2);
//        drive.followTrajectory(traj3);
//        drive.followTrajectory(traj4);
//        drive.followTrajectory(traj5);
//
//        drive.followTrajectory(traj2);
//        drive.followTrajectory(traj3);
//        drive.followTrajectory(traj4);
//        drive.followTrajectory(traj5);
//
//        drive.followTrajectory(traj2);
//        drive.followTrajectory(traj3);
//        drive.followTrajectory(traj4);
//        drive.followTrajectory(traj5);
//
//        drive.followTrajectory(traj2);
//        drive.followTrajectory(traj3);
//        drive.followTrajectory(traj4);
//        drive.followTrajectory(traj5);
//
//        drive.followTrajectory(traj6);



        sleep(2000);

    }
}
