package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive_MSE;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
//// (name = "Auto_AllFourJunctions_BlueStart_Test", group = "drive")
public class Auto_AllFourJunctions_BlueStart_Test extends LinearOpMode {
    @Override
//    public void runOpMode() throws InterruptedException {

    public void runOpMode() {
        SampleMecanumDrive_MSE drive = new SampleMecanumDrive_MSE(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        //We are assuming the center of the field of 0,0 and A6 is (+x, +y)
        //We want to start the bot at x: -66, y: -36, heading: 180 degrees
        Pose2d startPose = new Pose2d(-36, 66, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(36, 66))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(-90, 0))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineTo(new Vector2d(-90, 12))
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineTo(new Vector2d(-42, 12))
                .build();

//        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
//                .splineToConstantHeading(new Vector2d(-66, 12), Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(-42, 12), Math.toRadians(0))
//                .build();

        drive.followTrajectory(traj1);
//        drive.followTrajectory(traj2);
//        drive.followTrajectory(traj3);
//        drive.followTrajectory(traj4);
    }
}
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//        //We are assuming the center of the field of 0,0 and F6 is (+x, +y)
//        // We want to start the bot at x: -66, y: -36, heading: -90 degrees
//        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
//
//        drive.setPoseEstimate(startPose);
//
//        Trajectory traj1 = drive.trajectoryBuilder(startPose)
//                .splineTo(new Vector2d(-12, 0), 0)
////                .splineTo(new Vector2d(9, -10), 0)
//                .build();
//
//        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
//                .splineTo(new Vector2d(-12, 0), 0)
////                .splineTo(new Vector2d(9, -10), 0)
//                .build();

//        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
//                .strafeLeft(58)
//                .build();

//        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
//                .strafeRight(10)
//                .build();
//
//        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
//                .forward(24)
//                .build();

        ////////////this is where it changes from Auto_BlueX2/////////////////////////

//        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
//                .back(48)
//                .build();
//
//        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
//                .strafeLeft(12)
//                .build();
//
//        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
//                .strafeRight(12)
//                .build();
//
//        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
//                .forward(48)
//                .build();


        //Driving part of the file -- like last year's arraylist
//        drive.followTrajectory(traj1);
//        drive.followTrajectory(traj2);
//
//
//
//
//        sleep(2000);
//
//    }
//}
