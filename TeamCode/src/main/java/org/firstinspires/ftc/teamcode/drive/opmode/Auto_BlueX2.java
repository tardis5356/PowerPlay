package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive_Barney;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Disabled
@Autonomous(group = "drive")
public class Auto_BlueX2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive_Barney drive = new SampleMecanumDrive_Barney(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(58)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeRight(10)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .forward(24)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .back(24)
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .strafeLeft(10)
                .build();

        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .strafeRight(36)
                .build();


//        drive.followTrajectory(traj1);
//
//        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
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
