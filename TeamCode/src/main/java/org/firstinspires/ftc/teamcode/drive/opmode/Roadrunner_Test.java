package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive_Barney;
@Config
@Disabled
@Autonomous(name = "Roadrunner_Test", group = "Autonomous")


public abstract class Roadrunner_Test extends LinearOpMode {

    @Override
    public void runOpMode() {
        SampleMecanumDrive_Barney drive = new SampleMecanumDrive_Barney(hardwareMap);


        waitForStart();

        if(isStopRequested()) return;

        drive.setPoseEstimate(new Pose2d());

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .forward(12)
                .build();

        drive.followTrajectory(traj1);
    }
}
