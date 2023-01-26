package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive_R2V2;

/*
 * This is a simple routine to test translational drive capabilities.
 */
//@Disabled
@Config
@Autonomous(group = "drive")
public class StraightTest extends LinearOpMode {
    public static double DISTANCE = 60; // in

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive_R2V2 drive = new SampleMecanumDrive_R2V2(hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(DISTANCE)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.addData("mFL", hardwareMap.get(DcMotorEx.class, "mFL").getCurrentPosition() );
        telemetry.addData("mFR", -hardwareMap.get(DcMotorEx.class, "mFR").getCurrentPosition() );
        telemetry.addData("mBL", hardwareMap.get(DcMotorEx.class, "mBL").getCurrentPosition() );
        telemetry.addData("mBR", hardwareMap.get(DcMotorEx.class, "mBR").getCurrentPosition() );
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
