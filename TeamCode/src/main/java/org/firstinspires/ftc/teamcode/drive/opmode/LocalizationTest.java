package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive_Barney;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive_R2V2;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive_R2V2 drive = new SampleMecanumDrive_R2V2(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY()); //
            telemetry.addData("wheel positions", drive.getWheelPositions());
            telemetry.addData("mFL", hardwareMap.get(DcMotorEx.class, "mFL").getCurrentPosition() );
            telemetry.addData("mFR", -hardwareMap.get(DcMotorEx.class, "mFR").getCurrentPosition() );
            telemetry.addData("mBL", hardwareMap.get(DcMotorEx.class, "mBL").getCurrentPosition() );
            telemetry.addData("mBR", hardwareMap.get(DcMotorEx.class, "mBR").getCurrentPosition() );
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
