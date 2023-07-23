package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.drive.DriveConstants_V3PO;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive_V3PO;

/*
 * This routine determines the effective track width. The procedure works by executing a point turn
 * with a given angle and measuring the difference between that angle and the actual angle (as
 * indicated by an external IMU/gyro, track wheels, or some other localizer). The quotient
 * given angle / actual angle gives a multiplicative adjustment to the estimated track width
 * (effective track width = estimated track width * given angle / actual angle). The routine repeats
 * this procedure a few times and averages the values for additional accuracy. Note: a relatively
 * accurate track width estimate is important or else the angular constraints will be thrown off.
 */
//@Disabled
@Config
@Autonomous(group = "drive")
public class TrackWidthTuner extends LinearOpMode {
    public static double ANGLE = 180; // deg
    public static int NUM_TRIALS = 5;
    public static int DELAY = 1000; // ms
    double headingAccumulator = 0;

    //make sure heading and translational PIDs are set to 0 when doing track width tuner//


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //make sure that this points to the right bot - ie Gen3 or mse
        SampleMecanumDrive_V3PO drive = new SampleMecanumDrive_V3PO(hardwareMap);
        // TODO: if you haven't already, set the localizer to something that doesn't depend on
        // drive encoders for computing the heading
//        setLocalizer(new StandardTrackingWheelLocalizer_Gen3(hardwareMap));


        telemetry.addLine("Press play to begin the track width tuner routine");
        telemetry.addLine("Make sure your robot has enough clearance to turn smoothly");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.clearAll();
        telemetry.addLine("Running...");
        telemetry.addData("mFL", hardwareMap.get(DcMotorEx.class, "mFL").getCurrentPosition() );
        telemetry.addData("mFR", -hardwareMap.get(DcMotorEx.class, "mFR").getCurrentPosition() );
        telemetry.addData("mBL", hardwareMap.get(DcMotorEx.class, "mBL").getCurrentPosition() );
        telemetry.addData("mBR", hardwareMap.get(DcMotorEx.class, "mBR").getCurrentPosition() );
        telemetry.update();

        MovingStatistics trackWidthStats = new MovingStatistics(NUM_TRIALS);
        for (int i = 0; i < NUM_TRIALS; i++) {
            drive.setPoseEstimate(new Pose2d());

            // it is important to handle heading wraparounds
            double lastHeading = 0;
            headingAccumulator = 0;
            drive.turnAsync(Math.toRadians(ANGLE));
//            drive.turn(Math.toRadians(ANGLE));// will this work better?

            while (!isStopRequested() && drive.isBusy()) {
                double heading = drive.getPoseEstimate().getHeading();
                headingAccumulator += Angle.normDelta(heading - lastHeading);
                lastHeading = heading;

                drive.update();
            }

            double trackWidth = DriveConstants_V3PO.TRACK_WIDTH * Math.toRadians(ANGLE) / headingAccumulator;
            trackWidthStats.add(trackWidth);

            sleep(DELAY);
        }

        telemetry.clearAll();
        telemetry.addLine("Tuning complete");
        telemetry.addLine(Misc.formatInvariant("Effective track width = %.2f (SE = %.3f)",
                trackWidthStats.getMean(),
                trackWidthStats.getStandardDeviation() / Math.sqrt(NUM_TRIALS)));
        telemetry.addData("heading accumulator", headingAccumulator);
        telemetry.addData("angle", ANGLE);
        telemetry.update();

        while (!isStopRequested()) {
            idle();
        }
    }
}
