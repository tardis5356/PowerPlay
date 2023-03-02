package org.firstinspires.ftc.teamcode.auto.R2V2;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.apriltags.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.commands.LiftToPositionCommand;
import org.firstinspires.ftc.teamcode.commands.RobotToStateCommand;
import org.firstinspires.ftc.teamcode.commands.auto.R2V2.R2V2_AutoTrajectories;
import org.firstinspires.ftc.teamcode.commands.auto.R2V2.R2V2_DeliverMediumPreloadAutoCommand;
import org.firstinspires.ftc.teamcode.commands.auto.R2V2.R2V2_FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.commands.auto.R2V2.R2V2_GrabFromStackCloseWaypointCommand;
import org.firstinspires.ftc.teamcode.commands.auto.R2V2.R2V2_MedPoleToStackCloseWaypointAutoCommand;
import org.firstinspires.ftc.teamcode.commands.auto.R2V2.R2V2_StackToMediumPoleAutoCommand;
import org.firstinspires.ftc.teamcode.drive.DriveConstants_R2V2;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive_R2V2;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.BatWing;
import org.firstinspires.ftc.teamcode.subsystems.BeaconArm;
import org.firstinspires.ftc.teamcode.subsystems.BotPositions;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(group = "drive", name = "\uD83D\uDFE5 R2V2 Red Mid Cycle \uD83D\uDFE5")
public class R2V2_Red_Full_Mid_Cycle extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    TrajectorySequence parkTrajectory1FromPole, parkTrajectory2FromPole, parkTrajectory3FromPole,  parkTrajectory1FromStack, parkTrajectory2FromStack, parkTrajectory3FromStack;
    //private SampleMecanumDrive_R2V2 drive;

    boolean parking = false;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    int stackIndex = 4;

    private SampleMecanumDrive_R2V2 drive;
    private Lift lift;
    private Arm arm;
    private Wrist wrist;
    private Gripper gripper;
    private BeaconArm beaconArm;
    private BatWing batwing;
//    private Camera camera;

    private R2V2_DeliverMediumPreloadAutoCommand deliverMediumPreloadAutoCommand;
    private R2V2_GrabFromStackCloseWaypointCommand grabFromStackCloseWaypointCommand;
    private R2V2_MedPoleToStackCloseWaypointAutoCommand medPoleToStackCloseWaypointAutoCommand;
    private R2V2_StackToMediumPoleAutoCommand stackToMediumPoleAutoCommand;
    private R2V2_FollowTrajectoryCommand parkTrajectoryCommand;
    private LiftToPositionCommand liftToPositionCommand;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);
        drive = new SampleMecanumDrive_R2V2(hardwareMap);

        // declare subsystems
        drive = new SampleMecanumDrive_R2V2(hardwareMap);
        gripper = new Gripper(hardwareMap);
        lift = new Lift(hardwareMap);
        arm = new Arm(hardwareMap);
        wrist = new Wrist(hardwareMap);
        beaconArm = new BeaconArm(hardwareMap);
        batwing = new BatWing(hardwareMap);
//        camera = new Camera(hardwareMap, telemetry2);

        //generate trajectories, must do this before you declare commands
        drive.setPoseEstimate(R2V2_AutoTrajectories.red_StartPos);
        R2V2_AutoTrajectories.generateTrajectories(drive);

        // declare commands
        deliverMediumPreloadAutoCommand = new R2V2_DeliverMediumPreloadAutoCommand(drive, lift, arm, wrist, gripper, batwing, 0, false);
        grabFromStackCloseWaypointCommand = new R2V2_GrabFromStackCloseWaypointCommand(drive, lift, arm, wrist, gripper, batwing, 0, false);
        stackToMediumPoleAutoCommand = new R2V2_StackToMediumPoleAutoCommand(drive, lift, arm, wrist, gripper, batwing, false);
        medPoleToStackCloseWaypointAutoCommand = new R2V2_MedPoleToStackCloseWaypointAutoCommand(drive, lift, arm, wrist, gripper, batwing, 0, false);
        liftToPositionCommand = new LiftToPositionCommand(lift, 50, 25);

        gripper.close();
        ////////////////////////////DEFINING PARK TRAJECTORIES//////////////////////////////
        parkTrajectory1FromPole = drive.trajectorySequenceBuilder(R2V2_AutoTrajectories.red_MedPolePos)
                .setReversed(true)
//                .lineTo(new Vector2d(36, 12), SampleMecanumDrive_R2V2.getVelocityConstraint(20, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
//                        SampleMecanumDrive_R2V2.getAccelerationConstraint(30))
                .lineToLinearHeading(new Pose2d(58, 12, Math.toRadians(270)), SampleMecanumDrive_R2V2.getVelocityConstraint(50, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                        SampleMecanumDrive_R2V2.getAccelerationConstraint(50))
                .build();

        parkTrajectory2FromPole = drive.trajectorySequenceBuilder(R2V2_AutoTrajectories.red_MedPolePos)
                .setReversed(true)
//                .lineTo(new Vector2d(36, 12), SampleMecanumDrive_R2V2.getVelocityConstraint(20, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
//                        SampleMecanumDrive_R2V2.getAccelerationConstraint(30))
                .lineToLinearHeading(new Pose2d(36, 12, Math.toRadians(270)), SampleMecanumDrive_R2V2.getVelocityConstraint(50, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                        SampleMecanumDrive_R2V2.getAccelerationConstraint(50))
                .build();

        parkTrajectory3FromPole = drive.trajectorySequenceBuilder(R2V2_AutoTrajectories.red_MedPolePos)
                .setReversed(true)
//                .lineTo(new Vector2d(36, 12), SampleMecanumDrive_R2V2.getVelocityConstraint(20, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
//                        SampleMecanumDrive_R2V2.getAccelerationConstraint(30))
                .lineToLinearHeading(new Pose2d(12, 12, Math.toRadians(270)), SampleMecanumDrive_R2V2.getVelocityConstraint(50, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                        SampleMecanumDrive_R2V2.getAccelerationConstraint(50))
                .build();

        parkTrajectory1FromStack = drive.trajectorySequenceBuilder(R2V2_AutoTrajectories.red_MedPolePos)
                .setReversed(true)
//                .lineTo(new Vector2d(36, 12), SampleMecanumDrive_R2V2.getVelocityConstraint(20, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
//                        SampleMecanumDrive_R2V2.getAccelerationConstraint(30))
                .lineToLinearHeading(new Pose2d(58, 12, Math.toRadians(270)), SampleMecanumDrive_R2V2.getVelocityConstraint(50, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                        SampleMecanumDrive_R2V2.getAccelerationConstraint(50))
                .build();

        parkTrajectory2FromStack = drive.trajectorySequenceBuilder(R2V2_AutoTrajectories.red_MedPolePos)
                .setReversed(true)
//                .lineTo(new Vector2d(36, 12), SampleMecanumDrive_R2V2.getVelocityConstraint(20, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
//                        SampleMecanumDrive_R2V2.getAccelerationConstraint(30))
                .lineToLinearHeading(new Pose2d(36, 12, Math.toRadians(270)), SampleMecanumDrive_R2V2.getVelocityConstraint(50, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                        SampleMecanumDrive_R2V2.getAccelerationConstraint(50))
                .build();

        parkTrajectory3FromStack = drive.trajectorySequenceBuilder(R2V2_AutoTrajectories.red_MedPolePos)
                .setReversed(true)
//                .lineTo(new Vector2d(36, 12), SampleMecanumDrive_R2V2.getVelocityConstraint(20, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
//                        SampleMecanumDrive_R2V2.getAccelerationConstraint(30))
                .lineToLinearHeading(new Pose2d(12, 12, Math.toRadians(270)), SampleMecanumDrive_R2V2.getVelocityConstraint(50, DriveConstants_R2V2.MAX_ANG_VEL, DriveConstants_R2V2.TRACK_WIDTH),
                        SampleMecanumDrive_R2V2.getAccelerationConstraint(50))
                .build();
        ////////////////////////////////////DONE DEFINING PARK TRAJECTORIES///////////////////////////////////////


        schedule(new SequentialCommandGroup(
               deliverMediumPreloadAutoCommand,

                new R2V2_GrabFromStackCloseWaypointCommand(drive, lift, arm, wrist, gripper, batwing, 4, false),
                stackToMediumPoleAutoCommand,
                medPoleToStackCloseWaypointAutoCommand,
//
                new R2V2_GrabFromStackCloseWaypointCommand(drive, lift, arm, wrist, gripper, batwing, 3, false),
                stackToMediumPoleAutoCommand,
                medPoleToStackCloseWaypointAutoCommand,

                new R2V2_GrabFromStackCloseWaypointCommand(drive, lift, arm, wrist, gripper, batwing, 2, false),
                stackToMediumPoleAutoCommand,

                // release final cone
                new InstantCommand(arm::toTravelPosition),
                new InstantCommand(wrist::toTravelPosition),
                new InstantCommand(batwing::retract),
                new WaitCommand(400),
                new RobotToStateCommand(lift, arm, wrist, gripper, batwing, BotPositions.LIFT_INTAKE_R2V2, stackIndex, "travel")


//                medPoleToStackCloseWaypointAutoCommand,
//
//                new R2V2_GrabFromStackCloseWaypointCommand(drive, lift, arm, wrist, gripper, batwing, 1, true),
//                stackToMediumPoleAutoCommand,
//                medPoleToStackCloseWaypointAutoCommand,
//
//                new R2V2_GrabFromStackCloseWaypointCommand(drive, lift, arm, wrist, gripper, batwing, 0, true),
//                stackToMediumPoleAutoCommand,


        ));


        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            new RobotToStateCommand(lift, arm, wrist, gripper, batwing, -10, 0, "init");
            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    telemetry.addData("tag id", tag.id);
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);

            telemetry.addData("continueAuto", gripper.continueAuto);
        }
        ///////////////////////////////END OF INITIALIZATION/////////////////////////////////////////////////////////
/////////////////////////////////////////////////INIT DONE, ASSIGNING APRIL TAG////////////////////////////////////////////
/////////////////////////////////////////////////////////////TAG ASSIGNED, START AUTO LOOP/////////////////////////////////////////////////
        runtime.reset();
        while (!isStopRequested() && opModeIsActive()) {
            if (tagOfInterest != null) {
                telemetry.addLine("Tag snapshot:\n");
                tagToTelemetry(tagOfInterest);
                telemetry.update();
            } else {
                telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
                telemetry.update();
            }

            telemetry.addData("lift pos", lift.getLiftPosition());
            telemetry.addData("lift target", lift.getLiftTargetPosition());
            telemetry.addData("lift power", lift.getLiftPower());
            telemetry.addData("continueAuto", gripper.continueAuto);
//was 28
            if (runtime.seconds() > 28 && gripper.continueAuto || !gripper.continueAuto) {
                if (!parking) {
                    CommandScheduler.getInstance().cancelAll();
                    if(tagOfInterest.id != 1 && tagOfInterest.id != 2 && tagOfInterest.id != 3) tagOfInterest.id = 2; //if value is read as null, set default trajectory to middle (2)
                    switch (tagOfInterest.id) {
                        case 1:
//                            if(gripper.continueAuto) parkTrajectoryCommand = new R2V2_FollowTrajectoryCommand(drive, parkTrajectory1FromPole);
//                            else parkTrajectoryCommand = new R2V2_FollowTrajectoryCommand(drive, parkTrajectory1FromStack);
                            parkTrajectoryCommand = new R2V2_FollowTrajectoryCommand(drive, parkTrajectory1FromPole);
                            telemetry.addLine("park traj 1");
                            break;
                        case 2:
//                            if(gripper.continueAuto) parkTrajectoryCommand = new R2V2_FollowTrajectoryCommand(drive, parkTrajectory2FromPole);
//                            else parkTrajectoryCommand = new R2V2_FollowTrajectoryCommand(drive, parkTrajectory2FromStack);
                            parkTrajectoryCommand = new R2V2_FollowTrajectoryCommand(drive, parkTrajectory2FromPole);
                            telemetry.addLine("park traj 2");
                            break;
                        case 3:
//                            if(gripper.continueAuto) parkTrajectoryCommand = new R2V2_FollowTrajectoryCommand(drive, parkTrajectory3FromPole);
//                            else parkTrajectoryCommand = new R2V2_FollowTrajectoryCommand(drive, parkTrajectory3FromStack);
                            parkTrajectoryCommand = new R2V2_FollowTrajectoryCommand(drive, parkTrajectory3FromPole);
                            telemetry.addLine("park traj 3");
                            break;
                    }
                    schedule(
                            new RobotToStateCommand(lift, arm, wrist, gripper, batwing, 100, 0, "autoEnd"),
                            parkTrajectoryCommand,
                            new InstantCommand(() ->{
                                gripper.continueAuto = true;
                            })
                    );
                    parking = true;
                }
                CommandScheduler.getInstance().run();
            } else {
                CommandScheduler.getInstance().run();
            }
        }
        CommandScheduler.getInstance().reset();
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    public void schedule(Command... commands) {
        CommandScheduler.getInstance().schedule(commands);
    }
}






