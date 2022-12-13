package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Button;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.LiftToIntakePositionCommand;
import org.firstinspires.ftc.teamcode.commands.LiftToScoringPositionCommand;
import org.firstinspires.ftc.teamcode.commands.ManualLiftCommand;
import org.firstinspires.ftc.teamcode.commands.auto.BarneyCycleToPoleAutoCommand;
import org.firstinspires.ftc.teamcode.commands.auto.BarneyCycleToStackWaypointAutoCommand;
import org.firstinspires.ftc.teamcode.commands.auto.BarneyDeliverPreloadAutoCommand;
import org.firstinspires.ftc.teamcode.commands.auto.BarneyGrabFromStackCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive_R2V2;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.BeaconArm;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Junctions;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.commands.auto.R2V2AutoTrajectories;

@Autonomous(group = "drive", name = "Barney Blue 1+5")
public class Blue_1_5_Barney extends CommandOpMode {
    ElapsedTime runtime = new ElapsedTime();

    final static double fullAutoTime = 30, cycleTime = 5, parkTime = 2;

    boolean finalCycle = false;
    int totalCycles = 0;
    int stackIndex = 4;

    private SampleMecanumDrive_R2V2 drive;
    private Lift lift;
    private Arm arm;
    private Wrist wrist;
    private Gripper gripper;
    private BeaconArm beaconArm;

    private BarneyCycleToPoleAutoCommand cycleToPoleAutoCommand;
    private BarneyCycleToStackWaypointAutoCommand cycleToStackWaypointAutoCommand;
    private BarneyDeliverPreloadAutoCommand deliverPreloadAutoCommand;
    private BarneyGrabFromStackCommand grabFromStackCommand;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void initialize() {
        drive = new SampleMecanumDrive_R2V2(hardwareMap);
        gripper = new Gripper(hardwareMap);
        lift = new Lift(hardwareMap);
        arm = new Arm(hardwareMap);
        wrist = new Wrist(hardwareMap);
        beaconArm = new BeaconArm(hardwareMap);

        cycleToPoleAutoCommand = new BarneyCycleToPoleAutoCommand(drive, lift, arm, wrist, gripper);
        cycleToStackWaypointAutoCommand = new BarneyCycleToStackWaypointAutoCommand(drive, lift, arm, wrist, gripper, stackIndex);
        deliverPreloadAutoCommand = new BarneyDeliverPreloadAutoCommand(drive, lift, arm, wrist, gripper, stackIndex);
        grabFromStackCommand = new BarneyGrabFromStackCommand(drive, lift, arm, wrist, gripper, stackIndex);

        drive.setPoseEstimate(R2V2AutoTrajectories.blue_StartPos);
        R2V2AutoTrajectories.generateTrajectories(drive);

        gripper.close();

        while (!isStarted()) {
            telemetry.addLine("Ready for start!");
            telemetry.update();
        }

        schedule(new SequentialCommandGroup(
                deliverPreloadAutoCommand,
                grabFromStackCommand,
                new InstantCommand(() -> { stackIndex--; }),
                cycleToPoleAutoCommand,
                cycleToStackWaypointAutoCommand, grabFromStackCommand,
                new InstantCommand(() -> { stackIndex--; }),
                cycleToPoleAutoCommand,
                cycleToStackWaypointAutoCommand, grabFromStackCommand,
                new InstantCommand(() -> { stackIndex--; }),
                cycleToPoleAutoCommand,
                cycleToStackWaypointAutoCommand//, grabFromStackCommand,
//                new InstantCommand(() -> { stackIndex--; }),
//                cycleToPoleAutoCommand,
//                cycleToStackWaypointAutoCommand, grabFromStackCommand,
//                new InstantCommand(() -> { stackIndex--; }),
//                cycleToPoleAutoCommand
        ));

    }
}



