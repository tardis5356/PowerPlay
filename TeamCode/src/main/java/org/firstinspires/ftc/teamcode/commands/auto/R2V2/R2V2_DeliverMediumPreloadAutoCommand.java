package org.firstinspires.ftc.teamcode.commands.auto.R2V2;

import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.LIFT_MEDIUM_JUNCTION_R2V2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.RobotToStateCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive_R2V2;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.BatWing;
import org.firstinspires.ftc.teamcode.subsystems.BotPositions;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;


public class R2V2_DeliverMediumPreloadAutoCommand extends SequentialCommandGroup {
    private Gripper gripper;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    public R2V2_DeliverMediumPreloadAutoCommand(SampleMecanumDrive_R2V2 drive, Lift lift, Arm arm, Wrist wrist, Gripper gripper, BatWing batwing, int stackIndex, boolean isBlue) {
        this.gripper = gripper;

        addCommands(
//                new InstantCommand(() -> {
//                    batwing.deployed();
//                }),
//                new R2V2_FollowTrajectoryCommand(drive, isBlue ? R2V2_AutoTrajectories.blue_StartToPreloadPoleWaypoint : R2V2_AutoTrajectories.red_StartToPreloadPoleWaypoint),
//                new ParallelCommandGroup(
//                        new R2V2_FollowTrajectoryCommand(drive, isBlue ? R2V2_AutoTrajectories.blue_PreloadPoleWaypointToPreloadPole : R2V2_AutoTrajectories.red_PreloadPoleWaypointToPreloadPole),
//                                new RobotToStateCommand(lift, arm, wrist, gripper, batwing, LIFT_HIGH_JUNCTION_R2V2, 0, "delivery")
//
//                ),
                new ParallelCommandGroup(
                        new V3PO_FollowTrajectoryCommand(drive, isBlue ? V3PO_AutoTrajectories.blue_StartToMedPreloadPole : V3PO_AutoTrajectories.red_StartToMedPreloadPole),
                        new SequentialCommandGroup(
                                new InstantCommand(batwing::storage),
                                new WaitCommand(1000),
                                new RobotToStateCommand(lift, arm, wrist, gripper, batwing, LIFT_MEDIUM_JUNCTION_R2V2, 0, "delivery")
                        )
                ),
                new InstantCommand(gripper::open),
                new WaitCommand(100),
                new ParallelCommandGroup(
                        new V3PO_FollowTrajectoryCommand(drive, isBlue ? V3PO_AutoTrajectories.blue_MedPreloadPoleToStack : V3PO_AutoTrajectories.red_MedPreloadPoleToStack),
                        new SequentialCommandGroup(new InstantCommand(arm::toTravelPosition),
                                new InstantCommand(wrist::toTravelPosition),
//                        new WaitCommand(250),
                                new InstantCommand(batwing::retract),
                                new WaitCommand(400),
                                new RobotToStateCommand(lift, arm, wrist, gripper, batwing, BotPositions.LIFT_INTAKE_R2V2, stackIndex, "intakeWaypointFirst"),
                                new WaitCommand(350),
                                new InstantCommand(batwing::storage)
                        )
                )
        );
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
