package org.firstinspires.ftc.teamcode.commands.auto.R2V2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;


import org.firstinspires.ftc.teamcode.commands.RobotToStateCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive_R2V2;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.BatWing;
import org.firstinspires.ftc.teamcode.subsystems.BotPositions;

public class R2V2_CycleToStackCloseWaypointAutoCommand extends ParallelCommandGroup {
    FtcDashboard dashboard = FtcDashboard.getInstance();

    private Gripper gripper;

    public R2V2_CycleToStackCloseWaypointAutoCommand(SampleMecanumDrive_R2V2 drive, Lift lift, Arm arm, Wrist wrist, Gripper gripper, BatWing batwing, int stackIndex, boolean isBlue) {
        this.gripper = gripper;

        addCommands(
                new R2V2_FollowTrajectoryCommand(drive, isBlue ? R2V2_AutoTrajectories.blue_MedPoleToStackWaypoint : R2V2_AutoTrajectories.red_MedPoleToStackWaypoint), //TODO: speed up a lot
                new SequentialCommandGroup(
                        new WaitCommand(500),
                        new InstantCommand((() -> {
                            batwing.retract();
                        })),
                        // new LiftToPositionCommand(lift, 50, 25))
                        new RobotToStateCommand(lift, arm, wrist, gripper, batwing, BotPositions.LIFT_INTAKE_R2V2, stackIndex, "travel")
                ));

    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
