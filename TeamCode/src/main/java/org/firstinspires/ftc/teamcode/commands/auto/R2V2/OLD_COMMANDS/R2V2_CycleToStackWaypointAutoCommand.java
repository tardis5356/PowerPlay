package org.firstinspires.ftc.teamcode.commands.auto.R2V2.OLD_COMMANDS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.LiftToPositionCommand;
import org.firstinspires.ftc.teamcode.commands.auto.R2V2.R2V2_AutoTrajectories;
import org.firstinspires.ftc.teamcode.commands.auto.R2V2.R2V2_FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive_R2V2;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

public class R2V2_CycleToStackWaypointAutoCommand extends ParallelCommandGroup {
    FtcDashboard dashboard = FtcDashboard.getInstance();

    private Gripper gripper;

    public R2V2_CycleToStackWaypointAutoCommand(SampleMecanumDrive_R2V2 drive, Lift lift, Arm arm, Wrist wrist, Gripper gripper, int stackIndex, boolean isBlue) {
        this.gripper = gripper;

        addCommands(
                new R2V2_FollowTrajectoryCommand(drive, isBlue ? R2V2_AutoTrajectories.blue_MedPoleToStackWaypoint : R2V2_AutoTrajectories.red_MedPoleToStackWaypoint),
                new SequentialCommandGroup(
                    new WaitCommand(500),
                    new LiftToPositionCommand(lift, 50, 25))
            );

    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
