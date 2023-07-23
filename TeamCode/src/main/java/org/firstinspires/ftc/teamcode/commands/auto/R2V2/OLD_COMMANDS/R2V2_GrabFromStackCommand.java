package org.firstinspires.ftc.teamcode.commands.auto.R2V2.OLD_COMMANDS;

import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.LIFT_INTAKE_AUTO_R2V2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.RobotToStateCommand;
import org.firstinspires.ftc.teamcode.commands.auto.R2V2.V3PO_AutoTrajectories;
import org.firstinspires.ftc.teamcode.commands.auto.R2V2.V3PO_FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive_R2V2;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.subsystems.BatWing;


public class R2V2_GrabFromStackCommand extends SequentialCommandGroup {
    FtcDashboard dashboard = FtcDashboard.getInstance();

    private Gripper gripper;

    public R2V2_GrabFromStackCommand(SampleMecanumDrive_R2V2 drive, Lift lift, Arm arm, Wrist wrist, Gripper gripper, BatWing batwing, int stackIndex, boolean isBlue) {
        this.gripper = gripper;

        addCommands(
                new RobotToStateCommand(lift, arm, wrist, gripper, batwing, LIFT_INTAKE_AUTO_R2V2, stackIndex, "intake"),
                new InstantCommand(gripper::open),
                new WaitCommand(250),
                new V3PO_FollowTrajectoryCommand(drive, isBlue ? V3PO_AutoTrajectories.blue_StackWaypointToStack : V3PO_AutoTrajectories.red_StackWaypointToStack),
                new WaitCommand(250),
                new InstantCommand(() -> {
                    gripper.close();
                }),
                new WaitCommand(250),
                new ParallelCommandGroup(
                        new V3PO_FollowTrajectoryCommand(drive, isBlue ? V3PO_AutoTrajectories.blue_StackToStackWaypoint : V3PO_AutoTrajectories.red_StackToStackWaypoint),
                        new SequentialCommandGroup(
//                                new WaitCommand(700),
                                new RobotToStateCommand(lift, arm, wrist, gripper, batwing, LIFT_INTAKE_AUTO_R2V2, 0, "travel"))

                )
        );
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
