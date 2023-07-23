package org.firstinspires.ftc.teamcode.commands.auto.R2V2;

import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.LIFT_INTAKE_AUTO_R2V2;
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
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;


public class R2V2_StackToMediumPoleAutoCommand extends SequentialCommandGroup {
    FtcDashboard dashboard = FtcDashboard.getInstance();

    private Gripper gripper;

    public R2V2_StackToMediumPoleAutoCommand(SampleMecanumDrive_R2V2 drive, Lift lift, Arm arm, Wrist wrist, Gripper gripper, BatWing batwing, boolean isBlue) {
        this.gripper = gripper;

        addCommands(
                new ParallelCommandGroup(
                        new V3PO_FollowTrajectoryCommand(drive, isBlue ? V3PO_AutoTrajectories.blue_StackToMedPole : V3PO_AutoTrajectories.red_StackToMedPole),
                        new SequentialCommandGroup(
//                                new WaitCommand(200),
                                new RobotToStateCommand(lift, arm, wrist, gripper, batwing, LIFT_INTAKE_AUTO_R2V2, 0, "travel"),
                                new WaitCommand(500),
                                new RobotToStateCommand(lift, arm, wrist, gripper, batwing, LIFT_MEDIUM_JUNCTION_R2V2, 0, "delivery")
                        )
                ),
                new InstantCommand(() -> {
                    if (!gripper.continueAuto) gripper.abortAuto = true;
                }),
                new InstantCommand(gripper::open)
        );
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
