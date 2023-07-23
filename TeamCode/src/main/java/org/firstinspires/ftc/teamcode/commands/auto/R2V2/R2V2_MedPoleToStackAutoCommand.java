package org.firstinspires.ftc.teamcode.commands.auto.R2V2;

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

public class R2V2_MedPoleToStackAutoCommand extends ParallelCommandGroup {
    FtcDashboard dashboard = FtcDashboard.getInstance();

    private Gripper gripper;

    public R2V2_MedPoleToStackAutoCommand(SampleMecanumDrive_R2V2 drive, Lift lift, Arm arm, Wrist wrist, Gripper gripper, BatWing batwing, int stackIndex, boolean isBlue, boolean isSlow) {
        this.gripper = gripper;


        addCommands(
                new V3PO_FollowTrajectoryCommand(drive, isBlue ? (isSlow ? V3PO_AutoTrajectories.blue_MedPoleToStackSlow : V3PO_AutoTrajectories.blue_MedPoleToStack) :  (isSlow ? V3PO_AutoTrajectories.red_MedPoleToStackSlow : V3PO_AutoTrajectories.red_MedPoleToStack)), //TODO: speed up a lot
                new SequentialCommandGroup(
                        new InstantCommand(arm::toTravelPosition),
                        new InstantCommand(wrist::toTravelPosition),
//                        new WaitCommand(250),
                        new InstantCommand(batwing::retract),
                        new WaitCommand(250),
                        new RobotToStateCommand(lift, arm, wrist, gripper, batwing, BotPositions.LIFT_INTAKE_R2V2, stackIndex, "intakeWaypoint"),
                        new InstantCommand(gripper::open)
                ));

    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
