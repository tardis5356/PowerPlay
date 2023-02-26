package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.BatWing;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;

public class DropConeCommand extends SequentialCommandGroup {
    public DropConeCommand(Gripper gripper, BatWing batwing) {
        addCommands(
                new InstantCommand(gripper::open),
                new WaitCommand(400),
                new InstantCommand(batwing::retract)
        );
    }
}