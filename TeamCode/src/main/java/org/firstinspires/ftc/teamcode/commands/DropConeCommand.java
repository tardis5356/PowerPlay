package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.BatWing;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;

public class DropConeCommand extends ParallelCommandGroup {
    public DropConeCommand(Gripper gripper, BatWing batwing) {
        addCommands(
                new InstantCommand(gripper::open),
                new WaitCommand(250),
                new InstantCommand(batwing::retract)
        );
    }
}