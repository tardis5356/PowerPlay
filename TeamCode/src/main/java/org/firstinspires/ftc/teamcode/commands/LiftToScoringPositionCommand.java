package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;

import org.firstinspires.ftc.teamcode.subsystems.Junctions;

public class LiftToScoringPositionCommand extends ParallelCommandGroup {
    public LiftToScoringPositionCommand(Lift lift, Arm arm, Gripper gripper, Junctions junction) {
        addCommands(
                new LiftToPositionCommand(lift, junction.position, 5),
                new InstantCommand(() -> {
                    arm.toDeliverPosition();
                    gripper.close();
                })
        );
    }
}