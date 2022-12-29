package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Junctions;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

public class LiftToIntakePositionCommand extends ParallelCommandGroup {
    public LiftToIntakePositionCommand(Lift lift, Arm arm, Gripper gripper, Wrist wrist, Junctions junction, int stackIndex) {
        addCommands(
                new LiftToPositionCommand(lift, junction.position + (stackIndex * 38), 15),
                new InstantCommand(() -> {
                    wrist.toIntakePosition();
                    //gripper.open();
                    arm.toIntakePosition();
                })
        );
    }
}