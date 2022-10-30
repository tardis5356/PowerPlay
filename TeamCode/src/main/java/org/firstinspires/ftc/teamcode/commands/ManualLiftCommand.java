package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class ManualLiftCommand extends CommandBase {
    private Lift lift;
    private double stick;

    public ManualLiftCommand(Lift lift, double stick) {
        this.lift = lift;
        this.stick = stick;
    }

    @Override
    public void execute() { // runs continuously
        lift.manualControl(stick);
    }
}
