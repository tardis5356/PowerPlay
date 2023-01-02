package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.isBarney;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@Config
public class Coffin extends SubsystemBase {
    private Servo servo;

    public static double EXTENDED_POSITION = BotPositions.COFFIN_EXTENDED_R2V2, RETRACTED_POSITION = BotPositions.COFFIN_RETRACTED_R2V2;

    public Coffin(HardwareMap hardwareMap) {
        if (!isBarney) {
            servo = hardwareMap.get(Servo.class, "sC");
            retract();
        }

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void extend() {
        if (!isBarney) servo.setPosition(EXTENDED_POSITION);
    }

    public void retract() {
        if(!isBarney) servo.setPosition(RETRACTED_POSITION);
    }

    public double getCoffinPosition() {
        return servo.getPosition();
    }
}