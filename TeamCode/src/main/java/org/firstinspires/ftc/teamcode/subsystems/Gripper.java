package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gripper extends SubsystemBase {
    private final Servo servo;

    private final double openPosition = 0.36;
    private final double closePosition = 0.52;

    public Gripper(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "sGripper");
        open();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void open(){
        servo.setPosition(openPosition);
    }

    public void close(){
        servo.setPosition(closePosition);
    }
}