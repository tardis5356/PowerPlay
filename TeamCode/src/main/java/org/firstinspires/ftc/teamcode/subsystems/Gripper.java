package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Gripper extends SubsystemBase {
    private Servo servo;

    public static double OPEN_POSITION = 0.35, CLOSED_POSITION = 0.28;

    public Gripper(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "sG");
//        open();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void open(){ servo.setPosition(OPEN_POSITION); }

    public void close(){
        servo.setPosition(CLOSED_POSITION);
    }

    public double getGripperPosition() {
        return servo.getPosition();
    }
}