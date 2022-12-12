package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@Config
public class Gripper extends SubsystemBase {
    private Servo servo;

    public static double OPEN_POSITION = BotPositions.GRIPPER_OPEN, CLOSED_POSITION = BotPositions.GRIPPER_CLOSED;
    public double servoPosition = 0;

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

    public void increasePosition() {
        servoPosition += 0.005;
        servoPosition = (Range.clip(servoPosition, 0.7,1));
        servo.setPosition(servoPosition);
    }

    public void decreasePosition() {
        servoPosition -= 0.005;
        servoPosition = (Range.clip(servoPosition, 0.7,1));
        servo.setPosition(servoPosition);

    }

    public double getGripperPosition() {
        return servo.getPosition();
    }
}