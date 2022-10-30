package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist extends SubsystemBase {
    private final Servo servo;

    private final double initPosition = 0.4;
    private final double intakePosition = 0.36;
    private final double deliverPosition = 0.52;

    public Wrist(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "sW");
        toInitPosition();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void toInitPosition(){
        servo.setPosition(initPosition);
    }

    public void toIntakePosition(){
        servo.setPosition(intakePosition);
    }

    public void toDeliverPosition(){
        servo.setPosition(deliverPosition);
    }

    public double getGripperPosition() {
        return servo.getPosition();
    }
}