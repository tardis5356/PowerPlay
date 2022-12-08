package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
public class Wrist extends SubsystemBase {
    private final Servo servo;

    public static double INIT_POSITION = 0.8, INTAKE_POSITION = 0.3, DELIVER_POSITION = 0.9;//0.2
    public double servoPositionWrist = 0;

    public Wrist(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "sW");
        toInitPosition();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void increasePosition() {
        servoPositionWrist += 0.005;
        servoPositionWrist = (Range.clip(servoPositionWrist, 0,1));
        servo.setPosition(servoPositionWrist);
    }

    public void decreasePosition() {

        servoPositionWrist -= 0.005;
        servoPositionWrist = (Range.clip(servoPositionWrist, 0,1));
        servo.setPosition(servoPositionWrist);

    }

    public void toInitPosition(){
        servo.setPosition(INIT_POSITION);
    }

    public void toIntakePosition(){
        servo.setPosition(INTAKE_POSITION);
    }

    public void toDeliverPosition(){ servo.setPosition(DELIVER_POSITION); }

    public double getWristPosition() {
        return servo.getPosition();
    }
}