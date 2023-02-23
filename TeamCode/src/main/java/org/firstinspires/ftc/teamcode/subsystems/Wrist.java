package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.ARM_DELIVERY_Barney;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.ARM_DELIVERY_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.ARM_INTAKE_Barney;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.ARM_INTAKE_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.ARM_TRAVEL_Barney;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.ARM_TRAVEL_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.WRIST_AUTO_INTAKE_WAYPOINT_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.WRIST_DELIVERY_Barney;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.WRIST_DELIVERY_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.WRIST_INIT_Barney;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.WRIST_INIT_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.WRIST_INTAKE_Barney;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.WRIST_INTAKE_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.WRIST_TRAVEL_Barney;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.WRIST_TRAVEL_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.isBarney;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
public class Wrist extends SubsystemBase {
    private final Servo servo;

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
        servoPositionWrist = (Range.clip(servoPositionWrist, 0, 1));
        servo.setPosition(servoPositionWrist);
    }

    public void decreasePosition() {

        servoPositionWrist -= 0.005;
        servoPositionWrist = (Range.clip(servoPositionWrist, 0, 1));
        servo.setPosition(servoPositionWrist);

    }

    public void toInitPosition() {
        servo.setPosition(isBarney ? WRIST_INIT_Barney : WRIST_INIT_R2V2);
        servoPositionWrist = isBarney ? WRIST_INIT_Barney : WRIST_INIT_R2V2;
    }

    public void toIntakePosition() {
        servo.setPosition(isBarney ? WRIST_INTAKE_Barney : WRIST_INTAKE_R2V2);
        servoPositionWrist = isBarney ? WRIST_INTAKE_Barney : WRIST_INTAKE_R2V2;
    }

    public void toIntakeWaypointPosition() {
        servo.setPosition(isBarney ? WRIST_INTAKE_Barney : WRIST_AUTO_INTAKE_WAYPOINT_R2V2);
        servoPositionWrist = isBarney ? WRIST_INTAKE_Barney : WRIST_AUTO_INTAKE_WAYPOINT_R2V2;
    }

    public void toDeliverPosition() {
        servo.setPosition(isBarney ? WRIST_DELIVERY_Barney : WRIST_DELIVERY_R2V2);
        servoPositionWrist = isBarney ? WRIST_DELIVERY_Barney : WRIST_DELIVERY_R2V2;
    }

    public void toTravelPosition() {
        servo.setPosition(isBarney ? WRIST_TRAVEL_Barney : WRIST_TRAVEL_R2V2);
        servoPositionWrist = isBarney ? WRIST_TRAVEL_Barney : WRIST_TRAVEL_R2V2;
    }

    public double getWristPosition() {
        return servo.getPosition();
    }
}