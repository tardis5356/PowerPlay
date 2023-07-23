package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.ARM_DELIVERY_V3PO;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.ARM_DELIVERY_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.ARM_INTAKE_V3PO;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.ARM_INTAKE_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.ARM_TRAVEL_V3PO;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.ARM_TRAVEL_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.WRIST_AUTO_INTAKE_WAYPOINT_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.WRIST_DELIVERY_V3PO;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.WRIST_DELIVERY_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.WRIST_INIT_V3PO;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.WRIST_INIT_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.WRIST_INTAKE_V3PO;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.WRIST_INTAKE_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.WRIST_TRAVEL_V3PO;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.WRIST_TRAVEL_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.isV3PO;

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
        toDeliverPosition();
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
        servo.setPosition(isV3PO ? WRIST_INIT_V3PO : WRIST_INIT_R2V2);
        servoPositionWrist = isV3PO ? WRIST_INIT_V3PO : WRIST_INIT_R2V2;
    }

    public void toIntakePosition() {
        servo.setPosition(isV3PO ? WRIST_INTAKE_V3PO : WRIST_INTAKE_R2V2);
        servoPositionWrist = isV3PO ? WRIST_INTAKE_V3PO : WRIST_INTAKE_R2V2;
    }

    public void toIntakeWaypointPosition() {
        servo.setPosition(isV3PO ? WRIST_INTAKE_V3PO : WRIST_AUTO_INTAKE_WAYPOINT_R2V2);
        servoPositionWrist = isV3PO ? WRIST_INTAKE_V3PO : WRIST_AUTO_INTAKE_WAYPOINT_R2V2;
    }

    public void toDeliverPosition() {
        servo.setPosition(isV3PO ? WRIST_DELIVERY_V3PO : WRIST_DELIVERY_R2V2);
        servoPositionWrist = isV3PO ? WRIST_DELIVERY_V3PO : WRIST_DELIVERY_R2V2;
    }

    public void toTravelPosition() {
        servo.setPosition(isV3PO ? WRIST_TRAVEL_V3PO : WRIST_TRAVEL_R2V2);
        servoPositionWrist = isV3PO ? WRIST_TRAVEL_V3PO : WRIST_TRAVEL_R2V2;
    }

    public double getWristPosition() {
        return servo.getPosition();
    }
}