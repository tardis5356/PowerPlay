package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.GRIPPER_CLOSED_Barney;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.GRIPPER_CLOSED_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.GRIPPER_OPEN_Barney;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.GRIPPER_OPEN_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.isBarney;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Config
public class Gripper extends SubsystemBase {
    private Servo servo;
    private NormalizedColorSensor colorSensor;

    public double servoPosition = 0;

    public Gripper(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "sG");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorGripper");
        close();
        //it was originally open()
    }

//    public void ColorSensor(HardwareMap hardwareMap) {
//        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "Color");
//    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void open(){ servo.setPosition(isBarney ? GRIPPER_OPEN_Barney : GRIPPER_OPEN_R2V2); }

    public void close(){
        servo.setPosition(isBarney ? GRIPPER_CLOSED_Barney : GRIPPER_CLOSED_R2V2);
    }

    public boolean hasCone(){
        double distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
        boolean closed = false;
        if (distance < 2) {
           // servo.setPosition(isBarney ? GRIPPER_CLOSED_Barney : GRIPPER_CLOSED_R2V2);
            closed = true;
        } else {
            //servo.setPosition(isBarney ? GRIPPER_OPEN_Barney : GRIPPER_OPEN_R2V2);
            closed = false;
        }
        return closed;
    }


    public double getDistance(){
        if (!isBarney) return ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
        return 0;
    }


    public void increasePosition() {
        servoPosition += 0.005;
        servoPosition = (Range.clip(servoPosition, 0,1));
        servo.setPosition(servoPosition);
    }

    public void decreasePosition() {
        servoPosition -= 0.005;
        servoPosition = (Range.clip(servoPosition, 0,1));
        servo.setPosition(servoPosition);

    }

    public double getGripperPosition() {
        return servo.getPosition();
    }
}