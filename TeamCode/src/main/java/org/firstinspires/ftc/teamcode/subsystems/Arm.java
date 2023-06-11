package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.ARM_AUTO_END_Barney;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.ARM_AUTO_END_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.ARM_AUTO_INTAKE_WAYPOINT_Barney;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.ARM_AUTO_INTAKE_WAYPOINT_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.ARM_DELIVERY_Barney;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.ARM_DELIVERY_DROP_Barney;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.ARM_DELIVERY_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.ARM_INIT_Barney;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.ARM_INIT_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.ARM_INTAKE_Barney;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.ARM_INTAKE_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.ARM_TRAVEL_Barney;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.ARM_TRAVEL_R2V2;
import static org.firstinspires.ftc.teamcode.subsystems.BotPositions.isBarney;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.BotPositions;

@Config
public class Arm extends SubsystemBase {
    private Servo servo;
    private Servo servo2;
    public double servoPosition;

    private double servoOffset = 1.02;

    public Arm(HardwareMap hardwareMap) {
        if(isBarney) {
            servo = hardwareMap.get(Servo.class, "sAR");
            servo2 = hardwareMap.get(Servo.class, "sAL");
        }else{
            servo = hardwareMap.get(Servo.class, "sA");
        }
        toInitPosition();
    }

    @Override
    public void periodic() {

    }

    public void increasePosition() {
//        servoPosition += 0.005;
//        servoPosition = (Range.clip(servoPosition, 0, 1));
//        servo.setPosition(servoPosition);
    }

    public void decreasePosition() {
//        servoPosition -= 0.005;
//        servoPosition = (Range.clip(servoPosition, 0, 1));
//        servo.setPosition(servoPosition);
    }

    public void toInitPosition() {
        servoPosition = isBarney ? ARM_INIT_Barney : ARM_INIT_R2V2;
        if(isBarney){
            servo.setPosition(ARM_INIT_Barney);
            servo2.setPosition(servoOffset-ARM_INIT_Barney);
        }else{
            servo.setPosition(ARM_INIT_R2V2);
        }
    }

    public void toIntakePosition() {
        servoPosition = isBarney ? ARM_INTAKE_Barney : ARM_INTAKE_R2V2;
        if(isBarney){
            servo.setPosition(ARM_INTAKE_Barney);
            servo2.setPosition(servoOffset-ARM_INTAKE_Barney);
        }else{
            servo.setPosition(ARM_INTAKE_R2V2);
        }
    }

    public void toIntakeWaypointPosition() {
        servoPosition = isBarney ? ARM_AUTO_INTAKE_WAYPOINT_Barney : ARM_AUTO_INTAKE_WAYPOINT_R2V2;
        if(isBarney){
            servo.setPosition(ARM_AUTO_INTAKE_WAYPOINT_Barney);
            servo2.setPosition(servoOffset-ARM_AUTO_INTAKE_WAYPOINT_Barney);
        }else{
            servo.setPosition(ARM_AUTO_INTAKE_WAYPOINT_R2V2);
        }
    }

    public void toDeliverPosition() {
        servoPosition = isBarney ? ARM_DELIVERY_Barney : ARM_DELIVERY_R2V2;
        if(isBarney){
            servo.setPosition(ARM_DELIVERY_Barney);
            servo2.setPosition(servoOffset-ARM_DELIVERY_Barney);
        }else{
            servo.setPosition(ARM_DELIVERY_R2V2);
        }
    }
    public void toDeliverDropPosition() {
        servoPosition = isBarney ? ARM_DELIVERY_DROP_Barney : ARM_DELIVERY_R2V2;
        if(isBarney){
            servo.setPosition(ARM_DELIVERY_DROP_Barney);
            servo2.setPosition(servoOffset-ARM_DELIVERY_DROP_Barney);
        }else{
            servo.setPosition(ARM_DELIVERY_R2V2);
        }
    }

    public void toTravelPosition() {
        servoPosition = isBarney ? ARM_TRAVEL_Barney : ARM_TRAVEL_R2V2;
        if(isBarney){
            servo.setPosition(ARM_TRAVEL_Barney);
            servo2.setPosition(servoOffset-ARM_TRAVEL_Barney);
        }else{
            servo.setPosition(ARM_TRAVEL_R2V2);
        }
    }

    public void toAutoEndPosition() {
        servoPosition = isBarney ? ARM_AUTO_END_Barney : ARM_AUTO_END_R2V2;
        if(isBarney){
            servo.setPosition(ARM_AUTO_END_Barney);
            servo2.setPosition(servoOffset-ARM_AUTO_END_Barney);
        }else{
            servo.setPosition(ARM_AUTO_END_R2V2);
        }
    }

    public double getArmPosition() {
        return servo.getPosition();
    }
}