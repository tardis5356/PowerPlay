package org.firstinspires.ftc.teamcode.subsystems;

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

    public static double INIT_POSITION = BotPositions.ARM_INIT, INTAKE_POSITION = BotPositions.ARM_INTAKE, DELIVERY_POSITION = BotPositions.ARM_DELIVERY;//0.01

    public double servoPosition;

    public Arm(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "sA");
//        toInitPosition();
    }

    @Override
    public void periodic() {

    }

    public void increasePosition() {
        servoPosition += 0.005;
        servoPosition = (Range.clip(servoPosition, 0, 1));
        servo.setPosition(servoPosition);
    }

    public void decreasePosition() {
        servoPosition -= 0.005;
        servoPosition = (Range.clip(servoPosition, 0, 1));
        servo.setPosition(servoPosition);
    }

    public void toInitPosition() {
        servo.setPosition(INIT_POSITION);
    }

    public void toIntakePosition() {
        servo.setPosition(INTAKE_POSITION);
    }

    public void toDeliverPosition() {
        servo.setPosition(DELIVERY_POSITION);
    }

    public double getArmPosition() {
        return servo.getPosition();
    }
}