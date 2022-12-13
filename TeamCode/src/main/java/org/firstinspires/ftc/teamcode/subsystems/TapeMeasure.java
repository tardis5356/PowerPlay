package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
public class TapeMeasure extends SubsystemBase {
    private CRServo servo;

    public TapeMeasure(HardwareMap hardwareMap) {
        servo = hardwareMap.get(CRServo.class, "sTMT");
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void extend() {
        servo.setPower(-1);
    }

    public void retract() {
        servo.setPower(1);
    }

    public void stop() {
        servo.setPower(0);
    }
}