package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Subsystem extends SubsystemBase {
    private final Servo servo;

    private final double loadPosition = 0.99;
    private final double scoringPosition = 0.43;

    public boolean loading = true;

    public Subsystem(HardwareMap hardwareMap){
        servo = hardwareMap.get(Servo.class, "sBeaconArm");
        function1();
    }

    @Override
    public void periodic(){
        //happens every loop
    }

    public void function1(){
        servo.setPosition(loadPosition);
        loading = true;
    }

    public void function2(){
        servo.setPosition(scoringPosition);
        loading = false;
    }
}