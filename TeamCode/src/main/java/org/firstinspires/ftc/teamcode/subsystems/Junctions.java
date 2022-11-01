package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

public enum Junctions {
    FULL_RETRACTION(0), // 0
    INTAKE(100), // 100
    GROUND_JUNCTION(100),
    LOW_JUNCTION(200), // 200
    MEDIUM_JUNCTION(450), // 450
    HIGH_JUNCTION(680);// 600

    public int position;

    Junctions(int position) {
        this.position = position;
    }
}
