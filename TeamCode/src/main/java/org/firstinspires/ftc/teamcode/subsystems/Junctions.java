package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public enum Junctions {
    FULL_RETRACTION(0), // 0
    INTAKE(30), // 100
    GROUND_JUNCTION(100),
    LOW_JUNCTION(275), // 200
    MEDIUM_JUNCTION(550), // 450
    HIGH_JUNCTION(800);// 600

    public int position;

    Junctions(int position) {
        this.position = position;
    }
}
