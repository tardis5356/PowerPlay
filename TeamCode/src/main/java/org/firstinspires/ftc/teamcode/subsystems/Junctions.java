package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public enum Junctions {
    FULL_RETRACTION(0), // 0
    INTAKE(50), // 100
    GROUND_JUNCTION(100),
    LOW_JUNCTION(275), // 200
    MEDIUM_JUNCTION(550), // 450
    HIGH_JUNCTION(800),// 600

    FULL_RETRACTION_R2V2(0), // 0
    INTAKE_R2V2(30), // 100
    GROUND_JUNCTION_R2V2(100),
    LOW_JUNCTION_R2V2(450), // 200
    MEDIUM_JUNCTION_R2V2(900), // 450
    HIGH_JUNCTION_R2V2(1200);// TODO: 1275 w fixed wiring


    public int position;

    Junctions(int position) {
        this.position = position;
    }
}
