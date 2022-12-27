package org.firstinspires.ftc.teamcode.subsystems;

import android.drm.DrmStore;

import com.acmerobotics.dashboard.config.Config;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.util.RegressionUtil;

import java.util.Arrays;

@Config
public class BotPositions {
    public static int activeBot = 0;
    //0 = Barney
    //1 = R2V2

    public static double LIFT_FULL_RETRACTION, LIFT_INTAKE, LIFT_GROUND_JUNCTION, LIFT_LOW_JUNCTION, LIFT_MEDIUM_JUNCTION, LIFT_HIGH_JUNCTION;

    public static double LIFT_pE, LIFT_pR, LIFT_i, LIFT_d;

    public static double ARM_INIT, ARM_INTAKE, ARM_DELIVERY;

    public static double WRIST_INIT, WRIST_INTAKE, WRIST_DELIVERY;

    public static double GRIPPER_OPEN, GRIPPER_CLOSED;

    public static double BEACON_ARM_LOAD, BEACON_ARM_STORAGE, BEACON_ARM_DELIVERY;


    public static double LIFT_FULL_RETRACTION_Barney = 0, LIFT_INTAKE_Barney = 40, LIFT_GROUND_JUNCTION_Barney = 100, LIFT_LOW_JUNCTION_Barney = 275, LIFT_MEDIUM_JUNCTION_Barney = 550, LIFT_HIGH_JUNCTION_Barney = 800;

    public static double LIFT_pE_Barney = 0.07, LIFT_pR_Barney = 0.0001, LIFT_i_Barney = 0, LIFT_d_Barney = 0;

    public static double ARM_INIT_Barney = 0.6, ARM_INTAKE_Barney = 0.76, ARM_DELIVERY_Barney = 0.4;

    public static double WRIST_INIT_Barney = 0.8, WRIST_INTAKE_Barney = 0.25, WRIST_DELIVERY_Barney = 0.9;

    public static double GRIPPER_OPEN_Barney = 0.5, GRIPPER_CLOSED_Barney = 0.25; // 0.55, 0.3 // 0.62, 0.73

    public static double BEACON_ARM_LOAD_Barney = 0.99, BEACON_ARM_STORAGE_Barney = 0.5, BEACON_ARM_DELIVERY_Barney = 0.75, BEACON_ARM_TRAVEL_Barney = 0.55;


    public static double LIFT_FULL_RETRACTION_R2V2 = 0, LIFT_INTAKE_R2V2 = 0, LIFT_GROUND_JUNCTION_R2V2 = 0, LIFT_LOW_JUNCTION_R2V2 = 0, LIFT_MEDIUM_JUNCTION_R2V2 = 0, LIFT_HIGH_JUNCTION_R2V2 = 0;

    public static double LIFT_p_R2V2 = 0, LIFT_i_R2V2 = 0, LIFT_d_R2V2 = 0;

    public static double ARM_INIT_R2V2 = 0.53, ARM_INTAKE_R2V2 = 0.635, ARM_DELIVERY_R2V2 = 0.4;

    public static double WRIST_INIT_R2V2 = 0.7, WRIST_INTAKE_R2V2 = 0.31, WRIST_DELIVERY_R2V2 = 0.85;

    public static double GRIPPER_OPEN_R2V2 = 0.4, GRIPPER_CLOSED_R2V2 = 0.5;

    public static double BEACON_ARM_LOAD_R2V2 = 1, BEACON_ARM_STORAGE_R2V2 = 0.6, BEACON_ARM_DELIVERY_R2V2 = 0.8;

    public static void setActiveBot() {
        if (activeBot == 0) {
            LIFT_FULL_RETRACTION = LIFT_FULL_RETRACTION_Barney;
            LIFT_INTAKE = LIFT_INTAKE_Barney;
            LIFT_GROUND_JUNCTION = LIFT_GROUND_JUNCTION_Barney;
            LIFT_LOW_JUNCTION = LIFT_LOW_JUNCTION_Barney;
            LIFT_MEDIUM_JUNCTION = LIFT_MEDIUM_JUNCTION_Barney;
            LIFT_HIGH_JUNCTION = LIFT_HIGH_JUNCTION_Barney;

            LIFT_pE = LIFT_pE_Barney;
            LIFT_pR = LIFT_pR_Barney;
            LIFT_i = LIFT_i_Barney;
            LIFT_d = LIFT_d_Barney;

            ARM_INIT = ARM_INIT_Barney;
            ARM_INTAKE = ARM_INTAKE_Barney;
            ARM_DELIVERY = ARM_DELIVERY_Barney;

            WRIST_INIT = WRIST_INIT_Barney;
            WRIST_INTAKE = WRIST_INTAKE_Barney;
            WRIST_DELIVERY = WRIST_DELIVERY_Barney;

            GRIPPER_OPEN = GRIPPER_OPEN_Barney;
            GRIPPER_CLOSED = GRIPPER_CLOSED_Barney;

            BEACON_ARM_LOAD = BEACON_ARM_LOAD_Barney;
            BEACON_ARM_STORAGE = BEACON_ARM_STORAGE_Barney;
            BEACON_ARM_DELIVERY = BEACON_ARM_DELIVERY_Barney;
        }
        if (activeBot == 1) {
            LIFT_FULL_RETRACTION = LIFT_FULL_RETRACTION_R2V2;
            LIFT_INTAKE = LIFT_INTAKE_R2V2;
            LIFT_GROUND_JUNCTION = LIFT_GROUND_JUNCTION_R2V2;
            LIFT_LOW_JUNCTION = LIFT_LOW_JUNCTION_R2V2;
            LIFT_MEDIUM_JUNCTION = LIFT_MEDIUM_JUNCTION_R2V2;
            LIFT_HIGH_JUNCTION = LIFT_HIGH_JUNCTION_R2V2;

            LIFT_pE = LIFT_p_R2V2;
            LIFT_i = LIFT_i_R2V2;
            LIFT_d = LIFT_d_R2V2;

            ARM_INIT = ARM_INIT_R2V2;
            ARM_INTAKE = ARM_INTAKE_R2V2;
            ARM_DELIVERY = ARM_DELIVERY_R2V2;

            WRIST_INIT = WRIST_INIT_R2V2;
            WRIST_INTAKE = WRIST_INTAKE_R2V2;
            WRIST_DELIVERY = WRIST_DELIVERY_R2V2;

            GRIPPER_OPEN = GRIPPER_OPEN_R2V2;
            GRIPPER_CLOSED = GRIPPER_CLOSED_R2V2;

            BEACON_ARM_LOAD = BEACON_ARM_LOAD_R2V2;
            BEACON_ARM_STORAGE = BEACON_ARM_STORAGE_R2V2;
            BEACON_ARM_DELIVERY = ARM_DELIVERY_R2V2;

        }
    }
}
