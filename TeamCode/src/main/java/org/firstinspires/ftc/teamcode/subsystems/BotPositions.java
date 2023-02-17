package org.firstinspires.ftc.teamcode.subsystems;

import android.drm.DrmStore;

import com.acmerobotics.dashboard.config.Config;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.util.RegressionUtil;

import java.util.Arrays;

@Config
public class BotPositions {
    public static boolean isBarney = false;
    //true = Barney
    //false = R2V2

    public static int LIFT_FULL_RETRACTION_Barney = 0, LIFT_INTAKE_Barney = 50, LIFT_TRAVEL_Barney = 100, LIFT_LOW_JUNCTION_Barney = 275, LIFT_MEDIUM_JUNCTION_Barney = 550, LIFT_HIGH_JUNCTION_Barney = 800;

    public static double LIFT_pE_Barney = 0.01, LIFT_pR_Barney = 0.0001, LIFT_i_Barney = 0, LIFT_d_Barney = 0;

    public static double ARM_INIT_Barney = 0.6, ARM_INTAKE_Barney = 0.76, ARM_DELIVERY_Barney = 0.4, ARM_TRAVEL_Barney = 0.65, ARM_AUTO_END_Barney = 0.5;

    public static double WRIST_INIT_Barney = 0.8, WRIST_INTAKE_Barney = 0.2, WRIST_DELIVERY_Barney = 0.9, WRIST_TRAVEL_Barney = 0.1;

    public static double GRIPPER_OPEN_Barney = 0.5, GRIPPER_CLOSED_Barney = 0.20; // 0.55, 0.3 // 0.62, 0.73

    public static double BEACON_ARM_LOAD_Barney = 0.99, BEACON_ARM_STORAGE_Barney = 0.5, BEACON_ARM_DELIVERY_Barney = 0.75, BEACON_ARM_TRAVEL_Barney = 0.55;


    public static int LIFT_INTAKE_AUTO_R2V2 = 25, LIFT_INTAKE_R2V2 = -10, LIFT_TRAVEL_R2V2 = -10, LIFT_LOW_JUNCTION_R2V2 = 320, LIFT_MEDIUM_JUNCTION_R2V2 = 700, LIFT_HIGH_JUNCTION_R2V2 = 1100, AUTO_LIFT_HIGH_JUNCTION_R2V2 = 1100;//1175

    public static int[] STACK_POSITIONS_R2V2 = {-10, 35, 85, 140, 230};//0 = bottom cone, 4 = top cone

    public static double LIFT_p_R2V2 = 0.01, LIFT_i_R2V2 = 0.2, LIFT_d_R2V2 = 0.0001;

    public static double ARM_INIT_R2V2 = 0.56, ARM_INTAKE_R2V2 = 0.64, ARM_DELIVERY_R2V2 = 0.38, ARM_TRAVEL_R2V2 = 0.53, ARM_AUTO_END_R2V2 = 0.45;

    public static double WRIST_INIT_R2V2 = 0.78, WRIST_INTAKE_R2V2 = 0.35, WRIST_DELIVERY_R2V2 = 0.95, WRIST_TRAVEL_R2V2 = 0.3;

    public static double GRIPPER_OPEN_R2V2 = 0.5, GRIPPER_CLOSED_R2V2 = 0.15, GRIPPER_OPEN_AUTO_R2V2 = 0.6;

    public static double BEACON_ARM_LOAD_R2V2 = 1, BEACON_ARM_STORAGE_R2V2 = 0.6, BEACON_ARM_DELIVERY_R2V2 = 0.8;

    public static double BATWING_DEPLOYED_R2V2 = 0.66, BATWING_DEPLOYED_LOW_JUNCTION_R2V2 = 0.66, BATWING_RETRACTED_R2V2 = 0.85, BATWING_STORAGE_R2V2 = 0.2;
}
