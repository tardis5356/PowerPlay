package org.firstinspires.ftc.teamcode.subsystems;

import android.drm.DrmStore;

import com.acmerobotics.dashboard.config.Config;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.util.RegressionUtil;

import java.util.Arrays;

@Config
public class BotPositions {
    public static boolean isBarney = true;
    //true = Barney
    //false = R2V2/V3PO

    public static int LIFT_INTAKE_AUTO_Barney = 25, LIFT_INTAKE_Barney = -10, LIFT_TRAVEL_Barney = -10, LIFT_LOW_JUNCTION_Barney = 350, LIFT_MEDIUM_JUNCTION_Barney = 770, LIFT_HIGH_JUNCTION_Barney = 1150, AUTO_LIFT_HIGH_JUNCTION_Barney = 1100;//1175

    public static int[] STACK_POSITIONS_Barney = {-10, 45, 100, 170, 210, 250};//0 = bottom cone, 4 = top cone MTI SUBMISSION

    public static double LIFT_p_Barney = 0.01, LIFT_i_Barney = 0, LIFT_d_Barney = 0.0001;

    public static double ARM_INIT_Barney = 0.5, ARM_INTAKE_Barney = 0.08, ARM_AUTO_INTAKE_WAYPOINT_Barney = 0.60, ARM_DELIVERY_Barney = 0.78, ARM_DELIVERY_DROP_Barney = 0.9, ARM_TRAVEL_Barney = 0.53, ARM_STORAGE_Barney = 0.5, ARM_AUTO_END_Barney = 0.48; //old deliver 0.365

    public static double WRIST_INIT_Barney = 0.78, WRIST_INTAKE_Barney = 1, WRIST_AUTO_INTAKE_WAYPOINT_Barney = 0.05, WRIST_DELIVERY_Barney = 0.3, WRIST_TRAVEL_Barney = 0.3;

    public static double GRIPPER_OPEN_Barney = 0.6, GRIPPER_CLOSED_Barney = 0.4, GRIPPER_OPEN_AUTO_Barney = 0.6;

    public static double BEACON_ARM_LOAD_Barney = 1, BEACON_ARM_STORAGE_Barney = 0.6, BEACON_ARM_DELIVERY_Barney = 0.8;

    public static double BATWING_DEPLOYED_Barney = 0.58, BATWING_RETRACTED_Barney = 0.8, BATWING_STORAGE_Barney = 0.2, BATWING_DEPLOYED_LOW_JUNCTION_Barney = 0.62;


    public static int LIFT_INTAKE_AUTO_R2V2 = 25, LIFT_INTAKE_R2V2 = -10, LIFT_TRAVEL_R2V2 = -10, LIFT_LOW_JUNCTION_R2V2 = 350, LIFT_MEDIUM_JUNCTION_R2V2 = 770, LIFT_HIGH_JUNCTION_R2V2 = 1150, AUTO_LIFT_HIGH_JUNCTION_R2V2 = 1100;//1175

//    public static int[] STACK_POSITIONS_R2V2 = {-10, 65, 85, 140, 180, 210};//0 = bottom cone, 4 = top cone
//public static int[] STACK_POSITIONS_R2V2 = {-10, 0, 65, 130, 170, 200};//0 = bottom cone, 4 = top cone EXC CHAMPS
//    public static int[] STACK_POSITIONS_R2V2 = {-10, 45, 100, 170, 210, 250};//0 = bottom cone, 4 = top cone MTI SUBMISSION
    public static int[] STACK_POSITIONS_R2V2 = {-10, 45, 100, 170, 210, 250};//0 = bottom cone, 4 = top cone MTI SUBMISSION

    public static double LIFT_p_R2V2 = 0.01, LIFT_i_R2V2 = 0, LIFT_d_R2V2 = 0.0001;

    public static double ARM_INIT_R2V2 = 0.565, ARM_INTAKE_R2V2 = 0.65, ARM_AUTO_INTAKE_WAYPOINT_R2V2 = 0.60, ARM_DELIVERY_R2V2 = 0.39, ARM_TRAVEL_R2V2 = 0.53, ARM_STORAGE_R2V2 = 0.5, ARM_AUTO_END_R2V2 = 0.48; //old deliver 0.365

    public static double WRIST_INIT_R2V2 = 0.78, WRIST_INTAKE_R2V2 = 0.35, WRIST_AUTO_INTAKE_WAYPOINT_R2V2 = 0.05, WRIST_DELIVERY_R2V2 = 0.95, WRIST_TRAVEL_R2V2 = 0.3;

    public static double GRIPPER_OPEN_R2V2 = 0.5, GRIPPER_CLOSED_R2V2 = 0.16, GRIPPER_OPEN_AUTO_R2V2 = 0.6;

    public static double BEACON_ARM_LOAD_R2V2 = 1, BEACON_ARM_STORAGE_R2V2 = 0.6, BEACON_ARM_DELIVERY_R2V2 = 0.8;

    public static double BATWING_DEPLOYED_R2V2 = 0.58, BATWING_RETRACTED_R2V2 = 0.8, BATWING_STORAGE_R2V2 = 0.2, BATWING_DEPLOYED_LOW_JUNCTION_R2V2 = 0.62;
}
