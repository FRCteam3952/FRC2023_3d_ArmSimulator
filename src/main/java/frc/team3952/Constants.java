// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team3952;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
    /**
     * Constants for the Arm
     */
    public static class ArmConstants {

        // All constants are in inches
        public static final double ORIGIN_HEIGHT = 42.0;
        public static final double LIMB1_LENGTH = 32.5;
        public static final double LIMB2_LENGTH = 19.5;

        public static final double MIN_HOR_DISTANCE = 5; // change to be correct later
        public static final double MAX_REACH_REDUCTION = 2; // change to be correct later

        public static final double ARM_1_INITIAL_ANGLE = 10.0;
        public static final double ARM_2_INITIAL_ANGLE = 20.0;
    }
}
