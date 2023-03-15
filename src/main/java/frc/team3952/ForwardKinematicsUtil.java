package frc.team3952;

import frc.team3952.Constants.ArmConstants;

/**
 * Forward Kinematics helper for the arm
 */
public final class ForwardKinematicsUtil {
    private ForwardKinematicsUtil() {
        throw new UnsupportedOperationException("ForwardKinematicsUtil is a utility class and cannot be instantiated!");
    }

    /**
     * Given the angles of the limbs and the turret, find the coordinates
     *
     * @param a1          First limb's angle in degrees
     * @param a2          Second limb's angle in degrees
     * @param turretAngle Turret's angle in degrees
     * @return the coordinates x, y, z
     */
    public static double[] getCoordinatesFromAngles(double a1, double a2, double turretAngle) {
        double extendedDist = ArmConstants.LIMB1_LENGTH * Math.sin(Math.toRadians(a1)) + ArmConstants.LIMB2_LENGTH * Math.sin(Math.toRadians(a2 - a1));
        double z = Math.sin(Math.toRadians(turretAngle)) * extendedDist;
        double y = -ArmConstants.LIMB1_LENGTH * Math.cos(Math.toRadians(a1)) + ArmConstants.LIMB2_LENGTH * Math.cos(Math.toRadians(a2 - a1)) + ArmConstants.ORIGIN_HEIGHT;
        double x = Math.cos(Math.toRadians(turretAngle)) * extendedDist;

        // Solve for the distance from origin -> claw
        return new double[]{x, y, z};
    }
}