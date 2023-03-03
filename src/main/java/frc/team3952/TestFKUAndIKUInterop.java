package frc.team3952;

public final class TestFKUAndIKUInterop {
    private TestFKUAndIKUInterop() {
        throw new UnsupportedOperationException("Tests is a testing class and cannot be instantiated. Instead, use an IDE and run the main method in this class.");
    }

    public static final double[][] TEST_ANGLES = new double[360][2];
    public static final double TEST_POS_DELTA_OK = 0.01;
    public static final double TEST_ANGLE_DELTA_OK = 2.0;

    static {
        for(int i = 0; i < TEST_ANGLES.length; i++) {
            TEST_ANGLES[i] = new double[] {i / 360f, (i + 10) / 360f};
        }
    }

    public static void main(String[] args) {
        int zeroCount = 0;
        for(int i = 0; i < TEST_ANGLES.length; i++) {
            var angles = TEST_ANGLES[i];
            var randomRotAngle = Math.random() * 360;
            var coords = ForwardKinematicsUtil.getCoordinatesFromAngles(angles[0], angles[1], randomRotAngle);
            var backToAngles = InverseKinematicsUtil.getAnglesFromCoordinates(coords[0], coords[1], coords[2], false);

            if(backToAngles[0] == 0.0 || backToAngles[1] == 0.0 || backToAngles[2] == 0.0) {
                System.out.println("0.0 given, skipping.");
                zeroCount++;
                continue;
            }

            if(Math.abs(backToAngles[0] - angles[0]) > TEST_POS_DELTA_OK) {
                System.out.println("FAILED 0, expected: " + angles[0] + ", got: " + backToAngles[0]);
            }

            if(Math.abs(backToAngles[1] - angles[1]) > TEST_POS_DELTA_OK) {
                System.out.println("FAILED 1, expected: " + angles[1] + ", got: " + backToAngles[1]);
            }

            if(Math.abs(backToAngles[2] - randomRotAngle) > TEST_ANGLE_DELTA_OK && Math.abs(360 - backToAngles[2] - randomRotAngle) > TEST_ANGLE_DELTA_OK) {
                System.out.println("FAILED 2, expected: " + randomRotAngle + ", got: " + backToAngles[2]);
            }
        }
        System.out.println("Skipped " + zeroCount + " tests out of 360.");
    }
}