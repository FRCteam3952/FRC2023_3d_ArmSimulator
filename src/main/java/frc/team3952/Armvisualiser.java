package frc.team3952;

import com.jme3.app.SimpleApplication;
import com.jme3.font.BitmapText;
import com.jme3.input.KeyInput;
import com.jme3.input.controls.AnalogListener;
import com.jme3.input.controls.KeyTrigger;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.shape.Box;
import com.jme3.shadow.DirectionalLightShadowRenderer;
import com.jme3.system.AppSettings;

/**
 * This is the Main Class of your Game. It should boot up your game and do initial initialisation
 * Move your Logic into AppStates or Controls or other java classes
 */
public class Armvisualiser extends SimpleApplication {
    /**
     * JME's built-in rotation system doesn't suit our usage, so I've made my own rotation tracker
     */
    private double arm1AngleRad = Constants.ArmConstants.ARM_1_INITIAL_ANGLE * FastMath.DEG_TO_RAD, arm2AngleRad = Constants.ArmConstants.ARM_2_INITIAL_ANGLE * FastMath.DEG_TO_RAD;

    // These need global scope for access in other locations.
    private Geometry clawGeom, targetPointGeom;
    private Node baseNode, arm1, arm2;
    private BitmapText hudText;

    /**
     * The target position of the claw. Initialized to the claw's starting position.
     */
    private double targetX = 9.0297d, targetY = 29.1975d, targetZ = 0;

    /**
     * This handles keybinds to make changes appear on the model.
     */
    private final AnalogListener analogListener = (name, value, tpf) -> {
        switch (name) {
            case "X+":
                targetX += 0.2;
                break;
            case "X-":
                targetX -= 0.2;
                break;
            case "Y+":
                targetY += 0.2;
                break;
            case "Y-":
                targetY -= 0.2;
                break;
            case "Z+":
                targetZ += 0.2;
                break;
            case "Z-":
                targetZ -= 0.2;
                break;
        }
        updateTarget();
        updateHUDText();
    };

    public static void main(String[] args) {
        Armvisualiser app = new Armvisualiser(); // Instantiate the app
        // app.showSettings = false; // This stops the settings screen from appearing. Only uncomment this line after you've set your initial settings once.
        AppSettings settings = new AppSettings(false); // Loads the settings, making sure to load the previously used one. Makes the line above work
        app.setSettings(settings);
        app.start(); // Starts the app. Runs simpleInitApp()
    }

    /**
     * Rotates node Arm1
     * @param angle radians
     */
    private void rotateArm1(double angle) {
        arm1AngleRad += angle;
        arm1.rotate(0, 0, (float) angle);
    }

    /**
     * Rotates node Arm2
     * @param angle radians
     */
    private void rotateArm2(double angle) {
        arm2AngleRad -= angle; // bruh idk why but this needs to be done???
        arm2.rotate(0, 0, (float) angle);
    }

    /**
     * This function is run once on app startup. All initialization goes in here.
     */
    @Override
    public void simpleInitApp() {
        cam.getLocation().addLocal(0, 30, 100); // Move the camera up and back a bit, so you can see the entire structure immediately
        cam.update(); // Force update the camera so the previous command takes effect immediately
        flyCam.setMoveSpeed(100); // Lets you fly faster, since the geometry is quite large

        Box floor = new Box(1000, 0.1f, 1000); // The floor
        Geometry floorGeom = new Geometry("Floor", floor);

        Box base = new Box(1, 1, 1); // This is the base of the arm
        Geometry baseGeom = new Geometry("Box", base);

        Box baseTower = new Box(1, 0.5f * (float) Constants.ArmConstants.ORIGIN_HEIGHT * 1, 1); // The supporting beam of the arm
        Geometry baseTowerGeom = new Geometry("Box", baseTower);

        // These nodes allow me to rotate things independently of each other around a known point, and to find the world-relative coordinates of parts.
        baseNode = new Node("baseNode");

        arm1 = new Node("arm1");
        arm2 = new Node("arm2");
        Node claw = new Node("claw");

        Box limb1 = new Box(1, 0.5f * (float) Constants.ArmConstants.LIMB1_LENGTH * 1, 1); // Arm limb 1
        Geometry limb1Geom = new Geometry("Box", limb1);

        Box limb2 = new Box(1, 0.5f * (float) Constants.ArmConstants.LIMB2_LENGTH * 1, 1); // Arm limb 2
        Geometry limb2Geom = new Geometry("Box", limb2);

        Box clawB = new Box(1, 1, 1); // The claw (a box)
        clawGeom = new Geometry("Box", clawB);

        Box targetPoint = new Box(1f, 1f, 1f); // The target point
        targetPointGeom = new Geometry("Box", targetPoint);

        // This attaches items into the node system (consider it a tree). The items are linked specifically so I can rotate parts like the arm limbs at the same time.
        rootNode.attachChild(floorGeom);
        rootNode.attachChild(targetPointGeom);
        rootNode.attachChild(baseNode);
        baseNode.attachChild(baseGeom);
        baseNode.attachChild(baseTowerGeom);
        baseNode.attachChild(arm1);

        arm1.attachChild(arm2); // Attach arm2 node to arm1, so that when arm1 rotates, arm2 moves with it
        arm1.attachChild(limb1Geom);

        arm2.attachChild(limb2Geom);
        arm2.attachChild(claw);

        claw.attachChild(clawGeom);

        baseNode.center(); // Make sure the base of the arm is at the origin

        // This moves the items into the correct position relative to each other
        baseTowerGeom.move(0, ((float) Constants.ArmConstants.ORIGIN_HEIGHT * 1) / 2, 0);
        arm1.move(0, ((float) Constants.ArmConstants.ORIGIN_HEIGHT * 1), 0);
        arm2.move(0, ((float) Constants.ArmConstants.LIMB1_LENGTH * 1), 0);
        claw.move(0, ((float) Constants.ArmConstants.LIMB2_LENGTH * 1), 0);
        limb1Geom.move(0, ((float) Constants.ArmConstants.LIMB1_LENGTH * 1) / 2, 0);
        limb2Geom.move(0, ((float) Constants.ArmConstants.LIMB2_LENGTH * 1) / 2, 0);

        // Set the rotation of the arm limbs to their initial positions. This code works
        arm1.rotate(0, 0, (float) Math.toRadians(180f + Constants.ArmConstants.ARM_1_INITIAL_ANGLE));
        arm2.rotate(0, 0, (float) Math.toRadians(180f - Constants.ArmConstants.ARM_2_INITIAL_ANGLE));

        // Colors the limbs in different colors
        Material mat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        Material mat2 = mat.clone(), mat3 = mat.clone(), mat4 = mat.clone(), mat5 = mat.clone(), mat6 = mat.clone(), mat7 = mat.clone(); // I was too lazy to keep initializing new materials
        mat.setColor("Color", ColorRGBA.Blue);
        mat2.setColor("Color", ColorRGBA.Red);
        mat3.setColor("Color", ColorRGBA.Green);
        mat4.setColor("Color", ColorRGBA.Yellow);
        mat5.setColor("Color", ColorRGBA.Orange);
        mat6.setColor("Color", ColorRGBA.White);
        mat7.setColor("Color", ColorRGBA.Magenta);

        DirectionalLight sun = new DirectionalLight(); // This is the light source for shadows
        sun.setDirection(new Vector3f(-0.5f, -0.5f, -0.5f).normalizeLocal()); // Positions the light source
        sun.setColor(ColorRGBA.White);
        rootNode.addLight(sun); // Puts the sun in the map
        rootNode.setShadowMode(RenderQueue.ShadowMode.CastAndReceive); // Make sure everything can cast a shadow and have a shadow casted on

        final int SHADOWMAP_SIZE = 4096; // Size of the shadow map
        DirectionalLightShadowRenderer dlsr = new DirectionalLightShadowRenderer(assetManager, SHADOWMAP_SIZE, 3); // Renders shadows
        dlsr.setLight(sun);
        viewPort.addProcessor(dlsr); // Adds the shadow renderer to the viewport

        // Make sure the limbs have their materials applied
        baseGeom.setMaterial(mat);
        baseTowerGeom.setMaterial(mat2);
        limb1Geom.setMaterial(mat3);
        limb2Geom.setMaterial(mat4);
        clawGeom.setMaterial(mat5);
        floorGeom.setMaterial(mat6);
        targetPointGeom.setMaterial(mat7);

        // Adds keybinds
        initKeys();

        // Disables the built-in debug text
        setDisplayFps(false);
        setDisplayStatView(false);

        // Adds the IKU, FKU, and real value text
        hudText = new BitmapText(guiFont);
        hudText.setSize(guiFont.getCharSet().getRenderedSize());
        hudText.setColor(ColorRGBA.Black);
        hudText.setText("TEST");
        hudText.setLocalTranslation(10, hudText.getLineHeight() * 10, 0);
        guiNode.attachChild(hudText);

        hudText.setQueueBucket(RenderQueue.Bucket.Gui); // Tells JME that this is text

        updateHUDText(); // Initial update of the text
    }

    /**
     * This method initializes the keybinds.
     */
    private void initKeys() {
        inputManager.addMapping("X+", new KeyTrigger(KeyInput.KEY_J));
        inputManager.addMapping("X-", new KeyTrigger(KeyInput.KEY_L));
        inputManager.addMapping("Y+", new KeyTrigger(KeyInput.KEY_COMMA));
        inputManager.addMapping("Y-", new KeyTrigger(KeyInput.KEY_PERIOD));
        inputManager.addMapping("Z+", new KeyTrigger(KeyInput.KEY_U));
        inputManager.addMapping("Z-", new KeyTrigger(KeyInput.KEY_O));

        inputManager.addListener(analogListener, "X+", "X-", "Y+", "Y-", "Z+", "Z-");
    }

    /**
     * This method updates the HUD text. The text has a few modifications made to it to correct obvious errors caused by our 3D model (such as angles being negative).
     */
    private void updateHUDText() {
        double[] angles = getAnglesDeg();

        boolean isFlipped = angles[1] > 180;
        if (angles[0] > 180) {
            angles[0] = 360 - angles[0];
        }

        double[] fkuValues = ForwardKinematicsUtil.getCoordinatesFromAngles(angles[0], angles[1], angles[2]);

        Vector3f clawLocation = clawGeom.getWorldTranslation(), targetLocation = targetPointGeom.getWorldTranslation();

        double[] ikuValues = InverseKinematicsUtil.getAnglesFromCoordinates(clawLocation.x, clawLocation.y, clawLocation.z, isFlipped);

        // Updates the text
        hudText.setText("VALUES:\nIKU (DEG): " + ikuValues[0] + ", " + ikuValues[1] + ", " + ikuValues[2]+ ", flipped: " + isFlipped
                + "\nFKU (POS): " + fkuValues[0] + ", " + (42 + fkuValues[1]) + ", " + fkuValues[2]
                + "\n\nREAL:\nANGLES (DEG): Arm1: " + angles[0] + ", Arm2: " + angles[1] + ", Turret: " + angles[2]
                + "\nPOSITION: " + clawLocation.x + ", " + clawLocation.y + ", " + clawLocation.z
                + "\nTARGET: " + targetLocation.x + ", " + targetLocation.y + ", " + targetLocation.z
        );
    }

    /**
     * Returns the known angles of the arm components.
     *
     * @return [arm1AngleDeg, arm2AngleDeg, turretAngleDeg]
     */
    private double[] getAnglesDeg() {
        var tempTurretAngDeg = baseNode.getLocalRotation().toAngles(null);


        double arm1AngleDeg = Math.abs(arm1AngleRad * FastMath.RAD_TO_DEG) % 360; // Math.abs((tempArm1AngDeg[2] * FastMath.RAD_TO_DEG) % 360) % 180;
        double arm2AngleDeg = Math.abs(arm2AngleRad * FastMath.RAD_TO_DEG) % 360; // Math.abs(tempArm2AngDeg[2] * FastMath.RAD_TO_DEG);
        double turretAngleDeg = (360 - tempTurretAngDeg[1] * FastMath.RAD_TO_DEG) % 360;

        return new double[]{arm1AngleDeg, arm2AngleDeg, turretAngleDeg};
    }

    /**
     * This method updates the angles of the arm components using IKU to position the claw at the target position.
     */
    private void ikuUpdateAngles() {
        double[] angles = getAnglesDeg();

        if (angles[0] > 180) {
            angles[0] = 360 - angles[0];
        }
        if (angles[1] > 180) {
            angles[1] = 360 - angles[1];
        }

        //var clawLocation = clawGeom.getWorldTranslation();
        double[] ikuValues = InverseKinematicsUtil.getAnglesFromCoordinates(targetX, targetY, targetZ, false);

        float adjustArm1AngRad = (float) Math.toRadians(360 - (angles[0] - ikuValues[0]));
        float adjustArm2AngRad = (float) Math.toRadians(angles[1] - ikuValues[1]);
        float adjustTurretAngRad = (float) Math.toRadians(angles[2] - ikuValues[2]);

        baseNode.rotate(0, adjustTurretAngRad, 0);
        rotateArm1(adjustArm1AngRad);
        rotateArm2(adjustArm2AngRad);
    }

    /**
     * Updates the target position and the arm angles to match.
     */
    private void updateTarget() {
        targetPointGeom.center().move((float) targetX, (float) targetY, (float) targetZ);
        ikuUpdateAngles();
    }

    @Override
    public void simpleUpdate(float tpf) {
        //this method will be called every game tick and can be used to make updates
    }

    @Override
    public void simpleRender(RenderManager rm) {
        //add render code here (if any)
    }
}
