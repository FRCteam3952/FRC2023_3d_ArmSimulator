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
    public static void main(String[] args) {
        Armvisualiser app = new Armvisualiser(); // Instantiate the app
        // app.showSettings = false; // This stops the settings screen from appearing. Only uncomment this line after you've set your initial settings once.
        AppSettings settings = new AppSettings(false); // Loads the settings, making sure to load the previously used one. Makes the line above work
        app.setSettings(settings);
        app.start(); // Starts the app. Runs simpleInitApp()
    }

    /**
     * JME's built-in rotation system doesn't suit our usage, so I've made my own rotation tracker
     */
    private double arm1AngleRad = Constants.ArmConstants.ARM_1_INITIAL_ANGLE * FastMath.DEG_TO_RAD, arm2AngleRad = Constants.ArmConstants.ARM_2_INITIAL_ANGLE * FastMath.DEG_TO_RAD;

    // These need global scope for access in other locations.
    private Geometry clawGeom;
    private Node baseNode, arm1, arm2;
    private BitmapText hudText;

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

        // This attaches items into the node system (consider it a tree). The items are linked specifically so I can rotate parts like the arm limbs at the same time.
        rootNode.attachChild(floorGeom);
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
        baseTowerGeom.move(0, ((float)Constants.ArmConstants.ORIGIN_HEIGHT * 1) / 2, 0);
        arm1.move(0, ((float)Constants.ArmConstants.ORIGIN_HEIGHT * 1), 0);
        arm2.move(0, ((float)Constants.ArmConstants.LIMB1_LENGTH * 1), 0);
        claw.move(0, ((float)Constants.ArmConstants.LIMB2_LENGTH * 1), 0);
        limb1Geom.move(0, ((float)Constants.ArmConstants.LIMB1_LENGTH * 1) / 2, 0);
        limb2Geom.move(0, ((float)Constants.ArmConstants.LIMB2_LENGTH * 1) / 2, 0);

        // Set the rotation of the arm limbs to their initial positions. This code works
        arm1.rotate(0, 0, (float) Math.toRadians(180f + Constants.ArmConstants.ARM_1_INITIAL_ANGLE));
        arm2.rotate(0, 0, (float) Math.toRadians(180f - Constants.ArmConstants.ARM_2_INITIAL_ANGLE));

        // Colors the limbs in different colors
        Material mat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        Material mat2 = mat.clone(), mat3 = mat.clone(), mat4 = mat.clone(), mat5 = mat.clone(), mat6 = mat.clone(); // I was too lazy to keep initializing new materials
        mat.setColor("Color", ColorRGBA.Blue);
        mat2.setColor("Color", ColorRGBA.Red);
        mat3.setColor("Color", ColorRGBA.Green);
        mat4.setColor("Color", ColorRGBA.Yellow);
        mat5.setColor("Color", ColorRGBA.Orange);
        mat6.setColor("Color", ColorRGBA.White);

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
        inputManager.addMapping("Arm1+", new KeyTrigger(KeyInput.KEY_J));
        inputManager.addMapping("Arm1-", new KeyTrigger(KeyInput.KEY_L));
        inputManager.addMapping("Arm2+", new KeyTrigger(KeyInput.KEY_COMMA));
        inputManager.addMapping("Arm2-", new KeyTrigger(KeyInput.KEY_PERIOD));
        inputManager.addMapping("Turret+", new KeyTrigger(KeyInput.KEY_NUMPAD4));
        inputManager.addMapping("Turret-", new KeyTrigger(KeyInput.KEY_NUMPAD6));

        inputManager.addListener(analogListener, "Arm1+", "Arm1-", "Arm2+", "Arm2-", "Turret+", "Turret-");
    }

    /**
     * This method updates the HUD text. The text has a few modifications made to it to correct obvious errors caused by our 3D model (such as angles being negative).
     */
    private void updateHUDText() {
        var tempTurretAngDeg = baseNode.getLocalRotation().toAngles(null);

        // This is a bit of a hack, but since our arm1 limb is inverted, the rotation needs to be un-inverted properly.
        // The other ones have other weird tidbits applied to them, but this works
        double arm1AngleDeg = Math.abs(arm1AngleRad * FastMath.RAD_TO_DEG) % 360; // Math.abs((tempArm1AngDeg[2] * FastMath.RAD_TO_DEG) % 360) % 180;
        double arm2AngleDeg = Math.abs(arm2AngleRad * FastMath.RAD_TO_DEG) % 360; // Math.abs(tempArm2AngDeg[2] * FastMath.RAD_TO_DEG);
        double turretAngleDeg = (360 - tempTurretAngDeg[1] * FastMath.RAD_TO_DEG) % 360;

        boolean isFlipped = arm2AngleDeg > 180;
        if(arm1AngleDeg > 180) {
            arm1AngleDeg = 360 - arm1AngleDeg;
        }

        double[] fkuValues = ForwardKinematicsUtil.getCoordinatesFromAngles(arm1AngleDeg, arm2AngleDeg, turretAngleDeg);

        var clawLocation = clawGeom.getWorldTranslation();

        double[] ikuValues = InverseKinematicsUtil.getAnglesFromCoordinates(clawLocation.x, clawLocation.y, clawLocation.z, isFlipped);

        // Updates the text
        hudText.setText("VALUES:\nIKU (DEG): " + ikuValues[0] + ", " + ikuValues[1] + ", " + ikuValues[2] + ", flipped: " + isFlipped
                + "\nFKU (POS): " + fkuValues[0] + ", " + (42 + fkuValues[1]) + ", " + fkuValues[2]
                + "\n\nREAL:\nANGLES (DEG): Arm1: " + arm1AngleDeg + ", Arm2: " + arm2AngleDeg + ", Turret: " + turretAngleDeg
                + "\nPOSITION: " + clawLocation.x + ", " + clawLocation.y + ", " + clawLocation.z
        );
    }

    /**
     * This handles keybinds to make changes happen on the model.
     */
    private final AnalogListener analogListener = new AnalogListener() {
        @Override
        public void onAnalog(String name, float value, float tpf) {
            value *= 2f;
            switch (name) {
                case "Arm1+":
                    arm1.rotate(0, 0, value);
                    arm1AngleRad += value;
                    updateHUDText();
                    break;
                case "Arm1-":
                    arm1.rotate(0, 0, -value);
                    arm1AngleRad -= value;
                    updateHUDText();
                    break;
                case "Arm2+":
                    arm2.rotate(0, 0, value);
                    arm2AngleRad -= value;
                    updateHUDText();
                    break;
                case "Arm2-":
                    arm2.rotate(0, 0, -value);
                    arm2AngleRad += value;
                    updateHUDText();
                    break;
                case "Turret+":
                    baseNode.rotate(0, value, 0);
                    updateHUDText();
                    break;
                case "Turret-":
                    baseNode.rotate(0, -value, 0);
                    updateHUDText();
                    break;
            }
        }
    };

    @Override
    public void simpleUpdate(float tpf) {
        //this method will be called every game tick and can be used to make updates
    }

    @Override
    public void simpleRender(RenderManager rm) {
        //add render code here (if any)
    }
}
