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
import com.jme3.util.TangentBinormalGenerator;

/**
 * This is the Main Class of your Game. It should boot up your game and do initial initialisation
 * Move your Logic into AppStates or Controls or other java classes
 */
public class Armvisualiser extends SimpleApplication {
    private static final float SCALE_DOWN_FACTOR = 1; // 0.01f

    public static void main(String[] args) {
        Armvisualiser app = new Armvisualiser();
        // app.showSettings = false;
        AppSettings settings = new AppSettings(false);
        settings.setFullscreen(true);
        settings.setVSync(false);
        settings.setGammaCorrection(false);
        settings.setFrameRate(60);
        settings.setResolution(1920, 1080);
        app.setSettings(settings);
        app.start();
    }

    private Geometry clawGeom;
    private Node arm1, arm2;

    private BitmapText hudText;

    @Override
    public void simpleInitApp() {
        cam.getLocation().addLocal(0, 30, 100);
        cam.update();
        flyCam.setMoveSpeed(100);

        Box floor = new Box(1000, 0.1f, 1000);
        Geometry floorGeom = new Geometry("Floor", floor);

        Box base = new Box(1 * SCALE_DOWN_FACTOR, 1 * SCALE_DOWN_FACTOR, 1 * SCALE_DOWN_FACTOR);
        Geometry baseGeom = new Geometry("Box", base);

        Box baseTower = new Box(1 * SCALE_DOWN_FACTOR, 0.5f * (float) Constants.ArmConstants.ORIGIN_HEIGHT * SCALE_DOWN_FACTOR, 1 * SCALE_DOWN_FACTOR);
        Geometry baseTowerGeom = new Geometry("Box", baseTower);

        Node baseNode = new Node("baseNode");

        arm1 = new Node("arm1");
        arm2 = new Node("arm2");
        Node claw = new Node("claw");

        Box limb1 = new Box(1 * SCALE_DOWN_FACTOR, 0.5f * (float) Constants.ArmConstants.LIMB1_LENGTH * SCALE_DOWN_FACTOR, 1 * SCALE_DOWN_FACTOR);
        Geometry limb1Geom = new Geometry("Box", limb1);

        Box limb2 = new Box(1 * SCALE_DOWN_FACTOR, 0.5f * (float) Constants.ArmConstants.LIMB2_LENGTH * SCALE_DOWN_FACTOR, 1 * SCALE_DOWN_FACTOR);
        Geometry limb2Geom = new Geometry("Box", limb2);

        TangentBinormalGenerator.generate(baseTowerGeom.getMesh());
        TangentBinormalGenerator.generate(limb1Geom.getMesh());
        TangentBinormalGenerator.generate(limb2Geom.getMesh());

        Box clawB = new Box(1 * SCALE_DOWN_FACTOR, 1 * SCALE_DOWN_FACTOR, 1 * SCALE_DOWN_FACTOR);
        clawGeom = new Geometry("Box", clawB);

        rootNode.attachChild(floorGeom);
        rootNode.attachChild(baseNode);
        baseNode.attachChild(baseGeom);
        baseNode.attachChild(baseTowerGeom);
        baseNode.attachChild(arm1);

        arm1.attachChild(arm2);
        arm1.attachChild(limb1Geom);

        arm2.attachChild(limb2Geom);
        arm2.attachChild(claw);

        claw.attachChild(clawGeom);

        baseNode.center();
        baseTowerGeom.move(0, ((float)Constants.ArmConstants.ORIGIN_HEIGHT * SCALE_DOWN_FACTOR) / 2, 0);
        arm1.move(0, ((float)Constants.ArmConstants.ORIGIN_HEIGHT * SCALE_DOWN_FACTOR), 0);
        arm2.move(0, ((float)Constants.ArmConstants.LIMB1_LENGTH * SCALE_DOWN_FACTOR), 0);
        claw.move(0, ((float)Constants.ArmConstants.LIMB2_LENGTH * SCALE_DOWN_FACTOR), 0);
        limb1Geom.move(0, ((float)Constants.ArmConstants.LIMB1_LENGTH * SCALE_DOWN_FACTOR) / 2, 0);
        limb2Geom.move(0, ((float)Constants.ArmConstants.LIMB2_LENGTH * SCALE_DOWN_FACTOR) / 2, 0);
        arm1.rotate(0, 0, (float) Math.toRadians(180f + Constants.ArmConstants.ARM_1_INITIAL_ANGLE));
        arm2.rotate(0, 0, (float) Math.toRadians(180f - Constants.ArmConstants.ARM_2_INITIAL_ANGLE));

        Material mat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        Material mat2 = mat.clone(), mat3 = mat.clone(), mat4 = mat.clone(), mat5 = mat.clone(), mat6 = mat.clone();
        mat.setColor("Color", ColorRGBA.Blue);
        mat2.setColor("Color", ColorRGBA.Red);
        mat3.setColor("Color", ColorRGBA.Green);
        mat4.setColor("Color", ColorRGBA.Yellow);
        mat5.setColor("Color", ColorRGBA.Orange);
        mat6.setColor("Color", ColorRGBA.White);

        DirectionalLight sun = new DirectionalLight();
        sun.setDirection(new Vector3f(-0.5f, -0.5f, -0.5f).normalizeLocal());
        sun.setColor(ColorRGBA.Brown);
        rootNode.addLight(sun);
        rootNode.setShadowMode(RenderQueue.ShadowMode.CastAndReceive);

        final int SHADOWMAP_SIZE = 4096;
        DirectionalLightShadowRenderer dlsr = new DirectionalLightShadowRenderer(assetManager, SHADOWMAP_SIZE, 3);
        dlsr.setLight(sun);
        viewPort.addProcessor(dlsr);

        baseGeom.setMaterial(mat);
        baseTowerGeom.setMaterial(mat2);
        limb1Geom.setMaterial(mat3);
        limb2Geom.setMaterial(mat4);
        clawGeom.setMaterial(mat5);
        floorGeom.setMaterial(mat6);

        initKeys();

        setDisplayFps(false);
        setDisplayStatView(false);

        hudText = new BitmapText(guiFont);
        hudText.setSize(guiFont.getCharSet().getRenderedSize());
        hudText.setColor(ColorRGBA.Black);
        hudText.setText("TEST");
        hudText.setLocalTranslation(10, hudText.getLineHeight() * 10, 0);
        guiNode.attachChild(hudText);

        hudText.setQueueBucket(RenderQueue.Bucket.Gui);

        updateHUDText();
    }

    private void initKeys() {
        inputManager.addMapping("Arm1+", new KeyTrigger(KeyInput.KEY_J));
        inputManager.addMapping("Arm1-", new KeyTrigger(KeyInput.KEY_L));
        inputManager.addMapping("Arm2+", new KeyTrigger(KeyInput.KEY_COMMA));
        inputManager.addMapping("Arm2-", new KeyTrigger(KeyInput.KEY_PERIOD));
        inputManager.addMapping("Turret+", new KeyTrigger(KeyInput.KEY_NUMPAD4));
        inputManager.addMapping("Turret-", new KeyTrigger(KeyInput.KEY_NUMPAD6));

        inputManager.addListener(analogListener, "Arm1+", "Arm1-", "Arm2+", "Arm2-", "Turret+", "Turret-");
    }

    private void updateHUDText() {
        var arm1angdeg = arm1.getLocalRotation().toAngles(null);
        var arm2angdeg = arm2.getLocalRotation().toAngles(null);
        var turretangdeg = rootNode.getLocalRotation().toAngles(null);

        double arm1AngleDeg = Math.abs((arm1angdeg[2] * FastMath.RAD_TO_DEG) % 360) % 180;
        double arm2AngleDeg = Math.abs(arm2angdeg[2] * FastMath.RAD_TO_DEG);
        double turretAngleDeg = (360 - turretangdeg[1] * FastMath.RAD_TO_DEG) % 360;

        double[] fkuValues = ForwardKinematicsUtil.getCoordinatesFromAngles(arm1AngleDeg, arm2AngleDeg, turretAngleDeg);

        var clawLocation = clawGeom.getWorldTranslation();
        clawLocation = clawLocation.divide(SCALE_DOWN_FACTOR);
        double[] ikuValues = InverseKinematicsUtil.getAnglesFromCoordinates(clawLocation.x, clawLocation.y, clawLocation.z, false);

        hudText.setText("VALUES:\nIKU (DEG): " + ikuValues[0] + ", " + ikuValues[1] + ", " + ikuValues[2]
                + "\nFKU (POS): " + fkuValues[0] + ", " + (42 + fkuValues[1]) + ", " + fkuValues[2]
                + "\n\nREAL:\nANGLES (DEG): Arm1: " + arm1AngleDeg + ", Arm2: " + arm2AngleDeg + ", Turret: " + turretAngleDeg
                + "\nPOSITION: " + clawLocation.x + ", " + clawLocation.y + ", " + clawLocation.z
        );
    }

    private final AnalogListener analogListener = new AnalogListener() {
        @Override
        public void onAnalog(String name, float value, float tpf) {
            switch (name) {
                case "Arm1+":
                    arm1.rotate(0, 0, value);
                    updateHUDText();
                    break;
                case "Arm1-":
                    arm1.rotate(0, 0, -value);
                    updateHUDText();
                    break;
                case "Arm2+":
                    arm2.rotate(0, 0, value);
                    updateHUDText();
                    break;
                case "Arm2-":
                    arm2.rotate(0, 0, -value);
                    updateHUDText();
                    break;
                case "Turret+":
                    rootNode.rotate(0, value, 0);
                    updateHUDText();
                    break;
                case "Turret-":
                    rootNode.rotate(0, -value, 0);
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
