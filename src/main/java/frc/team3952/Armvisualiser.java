package frc.team3952;

import com.jme3.app.DebugKeysAppState;
import com.jme3.app.FlyCamAppState;
import com.jme3.app.SimpleApplication;
import com.jme3.app.StatsAppState;
import com.jme3.app.state.ConstantVerifierState;
import com.jme3.audio.AudioListenerState;
import com.jme3.font.BitmapText;
import com.jme3.input.CameraInput;
import com.jme3.input.KeyInput;
import com.jme3.input.controls.ActionListener;
import com.jme3.input.controls.AnalogListener;
import com.jme3.input.controls.KeyTrigger;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.queue.RenderQueue;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.shape.Box;
import com.jme3.shadow.DirectionalLightShadowRenderer;
import com.jme3.system.AppSettings;
import edu.wpi.first.math.WPIMathJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.util.CombinedRuntimeLoader;
import edu.wpi.first.util.WPIUtilJNI;

import java.io.IOException;
import java.util.Arrays;
import java.util.Scanner;

/**
 * This is the Main Class of your Game. It should boot up your game and do initial initialisation
 * Move your Logic into AppStates or Controls or other java classes
 */
public class Armvisualiser extends SimpleApplication {
    /**
     * JME's built-in rotation system doesn't suit our usage, so I've made my own rotation tracker
     */
    private double arm1AngleRad = Constants.ArmConstants.ARM_1_INITIAL_ANGLE * FastMath.DEG_TO_RAD, arm2AngleRad = Constants.ArmConstants.ARM_2_INITIAL_ANGLE * FastMath.DEG_TO_RAD, turretAngleRad = 0.0;

    // These need global scope for access in other locations.
    private Geometry clawGeom, targetPointGeom, cameraGeom;
    private Node robotNode, positionsNode, baseNode, arm1, arm2;
    private BitmapText hudText;

    /**
     * Used so that we can use the library's optimizations for the pose boxes by giving them the same material.
     */
    private Material poseMat;

    /**
     * The target position of the claw. Initialized to the claw's starting position.
     */
    private double targetX, targetY, targetZ;
    private boolean isFlipped = false;

    private final Scanner scanner;

    // Keybind handlers
    private final ActionListener actionListener = (name, pressed, tpf) -> { // When you press it, don't keep flipping while it's pressed, only flip once (equiv. to getRawButtonPressed)
        if (pressed) {
            return;
        }

        //noinspection SwitchStatementWithTooFewBranches
        switch(name) {
            case "Flip":
                isFlipped = !isFlipped;
                ikuUpdateAngles();
                break;
        }
    };

    private final AnalogListener analogListener = (name, value, tpf) -> { // Keep going while pressed (equiv. to getRawButton)
        value *= 2;
        double[] rotated;
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
            case "Turret+":
                targetZ += 0.2;
                break;
            case "Turret-":
                targetZ -= 0.2;
                break;
            case "Robot Forward":
                rotated = MathUtil.polarToXY(1, robotNode.getLocalRotation().toAngles(null)[1] * FastMath.RAD_TO_DEG + 180);
                robotNode.move(-(float) rotated[0], 0, (float) rotated[1]);
                break;
            case "Robot Backward":
                rotated = MathUtil.polarToXY(-1, robotNode.getLocalRotation().toAngles(null)[1] * FastMath.RAD_TO_DEG + 180);
                robotNode.move(-(float) rotated[0], 0, (float) rotated[1]);
                break;
            case "Robot CCW":
                robotNode.rotate(0, value * 1.5f, 0);
                break;
            case "Robot CW":
                robotNode.rotate(0, -value * 1.5f, 0);
                break;
        }
        updateTarget(true);
    };

    public Armvisualiser() {
        super(new StatsAppState(), new FlyCamAppState(), new AudioListenerState(), new DebugKeysAppState(), new ConstantVerifierState());
        double[] initialPos = ForwardKinematicsUtil.getCoordinatesFromAngles(Constants.ArmConstants.ARM_1_INITIAL_ANGLE, Constants.ArmConstants.ARM_2_INITIAL_ANGLE, 0);
        targetX = initialPos[0];
        targetY = initialPos[1];

        this.scanner = new Scanner(System.in);
        (new Thread(() -> { // Allow user input to conosle to control the GUI
            while (true) {
                String line = scanner.nextLine();
                String[] splitLine = line.split(" ");
                if(splitLine.length != 3 && splitLine.length != 6) {
                    System.out.println("Invalid input");
                    continue;
                }
                float[] pos = new float[splitLine.length];
                for(int i = 0; i < pos.length; i++) {
                    pos[i] = Float.parseFloat(splitLine[i]);
                }
                this.robotNode.setLocalTranslation(new Vector3f(pos[0], pos[1], pos[2]));
                if(pos.length == 6) {
                    this.robotNode.setLocalRotation(new Quaternion());
                    this.robotNode.rotate(0, pos[5] * FastMath.DEG_TO_RAD, 0);
                    cam.update();
                }
            }
        })).start();
    }

    public static void main(String[] args) throws InterruptedException, IOException {
        NetworkTablesJNI.Helper.setExtractOnStaticLoad(false);
        WPIUtilJNI.Helper.setExtractOnStaticLoad(false);
        WPIMathJNI.Helper.setExtractOnStaticLoad(false);

        CombinedRuntimeLoader.loadLibraries(Armvisualiser.class, "wpiutiljni", "wpimathjni", "ntcorejni");

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.startClient4("Robot 3D Gui");
        inst.setServer("localhost");
        inst.startDSClient();

        Constants.FieldConstants.GamePiecePlacementLocationConstants.poke();
        Thread.sleep(50);
        Armvisualiser app = new Armvisualiser(); // Instantiate the app
        app.showSettings = false; // This stops the settings screen from appearing. Only uncomment this line after you've set your initial settings once.
        AppSettings settings = new AppSettings(true); // Loads the settings, making sure to load the previously used one. Makes the line above work
        settings.setResolution(1280, 720);
        settings.setFullscreen(false);
        settings.setVSync(false);
        settings.setGammaCorrection(false);
        settings.setRenderer(AppSettings.LWJGL_OPENGL33); // not sure why i decided to do this, you can probably remove it if you want
        settings.setTitle("Arm Visualiser");
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
     * Rotates node baseNode
     * @param angle radians
     */
    private void rotateTurret(double angle) {
        turretAngleRad += angle;
        if(turretAngleRad > Math.PI) {
            turretAngleRad -= Math.PI * 2;
        } else if(turretAngleRad < -Math.PI) {
            turretAngleRad += Math.PI * 2;
        }
        baseNode.rotate(0, (float) angle, 0);
    }

    private void resetArm1() {
        arm1.setLocalRotation(new Quaternion());
        arm1AngleRad = 0;
    }

    private void resetArm2() {
        arm2.setLocalRotation(new Quaternion());
        arm2AngleRad = 0;
    }

    private void resetTurret() {
        baseNode.setLocalRotation(new Quaternion());
        turretAngleRad = 0;
    }

    /**
     * Sets the angle of arm1 to the specified angle
     * @param angle radians
     */
    private void setArm1Angle(double angle) {
        resetArm1();
        rotateArm1(angle);
    }

    /**
     * Sets the angle of arm2 to the specified angle
     * @param angle radians
     */
    private void setArm2Angle(double angle) {
        resetArm2();
        rotateArm2(angle);
    }

    /**
     * Sets the angle of the turret to the specified angle
     * @param angle radians
     */
    private void setTurretAngle(double angle) {
        resetTurret();
        rotateTurret(angle);
    }

    private void setRobotNodeAngle(double angle) {
        robotNode.setLocalRotation(new Quaternion());
        robotNode.rotate(0, (float) angle, 0);
    }

    /**
     * This function is run once on app startup. All initialization goes in here.
     */
    @Override
    public void simpleInitApp() {
        cam.setLocation(new Vector3f(0, -20, 0));
        cam.setAxes(new Vector3f(0, 0, -1), new Vector3f(0, -1, 0), new Vector3f(1, 0, 0));
        // flyCam.setRotationSpeed(-flyCam.getRotationSpeed());
        flyCam.setUpVector(new Vector3f(0, -1, 0));
        // a quaternion facing upwards
        cam.setRotation(new Quaternion(1, 0, 0, 0));
        cam.update(); // Force update the camera so the previous command takes effect immediately
        flyCam.setMoveSpeed(100); // Lets you fly faster, since the geometry is quite large

        inputManager.addMapping(CameraInput.FLYCAM_RISE, new KeyTrigger(KeyInput.KEY_SPACE)); // "minecraft control system"
        inputManager.addMapping(CameraInput.FLYCAM_LOWER, new KeyTrigger(KeyInput.KEY_LCONTROL));

        Spatial field = assetManager.loadModel("Models/Field/field.gltf.j3o");
        field.rotate(FastMath.HALF_PI, FastMath.PI, 0); // long side is x, short is z (correct).
        rootNode.rotate(FastMath.PI, 0, 0); // lol
        // 888.57 in by 427.85 in
        // final float fieldXModel = 888.57f;
        final float fieldYModel = 427.85f;

        final float yo10 = fieldYModel / 10f;
        final float fieldScale = yo10 * 0.920f; // determined experimentially lel
        field.scale(fieldScale);
        field.center();
        field.move((float) Constants.FieldConstants.FIELD_X_LENGTH / 2f + 0.f, fieldScale, -((float) Constants.FieldConstants.FIELD_Y_LENGTH / 2f - 20.69f));
        Material mat_default = new Material( assetManager, "Common/MatDefs/Misc/ShowNormals.j3md");
        field.setMaterial(mat_default);

        poseMat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        poseMat.setColor("Color", ColorRGBA.Black);

        Box floor = new Box(1000, 0.0001f, 1000); // The floor
        Geometry floorGeom = new Geometry("Floor", floor);

        Box robot = new Box(0.5f * 35, 4, 0.5f * 30); // The robot
        Geometry robotGeom = new Geometry("Box", robot);

        Box base = new Box(1, 1, 1); // This is the base of the arm
        Geometry baseGeom = new Geometry("Box", base);

        Box baseTower = new Box(1, 0.5f * (float) Constants.ArmConstants.ORIGIN_HEIGHT, 1); // The supporting beam of the arm
        Geometry baseTowerGeom = new Geometry("Box", baseTower);

        // These nodes allow me to rotate things independently of each other around a known point, and to find the world-relative coordinates of parts.
        robotNode = new Node("robotNode");
        baseNode = new Node("baseNode");
        positionsNode = new Node("posesNode");

        arm1 = new Node("arm1");
        arm2 = new Node("arm2");
        Node claw = new Node("claw");

        Box limb1 = new Box(1, 0.5f * (float) Constants.ArmConstants.LIMB1_LENGTH, 1); // Arm limb 1
        Geometry limb1Geom = new Geometry("Box", limb1);

        Box limb2 = new Box(1, 0.5f * (float) Constants.ArmConstants.LIMB2_LENGTH, 1); // Arm limb 2
        Geometry limb2Geom = new Geometry("Box", limb2);

        Box clawB = new Box(1, 1, 1); // The claw (a box)
        clawGeom = new Geometry("Box", clawB);

        Box targetPoint = new Box(1f, 1f, 1f); // The target point
        targetPointGeom = new Geometry("Box", targetPoint);

        Box camera = new Box(1f, 1f, 1f);
        cameraGeom = new Geometry("Box", camera);

        cameraGeom.move(0, 17, -7.5f);

        // This attaches items into the node system (consider it a tree). The items are linked specifically so I can rotate parts like the arm limbs at the same time.
        rootNode.attachChild(floorGeom);
        robotNode.attachChild(targetPointGeom);
        rootNode.attachChild(robotNode);
        rootNode.attachChild(positionsNode);
        robotNode.attachChild(baseNode);
        robotNode.attachChild(robotGeom);
        baseNode.attachChild(baseGeom);
        baseNode.attachChild(baseTowerGeom);
        baseNode.attachChild(arm1);
        rootNode.attachChild(field);
        baseNode.attachChild(cameraGeom);

        arm1.attachChild(arm2); // Attach arm2 node to arm1, so that when arm1 rotates, arm2 moves with it
        arm1.attachChild(limb1Geom);

        arm2.attachChild(limb2Geom);
        arm2.attachChild(claw);

        claw.attachChild(clawGeom);

        robotNode.center(); // Make sure the base of the arm is at the origin
        robotNode.move(0, 4, 0); // and now make it correct height

        // This moves the items into the correct position relative to each other
        baseTowerGeom.move(0, ((float) Constants.ArmConstants.ORIGIN_HEIGHT) / 2, 0);
        arm1.move(0, ((float) Constants.ArmConstants.ORIGIN_HEIGHT), 0);
        arm2.move(0, ((float) Constants.ArmConstants.LIMB1_LENGTH), 0);
        claw.move(0, ((float) Constants.ArmConstants.LIMB2_LENGTH), 0);
        limb1Geom.move(0, ((float) Constants.ArmConstants.LIMB1_LENGTH) / 2, 0);
        limb2Geom.move(0, ((float) Constants.ArmConstants.LIMB2_LENGTH) / 2, 0);

        // Set the rotation of the arm limbs to their initial positions. This code works
        arm1.rotate(0, 0, (float) Math.toRadians(180f + Constants.ArmConstants.ARM_1_INITIAL_ANGLE));
        arm2.rotate(0, 0, (float) Math.toRadians(180f - Constants.ArmConstants.ARM_2_INITIAL_ANGLE));

        // Colors the limbs in different colors
        Material mat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        Material mat2 = mat.clone(), mat3 = mat.clone(), mat4 = mat.clone(), mat5 = mat.clone(), mat6 = mat.clone(), mat7 = mat.clone(), mat8 = mat.clone(); // I was too lazy to keep initializing new materials
        mat.setColor("Color", ColorRGBA.Blue);
        mat2.setColor("Color", ColorRGBA.Cyan);
        mat3.setColor("Color", ColorRGBA.Green);
        mat4.setColor("Color", ColorRGBA.Yellow);
        mat5.setColor("Color", ColorRGBA.Orange);
        mat6.setColor("Color", ColorRGBA.Gray);
        mat7.setColor("Color", ColorRGBA.Magenta);
        mat8.setColor("Color", ColorRGBA.Red);

        DirectionalLight sun = new DirectionalLight(); // This is the light source for shadows
        sun.setDirection(new Vector3f(-0.5f, 0.5f, -0.5f).normalizeLocal()); // Positions the light source
        sun.setColor(ColorRGBA.White);
        rootNode.addLight(sun); // Puts the sun in the map
        rootNode.setShadowMode(RenderQueue.ShadowMode.Receive); // Make sure everything can have a shadow casted on, but cannot cast a shadow because that will kill fps
        baseNode.setShadowMode(RenderQueue.ShadowMode.CastAndReceive);
        positionsNode.setShadowMode(RenderQueue.ShadowMode.Off);

        final int SHADOWMAP_SIZE = 128; // Size of the shadow map
        DirectionalLightShadowRenderer dlsr = new DirectionalLightShadowRenderer(assetManager, SHADOWMAP_SIZE, 1); // Renders shadows
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
        robotGeom.setMaterial(mat8);
        cameraGeom.setMaterial(mat2);

        // Adds keybinds
        initKeys();

        // Disables the built-in debug text
        setDisplayFps(true);
        setDisplayStatView(false);

        // Adds the IKU, FKU, and real value text
        hudText = new BitmapText(guiFont);
        hudText.setSize(guiFont.getCharSet().getRenderedSize());
        hudText.setColor(ColorRGBA.White);
        hudText.setText("TEST");
        hudText.setLocalTranslation(10, hudText.getLineHeight() * 15, 0);
        guiNode.attachChild(hudText);

        hudText.setQueueBucket(RenderQueue.Bucket.Gui); // Tells JME that this is text

        for(int i = 0; i < Constants.FieldConstants.GamePiecePlacementLocationConstants.POLE_POSITIONS.length; i++) {
            for(int j = 0; j < Constants.FieldConstants.GamePiecePlacementLocationConstants.POLE_POSITIONS[i].length; j++) {
                var pose = Constants.FieldConstants.GamePiecePlacementLocationConstants.POLE_POSITIONS[i][j];
                renderPose3d(pose);
                renderPose3d(MathUtil.mirrorPoseOnFieldForOppositeSide(pose));
            }
        }
        for(int i = 0; i < Constants.FieldConstants.GamePiecePlacementLocationConstants.PLATFORM_POSITIONS.length; i++) {
            for(int j = 0; j < Constants.FieldConstants.GamePiecePlacementLocationConstants.PLATFORM_POSITIONS[i].length; j++) {
                var pose = Constants.FieldConstants.GamePiecePlacementLocationConstants.PLATFORM_POSITIONS[i][j];
                renderPose3d(pose);
                renderPose3d(MathUtil.mirrorPoseOnFieldForOppositeSide(pose));
            }
        }
        for(int i = 0; i < Constants.FieldConstants.AprilTagConstants.APRILTAG_LOCATIONS.length; i++) {
            var pose = Constants.FieldConstants.AprilTagConstants.APRILTAG_LOCATIONS[i];
            // System.out.println("Placed tag at " + Arrays.toString(pose));
            renderTagLocation(pose);
        }
    }

    /**
     * This method initializes the keybinds.
     */
    private void initKeys() {
        inputManager.addMapping("X+", new KeyTrigger(KeyInput.KEY_L));
        inputManager.addMapping("X-", new KeyTrigger(KeyInput.KEY_J));
        inputManager.addMapping("Y+", new KeyTrigger(KeyInput.KEY_COMMA));
        inputManager.addMapping("Y-", new KeyTrigger(KeyInput.KEY_PERIOD));
        inputManager.addMapping("Turret+", new KeyTrigger(KeyInput.KEY_U));
        inputManager.addMapping("Turret-", new KeyTrigger(KeyInput.KEY_O));

        inputManager.addMapping("Flip", new KeyTrigger(KeyInput.KEY_F));

        inputManager.addMapping("Robot Forward", new KeyTrigger(KeyInput.KEY_NUMPAD8));
        inputManager.addMapping("Robot Backward", new KeyTrigger(KeyInput.KEY_NUMPAD2));
        inputManager.addMapping("Robot CCW", new KeyTrigger(KeyInput.KEY_NUMPAD4));
        inputManager.addMapping("Robot CW", new KeyTrigger(KeyInput.KEY_NUMPAD6));

        inputManager.addListener(actionListener, "Flip");
        inputManager.addListener(analogListener, "X+", "X-", "Y+", "Y-", "Turret+", "Turret-", "Robot Forward", "Robot Backward", "Robot CCW", "Robot CW");
    }

    private Pose2d getRobotPose() {
        return new Pose2d(robotNode.getWorldTranslation().x, robotNode.getWorldTranslation().z, new Rotation2d(robotNode.getWorldRotation().toAngles(null)[1]));
    }

    /**
     * This method updates the HUD text. The text has a few modifications made to it to correct obvious errors caused by our 3D model (such as angles being negative).
     */
    private void updateHUDText() {
        double[] angles = getAnglesDeg();

        boolean simIsFlipped = angles[1] > 180;
        if (angles[0] > 180) {
            angles[0] = 360 - angles[0];
        }

        double[] fkuValues = ForwardKinematicsUtil.getCoordinatesFromAngles(angles[0], angles[1], angles[2]);

        Vector3f robotWorld = robotNode.getWorldTranslation();
        Vector3f clawWorldLoc = clawGeom.getWorldTranslation();
        Vector3f clawLocationRelative = clawWorldLoc.subtract(robotWorld), targetLocation = targetPointGeom.getWorldTranslation();

        double[] robotRelativeClawReal = MathUtil.rotatePoint(clawLocationRelative.x, clawLocationRelative.z, -robotNode.getLocalRotation().toAngles(null)[1] * FastMath.RAD_TO_DEG);

        double[] ikuValues = InverseKinematicsUtil.getAnglesFromCoordinates(robotRelativeClawReal[0], -clawLocationRelative.y, robotRelativeClawReal[1], this.isFlipped);

        Pose2d fieldPoseClaw = MathUtil.findFieldRelativePose(getRobotPose(), new Pose2d(clawLocationRelative.x, clawLocationRelative.z, new Rotation2d()));


        // Updates the text
        // Also, turns out rendering less text makes your game a bit faster who woulda known (rounding)
        hudText.setText("VALUES:\nIKU (DEG): " + String.format("%.2f, %.2f, %.2f, flipped: %b", ikuValues[0], ikuValues[1], ikuValues[2], simIsFlipped)
                + "\nFKU (POS): " + String.format("%.2f, %.2f, %.2f", fkuValues[0], fkuValues[1], fkuValues[2])
                + "\n\nREAL:\nANGLES (DEG): " + String.format("%.2f, %.2f, %.2f, flipped: %b", angles[0], angles[1], angles[2], this.isFlipped)
                + "\nPOSITION: " + String.format("%.2f, %.2f, %.2f", clawWorldLoc.x, -clawWorldLoc.y, clawWorldLoc.z)
                // + "\nCLAW RELATIVE: " + clawLocationRelative.x + ", " + clawLocationRelative.y + ", " + clawLocationRelative.z
                + "\nCLAW RELATIVE ROBOT ROTATED: " + String.format("%.2f, %.2f, %.2f", robotRelativeClawReal[0], -clawLocationRelative.y, robotRelativeClawReal[1])
                + "\nTARGET: " + String.format("%.2f, %.2f, %.2f", targetLocation.x, -targetLocation.y, targetLocation.z)
                + "\nCAMERA COORDS: " + String.format("%.2f, %.2f, %.2f", cameraGeom.getWorldTranslation().x, cameraGeom.getWorldTranslation().y, cameraGeom.getWorldTranslation().z)
                + "\nRobot pose: " + getRobotPose()
                + "\nFieldPoseClaw: " + fieldPoseClaw
        );
    }

    /**
     * Returns the known angles of the arm components.
     *
     * @return [arm1AngleDeg, arm2AngleDeg, turretAngleDeg]
     */
    private double[] getAnglesDeg() {
        var tempTurretAngDeg = (float) turretAngleRad;

        double arm1AngleDeg = Math.abs(180 - (Math.abs(arm1AngleRad * FastMath.RAD_TO_DEG) % 360));
        double arm2AngleDeg = 180 - (Math.abs(arm2AngleRad * FastMath.RAD_TO_DEG) % 360);
        double turretAngleDeg = (tempTurretAngDeg * FastMath.RAD_TO_DEG) % 360;

        return new double[]{arm1AngleDeg, arm2AngleDeg, turretAngleDeg};
    }

    /**
     * This method updates the angles of the arm components using IKU to position the claw at the target position.
     */
    private void ikuUpdateAngles() {
        double[] ikuValues = InverseKinematicsUtil.getAnglesFromCoordinates(targetX, targetY, targetZ, this.isFlipped);

        setArm1Angle((180f + ikuValues[0]) * FastMath.DEG_TO_RAD);
        setArm2Angle((180f - ikuValues[1]) * FastMath.DEG_TO_RAD);
        setTurretAngle(-ikuValues[2] * FastMath.DEG_TO_RAD);
    }

    /**
     * Updates the target position and the arm angles to match.
     */
    private void updateTarget(boolean ikuUpdate) {
        targetPointGeom.center().move((float) targetX, (float) targetY, (float) targetZ);
        if(ikuUpdate) {
            ikuUpdateAngles();
        }
    }

    /**
     * Renders a Pose3d to the screen
     * @param pose The pose
     */
    private void renderPose3d(Pose3d pose) {
        Box box = new Box(0.5f, 0.5f, 0.5f);
        Geometry geom = new Geometry("Box", box);

        geom.setMaterial(poseMat);

        positionsNode.attachChild(geom);
        geom.setLocalTranslation((float) pose.getX(), (float) pose.getZ(), -(float) pose.getY());

    }

    private void renderTagLocation(double[] tag) {
        Box box = new Box(0.5f, 0.5f, 0.5f);
        Geometry geom = new Geometry("Box", box);

        geom.setMaterial(poseMat);

        positionsNode.attachChild(geom);
        geom.setLocalTranslation((float) tag[0], (float) tag[2], -(float) tag[1]);
    }

    @Override
    public void simpleUpdate(float tpf) {
        //this method will be called every game tick and can be used to make updates
        Pose2d pos = NetworkTablesWrapper.getJetsonPose();
        // System.out.println("jetson pose: " + pos);
        if(!(pos.getX() == 0 && pos.getY() == 0)) {
            // this.robotNode.setLocalTranslation(new Vector3f((float) pos.getX(), this.robotNode.getLocalTranslation().y, (float) pos.getY()));
            // setRobotNodeAngle(pos.getRotation().getRadians());
        }
        double[] armAngles = NetworkTablesWrapper.getArmAngles();
        if(NetworkTablesWrapper.getAtHumanPlayer()) {
            armAngles[2] += 180f;
        }
        if(!(armAngles[0] == 0 && armAngles[1] == 0 && armAngles[2] == 0)) {
            setArm1Angle((180f + armAngles[0]) * FastMath.DEG_TO_RAD);
            setArm2Angle((180f - armAngles[1]) * FastMath.DEG_TO_RAD);
            setTurretAngle(armAngles[2] * FastMath.DEG_TO_RAD);
        }
        double[] target = NetworkTablesWrapper.getArmTargetCoordinates();

        //System.out.println("arm angles: " + Arrays.toString(armAngles));
        //System.out.println("target: " + Arrays.toString(target));

        if(!(target[0] == 0 && target[1] == 0 && target[2] == 0)) {
            targetY = target[1];
            if(NetworkTablesWrapper.get2dMode()) {
                double[] rotatedTargets = MathUtil.rotatePoint(target[0], target[2], -armAngles[2]);
                targetX = rotatedTargets[0];
                targetZ = rotatedTargets[1];
            } else {
                targetX = target[0];
                targetZ = target[2];
            }
        }
        updateTarget(false);
        updateHUDText();
    }

    @Override
    public void simpleRender(RenderManager rm) {
        //add render code here (if any)
    }
}
