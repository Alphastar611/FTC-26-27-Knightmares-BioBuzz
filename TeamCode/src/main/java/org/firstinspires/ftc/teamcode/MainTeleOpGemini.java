package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

@TeleOp(name = "MainTeleOp")
public class MainTeleOpGemini extends LinearOpMode {

    // =========================================================================
    // 1. TUNING CONSTANTS (Edit these to tune driving, turret, and shooter)
    // =========================================================================

    // Drive & Starting Pose
    static final double START_X = -59.0;
    static final double START_Y = 70.0;
    static final double START_HEADING = -90.0;
    static final double DRIVER_HEADING_OFFSET = -90.0;
    static final double STICK_DEADBAND = 0.05;
    static final boolean BRAKE_MODE = false;

    // Shooter Formula & Trims
    static final double SHOOTER_SLOPE = 3.64541;
    static final double SHOOTER_BASE_VEL = 1260.0;
    static final double POS_SIDE_VEL_TRIM = -80.0;     // Applied when Robot_X > 0
    static final double MIN_SHOOTER_VEL = 0.0;
    static final double MAX_SHOOTER_VEL = 3000.0;
    static final int VELOCITY_STEP = 20;

    // Shoot-On-The-Move
    static final double SHOT_TIME = 0.65;
    static final double MAX_VEL_CORRECTION = 300.0;

    // Turret & Goal Geometry
    static final double BASE_GOAL_X_NEG = -10.0;
    static final double BASE_GOAL_X_POS = 10.0;
    static final double BASE_GOAL_Y = 18.0;
    static final double GOAL_SWITCH_BAND = 3.0;
    static final double MAX_SIDE_SHIFT = 6.0;
    static final double MAX_X_SIDE_SHIFT = 4.0;
    static final double FIELD_HALF_WIDTH = 72.0;

    static final double TURRET_OFFSET_FORWARD = -2.0;
    static final double SERVO_MAX_RANGE = 355.0;
    static final double TURRET_KF = 0.275;
    static final double POS_SIDE_HEADING_TRIM = -3.0;  // Applied when Robot_X > 0
    static final int HEADING_STEP = 5;

    // Intake & Transfer
    static final double INTAKE_VEL = 2400.0;
    static final double TRANSFER_FEED_VEL = 2800.0;
    static final double TRANSFER_IDLE_POWER = -0.15;

    // Pinpoint Reliability
    static final double MAX_PLAUSIBLE_SPEED = 120.0;
    static final double MAX_HEADING_VEL = 1500.0;
    static final double MAX_POSE_JUMP = 12.0;
    static final double MAX_HEADING_JUMP = 45.0;
    static final double ABS_MAX_POSE_JUMP = 100.0;
    static final int MAX_CONSECUTIVE_REJECTS = 5;
    static final double VEL_FILTER_ALPHA = 0.5;
    static final double RESET_DELAY_MS = 500.0;
    static final double AIM_TIMEOUT_MS = 250.0;

    // =========================================================================
    // 2. HARDWARE & STATE VARIABLES
    // =========================================================================

    private DcMotor FL, BL, FR, BR, Shooter, Intake, Transfer;
    private Servo Turret, FlowerMechanism;
    private GoBildaPinpointDriver Pinpoint;

    // Driver Inputs & Toggles
    private double stickY, stickX, stickRX;
    private boolean turretOn = false;
    private boolean flowerOn = false;
    private boolean shooterOn = false;
    private boolean fieldCentricOn = false;

    // Localization State
    private double robotX = START_X, robotY = START_Y, robotYaw = START_HEADING;
    private double velX = 0, velY = 0, headingVel = 0;
    private boolean pinpointOK = false, hasGoodPose = false;
    private boolean poseSetManually = false, acceptNextPose = true, resetPending = false;
    private GoBildaPinpointDriver.DeviceStatus pinpointStatus;
    private int rejectedFrames = 0, consecutiveRejects = 0;

    // Aiming & Shooter State
    private double goalX = BASE_GOAL_X_NEG, goalY = BASE_GOAL_Y;
    private double distance = 0, targetVel = 0, velCorrection = 0, targetTicks = 0.5;
    private boolean turretInRange = true, wasTurretOutOfRange = false;
    private int headingOffset = 0, velocityOffset = 0;

    // Timers & Diagnostics
    private final ElapsedTime resetTimer = new ElapsedTime();
    private final ElapsedTime goodFrameTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime refreshTimer = new ElapsedTime();
    private double loopMs = 0, maxLoopMs = 0;
    private int stallCount = 0;
    private boolean firstLoop = true, forceWrite = false;

    // Hardware Write Cache
    private final double[] lastDrive = {Double.NaN, Double.NaN, Double.NaN, Double.NaN};
    private double lastIntakeVel = Double.NaN, lastTransferCmd = Double.NaN;
    private double lastShooterVel = Double.NaN, lastShooterPower = Double.NaN;
    private double lastTurretPos = Double.NaN, lastFlowerPos = Double.NaN;
    private boolean lastTransferWasVel = false;

    // =========================================================================
    // 3. MAIN OPCODE LOOP
    // =========================================================================

    @Override
    public void runOpMode() {
        initHardware();

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Status", "Ready");
            telemetry.addData("Start Pose", "(%.1f, %.1f) @ %.1f°", robotX, robotY, robotYaw);
            telemetry.update();
            sleep(50);
        }
        if (isStopRequested()) return;

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        loopTimer.reset();
        refreshTimer.reset();

        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) hub.clearBulkCache();

            updateDiagnostics();
            readInputs();
            updateGoalAndToggles();
            computeAiming();
            actuate();

            if (forceWrite) updateTelemetry();
        }
    }

    private void initHardware() {
        Shooter = hardwareMap.get(DcMotor.class, "Shooter");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Transfer = hardwareMap.get(DcMotor.class, "Transfer");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");
        Turret = hardwareMap.get(Servo.class, "Turret");
        FlowerMechanism = hardwareMap.get(Servo.class, "Flower Mechanism");
        Pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Pinpoint Setup
        Pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        Pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        Pinpoint.setOffsets(0, -2.5, DistanceUnit.INCH);
        Pinpoint.resetPosAndIMU();
        resetTimer.reset();
        resetPending = true;

        // Motor Setup
        setupVelocityMotor(Shooter, DcMotor.Direction.REVERSE, 50, 14);
        setupVelocityMotor(Intake, DcMotor.Direction.REVERSE, 30, 16);
        setupVelocityMotor(Transfer, DcMotor.Direction.FORWARD, 30, 14);

        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.FORWARD);

        DcMotor.ZeroPowerBehavior driveZero = BRAKE_MODE
                ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
        FL.setZeroPowerBehavior(driveZero);
        BL.setZeroPowerBehavior(driveZero);
        FR.setZeroPowerBehavior(driveZero);
        BR.setZeroPowerBehavior(driveZero);

        // Servo Setup
        ((ServoImplEx) Turret).setPwmRange(new PwmControl.PwmRange(500, 2500));
        Turret.setDirection(Servo.Direction.REVERSE);
        FlowerMechanism.setDirection(Servo.Direction.FORWARD);

        telemetry.setMsTransmissionInterval(100);
        goodFrameTimer.reset();
    }

    private void setupVelocityMotor(DcMotor motor, DcMotor.Direction dir, double p, double f) {
        motor.setDirection(dir);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ((DcMotorEx) motor).setVelocityPIDFCoefficients(p, 0, 0, f);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // =========================================================================
    // 4. INPUTS, LOCALIZATION & TOGGLES
    // =========================================================================

    private void readInputs() {
        // Manual Pose Reset (gamepad1 Back)
        if (gamepad1.backWasPressed()) {
            boolean stationary = Math.abs(velX) <= 1.0 && Math.abs(velY) <= 1.0 && Math.abs(headingVel) <= 5.0;
            if (stationary) {
                Pinpoint.resetPosAndIMU();
                resetTimer.reset();
                resetPending = true;
                acceptNextPose = true;
                hasGoodPose = false;
                poseSetManually = true;
                consecutiveRejects = 0;
            } else {
                gamepad1.rumble(150);
            }
        }

        if (resetPending && resetTimer.milliseconds() >= RESET_DELAY_MS) {
            Pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, START_X, START_Y, AngleUnit.DEGREES, START_HEADING));
            resetPending = false;
            acceptNextPose = true;
            hasGoodPose = false;
            velX = velY = headingVel = 0;
        }

        stickY = deadband(gamepad1.left_stick_y);
        stickX = deadband(gamepad1.left_stick_x);
        stickRX = deadband(gamepad1.right_stick_x);

        pinpointOK = false;
        try {
            Pinpoint.update();
            pinpointStatus = Pinpoint.getDeviceStatus();
            if (pinpointStatus == GoBildaPinpointDriver.DeviceStatus.READY) {
                processPinpointFrame(
                        Pinpoint.getPosX(DistanceUnit.INCH),
                        Pinpoint.getPosY(DistanceUnit.INCH),
                        Pinpoint.getHeading(AngleUnit.DEGREES),
                        Pinpoint.getVelX(DistanceUnit.INCH),
                        Pinpoint.getVelY(DistanceUnit.INCH),
                        Pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
            }
        } catch (Exception ignored) {
        }
    }

    private void processPinpointFrame(double px, double py, double yaw, double vx, double vy, double hv) {
        if (!Double.isFinite(px) || !Double.isFinite(py) || !Double.isFinite(yaw)
                || !Double.isFinite(vx) || !Double.isFinite(vy) || !Double.isFinite(hv)) {
            rejectedFrames++;
            return;
        }

        if (hasGoodPose && !acceptNextPose) {
            double jump = Math.hypot(px - robotX, py - robotY);
            double headingJump = Math.abs(AngleUnit.DEGREES.normalize(yaw - robotYaw));

            if (jump > ABS_MAX_POSE_JUMP) {
                rejectedFrames++;
                consecutiveRejects++;
                return;
            }
            if ((jump > MAX_POSE_JUMP || headingJump > MAX_HEADING_JUMP)
                    && consecutiveRejects < MAX_CONSECUTIVE_REJECTS) {
                consecutiveRejects++;
                rejectedFrames++;
                return;
            }
        }

        consecutiveRejects = 0;
        acceptNextPose = false;
        hasGoodPose = true;
        robotX = px;
        robotY = py;
        robotYaw = yaw;

        if (Math.abs(vx) <= MAX_PLAUSIBLE_SPEED && Math.abs(vy) <= MAX_PLAUSIBLE_SPEED) {
            velX = VEL_FILTER_ALPHA * vx + (1 - VEL_FILTER_ALPHA) * velX;
            velY = VEL_FILTER_ALPHA * vy + (1 - VEL_FILTER_ALPHA) * velY;
        }
        if (Math.abs(hv) <= MAX_HEADING_VEL) {
            headingVel = VEL_FILTER_ALPHA * hv + (1 - VEL_FILTER_ALPHA) * headingVel;
        }

        pinpointOK = true;
        goodFrameTimer.reset();
    }

    private void updateGoalAndToggles() {
        // Select Goal X side
        double baseGoalX = (robotX > GOAL_SWITCH_BAND) ? BASE_GOAL_X_POS
                : (robotX < -GOAL_SWITCH_BAND) ? BASE_GOAL_X_NEG
                : (goalX >= 0 ? BASE_GOAL_X_POS : BASE_GOAL_X_NEG);

        // Dynamic Goal Y & X side-shift
        double yOffset = robotY - BASE_GOAL_Y;
        double maxWallDist = (yOffset >= 0) ? (FIELD_HALF_WIDTH - BASE_GOAL_Y) : (FIELD_HALF_WIDTH + BASE_GOAL_Y);
        double sideRatio = clamp(yOffset / maxWallDist, -1.0, 1.0);
        double sideShift = sideRatio * MAX_SIDE_SHIFT;
        double depthShift = Math.abs(sideRatio) * MAX_X_SIDE_SHIFT;

        goalY = (baseGoalX < 0) ? (BASE_GOAL_Y - sideShift) : (BASE_GOAL_Y + sideShift);
        goalX = (baseGoalX < 0) ? (baseGoalX + depthShift) : (baseGoalX - depthShift);

        // Driver 1 Toggles
        if (gamepad1.aWasPressed()) turretOn = !turretOn;
        if (gamepad1.bWasPressed()) shooterOn = !shooterOn;
        if (gamepad1.xWasPressed()) fieldCentricOn = !fieldCentricOn;
        if (gamepad1.leftBumperWasPressed()) flowerOn = !flowerOn;

        // Driver 2 Live Trims
        if (gamepad2.dpadRightWasPressed()) headingOffset -= HEADING_STEP;
        if (gamepad2.dpadLeftWasPressed()) headingOffset += HEADING_STEP;
        if (gamepad2.dpadDownWasPressed()) velocityOffset -= VELOCITY_STEP;
        if (gamepad2.dpadUpWasPressed()) velocityOffset += VELOCITY_STEP;
    }

    // =========================================================================
    // 5. AIMING & ACTUATION
    // =========================================================================

    private void computeAiming() {
        if (!pinpointOK) {
            if (goodFrameTimer.milliseconds() >= AIM_TIMEOUT_MS) {
                targetTicks = 0.5;
                velCorrection = 0;
            }
            return;
        }

        double yawRad = Math.toRadians(robotYaw);
        double turretX = robotX + TURRET_OFFSET_FORWARD * Math.cos(yawRad);
        double turretY = robotY + TURRET_OFFSET_FORWARD * Math.sin(yawRad);

        // Real & Virtual (shoot-on-the-move) distances
        distance = Math.hypot(goalX - turretX, goalY - turretY);
        double virtualDeltaX = (goalX - velX * SHOT_TIME) - turretX;
        double virtualDeltaY = (goalY - velY * SHOT_TIME) - turretY;
        double virtualDist = Math.hypot(virtualDeltaX, virtualDeltaY);

        // Shooter Velocity
        double sideVelTrim = (robotX > 0) ? POS_SIDE_VEL_TRIM : 0;
        targetVel = (SHOOTER_SLOPE * distance) + SHOOTER_BASE_VEL + sideVelTrim;
        velCorrection = clamp((virtualDist - distance) / SHOT_TIME, -MAX_VEL_CORRECTION, MAX_VEL_CORRECTION);

        // Turret Angle
        double targetAngle = AngleUnit.DEGREES.normalize(
                Math.toDegrees(Math.atan2(virtualDeltaY, virtualDeltaX)) + 180);
        double leadAngle = clamp(TURRET_KF * headingVel, -15, 15);
        double rawTurretAngle = (((targetAngle - leadAngle) - robotYaw) % 360 + 360) % 360;

        turretInRange = rawTurretAngle <= SERVO_MAX_RANGE;
        double sideHeadingTrim = (robotX > 0) ? POS_SIDE_HEADING_TRIM : 0;

        double normalizedHeading = turretInRange
                ? (rawTurretAngle + headingOffset + sideHeadingTrim)
                : (rawTurretAngle >= (SERVO_MAX_RANGE + 360) / 2.0 ? 0 : SERVO_MAX_RANGE);

        targetTicks = clamp(normalizedHeading / SERVO_MAX_RANGE, 0.0, 1.0);
    }

    private void actuate() {
        // Drivetrain (Robot-Centric default; Field-Centric if toggled ON and pose set manually)
        double driveX = stickX;
        double driveY = stickY;

        if (fieldCentricOn && poseSetManually && hasGoodPose) {
            double fieldFwd = -stickY;
            double fieldLeft = -stickX;
            double rad = Math.toRadians(robotYaw - DRIVER_HEADING_OFFSET);
            double cos = Math.cos(-rad);
            double sin = Math.sin(-rad);

            driveY = -(fieldFwd * cos - fieldLeft * sin);
            driveX = -(fieldFwd * sin + fieldLeft * cos);
        }

        double fl = (driveY - driveX) - stickRX;
        double bl = (driveY + driveX) - stickRX;
        double fr = (driveY + driveX) + stickRX;
        double br = (driveY - driveX) + stickRX;
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(bl)), Math.max(Math.abs(fr), Math.abs(br))));

        setDrive(FL, 0, fl / max);
        setDrive(BL, 1, bl / max);
        setDrive(FR, 2, fr / max);
        setDrive(BR, 3, br / max);

        // Intake & Transfer
        if (gamepad1.right_trigger > 0.1) {
            setTransfer(TRANSFER_FEED_VEL, true);
            setIntakeVelocity(INTAKE_VEL);
        } else if (gamepad1.right_bumper) {
            setTransfer(TRANSFER_IDLE_POWER, false);
            setIntakeVelocity(-INTAKE_VEL);
        } else {
            setTransfer(TRANSFER_IDLE_POWER, false);
            setIntakeVelocity(gamepad1.left_trigger * INTAKE_VEL);
        }

        // Turret, Shooter & Flower
        setServo(Turret, true, turretOn ? targetTicks : 0.5);
        setServo(FlowerMechanism, false, flowerOn ? 0.425 : 0.0);

        if (shooterOn) {
            setShooterVelocity(targetVel + velCorrection + velocityOffset);
        } else {
            setShooterOff();
        }

        // Rumble if turret hits travel limit
        boolean outOfRange = turretOn && !turretInRange;
        if (outOfRange && !wasTurretOutOfRange) gamepad1.rumble(150);
        wasTurretOutOfRange = outOfRange;
    }

    // =========================================================================
    // 6. HARDWARE CACHING & DIAGNOSTICS HELPERS
    // =========================================================================

    private void updateDiagnostics() {
        loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        if (firstLoop) {
            firstLoop = false;
        } else {
            maxLoopMs = Math.max(maxLoopMs, loopMs);
            if (loopMs > 50) stallCount++;
        }

        if (gamepad2.backWasPressed()) {
            maxLoopMs = 0;
            stallCount = 0;
            rejectedFrames = 0;
            consecutiveRejects = 0;
        }

        forceWrite = refreshTimer.milliseconds() >= 200;
        if (forceWrite) refreshTimer.reset();
    }

    private void updateTelemetry() {
        double cmdShooterVel = clamp(targetVel + velCorrection + velocityOffset, MIN_SHOOTER_VEL, MAX_SHOOTER_VEL);
        telemetry.addData("Field Centric", "%s (Active: %s)", fieldCentricOn, fieldCentricOn && poseSetManually && hasGoodPose);
        telemetry.addData("Pose (X, Y, Yaw)", "%.1f, %.1f, %.1f°", robotX, robotY, robotYaw);
        telemetry.addData("Goal (X, Y) | Dist", "%.1f, %.1f | %.1f in", goalX, goalY, distance);
        telemetry.addData("Shooter Vel (Act / Cmd)", "%.0f / %.0f", ((DcMotorEx) Shooter).getVelocity(), cmdShooterVel);
        telemetry.addData("Trims (Vel / Head)", "%d / %d", velocityOffset, headingOffset);
        telemetry.addData("Pinpoint", "%s (OK: %s, Rejects: %d)", pinpointStatus, pinpointOK, rejectedFrames);
        telemetry.addData("Loop ms (Cur / Max)", "%.1f / %.1f (Stalls: %d)", loopMs, maxLoopMs, stallCount);
        telemetry.update();
    }

    private boolean needsWrite(double newVal, double lastVal, double eps) {
        if (forceWrite || Double.isNaN(lastVal)) return true;
        if (newVal == 0) return lastVal != 0;
        return Math.abs(newVal - lastVal) > eps;
    }

    private void setDrive(DcMotor motor, int idx, double power) {
        power = clamp(power, -1.0, 1.0);
        if (needsWrite(power, lastDrive[idx], 0.01)) {
            motor.setPower(power);
            lastDrive[idx] = power;
        }
    }

    private void setIntakeVelocity(double vel) {
        if (needsWrite(vel, lastIntakeVel, 5)) {
            ((DcMotorEx) Intake).setVelocity(vel);
            lastIntakeVel = vel;
        }
    }

    private void setTransfer(double val, boolean useVelocity) {
        DcMotor.RunMode targetMode = useVelocity ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        double eps = useVelocity ? 5.0 : 0.001;

        if (lastTransferWasVel != useVelocity || needsWrite(val, lastTransferCmd, eps)) {
            if (Transfer.getMode() != targetMode) Transfer.setMode(targetMode);
            if (useVelocity) {
                ((DcMotorEx) Transfer).setVelocity(val);
            } else {
                Transfer.setPower(clamp(val, -1.0, 1.0));
            }
            lastTransferCmd = val;
            lastTransferWasVel = useVelocity;
        }
    }

    private void setShooterVelocity(double vel) {
        vel = clamp(vel, MIN_SHOOTER_VEL, MAX_SHOOTER_VEL);
        if (needsWrite(vel, lastShooterVel, 2)) {
            ((DcMotorEx) Shooter).setVelocity(vel);
            lastShooterVel = vel;
            lastShooterPower = Double.NaN;
        }
    }

    private void setShooterOff() {
        if (needsWrite(0, lastShooterPower, 0.001)) {
            Shooter.setPower(0);
            lastShooterPower = 0;
            lastShooterVel = Double.NaN;
        }
    }

    private void setServo(Servo servo, boolean isTurret, double pos) {
        pos = clamp(pos, 0.0, 1.0);
        double last = isTurret ? lastTurretPos : lastFlowerPos;
        if (needsWrite(pos, last, 0.001)) {
            servo.setPosition(pos);
            if (isTurret) lastTurretPos = pos;
            else lastFlowerPos = pos;
        }
    }

    private double deadband(double val) {
        return Math.abs(val) < STICK_DEADBAND ? 0 : val;
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}