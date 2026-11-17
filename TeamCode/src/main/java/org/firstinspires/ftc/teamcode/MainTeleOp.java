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
public class MainTeleOp extends LinearOpMode {

    // =========================================================================
    // TUNING CONSTANTS
    // =========================================================================

    // Starting Pose & Drive
    static final double START_X = -59.0;
    static final double START_Y = 70.0;
    static final double START_HEADING = -90.0;
    static final double STICK_DEADBAND = 0.05;

    // Shooter Formula & Trims
    static final double SHOOTER_SLOPE = 3.64541;
    static final double SHOOTER_BASE_VEL = 1260.0;
    static final double POS_SIDE_VEL_TRIM = -80.0;    // Applied when robotX > 0
    static final double MAX_SHOOTER_VEL = 3000.0;
    static final double SHOT_TIME = 0.65;
    static final double MAX_VEL_CORRECTION = 300.0;

    // Goal & Turret Geometry
    static final double BASE_GOAL_X_NEG = -10.0;
    static final double BASE_GOAL_X_POS = 10.0;
    static final double BASE_GOAL_Y = 18.0;
    static final double MAX_SIDE_SHIFT = 6.0;
    static final double MAX_X_SIDE_SHIFT = 4.0;
    static final double FIELD_HALF_WIDTH = 72.0;

    static final double TURRET_OFFSET_FORWARD = -2.0;
    static final double SERVO_MAX_RANGE = 355.0;
    static final double TURRET_KF = 0.275;
    static final double POS_SIDE_HEADING_TRIM = -3.0; // Applied when robotX > 0
    static final double VEL_SMOOTHING = 0.5;          // Smooths turret & shooter jitter

    // Intake & Transfer
    static final double INTAKE_VEL = 2400.0;
    static final double TRANSFER_FEED_VEL = 2800.0;
    static final double TRANSFER_IDLE_POWER = -0.15;

    // =========================================================================
    // HARDWARE & STATE
    // =========================================================================

    private DcMotor FL, BL, FR, BR;
    private DcMotorEx Shooter, Intake, Transfer;
    private ServoImplEx Turret;
    private Servo FlowerMechanism;
    private GoBildaPinpointDriver Pinpoint;

    private boolean turretOn = false, flowerOn = false, shooterOn = false;
    private boolean pinpointOK = false, resetPending = false, wasOutOfRange = false;

    private double robotX = START_X, robotY = START_Y, robotYaw = START_HEADING;
    private double velX = 0, velY = 0, headingVel = 0;
    private double goalX = BASE_GOAL_X_NEG, goalY = BASE_GOAL_Y;
    private double distance = 0, targetVel = 0, velCorrection = 0, targetTicks = 0.5;
    private int headingOffset = 0, velocityOffset = 0;

    // Hardware write cache (prevents bus spam & servo hunting current)
    private final double[] lastDrive = {Double.NaN, Double.NaN, Double.NaN, Double.NaN};
    private double lastIntakeVel = Double.NaN, lastShooterVel = Double.NaN, lastTurretPos = Double.NaN;
    private Boolean wasFeeding = null;

    private final ElapsedTime resetTimer = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double loopMs = 0;

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

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        telemetryTimer.reset();
        loopTimer.reset();

        while (opModeIsActive()) {
            loopMs = loopTimer.milliseconds();
            loopTimer.reset();

            for (LynxModule hub : hubs) hub.clearBulkCache();

            updateLocalization();
            updateTogglesAndGoal();
            computeAiming();
            actuate();

            if (telemetryTimer.milliseconds() >= 250) {
                updateTelemetry();
                telemetryTimer.reset();
            }
        }
    }

    private void initHardware() {
        Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        Transfer = hardwareMap.get(DcMotorEx.class, "Transfer");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");
        Turret = hardwareMap.get(ServoImplEx.class, "Turret");
        FlowerMechanism = hardwareMap.get(Servo.class, "Flower Mechanism");
        Pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Pinpoint
        Pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        Pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        Pinpoint.setOffsets(0, -2.5, DistanceUnit.INCH);
        Pinpoint.resetPosAndIMU();
        resetTimer.reset();
        resetPending = true;

        // Motors (FLOAT mode saves battery by coasting instead of braking with current)
        setupMotor(Shooter, DcMotor.Direction.REVERSE, 50, 14);
        setupMotor(Intake, DcMotor.Direction.REVERSE, 30, 16);
        setupMotor(Transfer, DcMotor.Direction.FORWARD, 30, 14);

        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.FORWARD);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Servos
        Turret.setPwmRange(new PwmControl.PwmRange(500, 2500));
        Turret.setDirection(Servo.Direction.REVERSE);
        Turret.setPosition(0.5);
        FlowerMechanism.setDirection(Servo.Direction.FORWARD);
        FlowerMechanism.setPosition(0.0);
    }

    private void setupMotor(DcMotorEx motor, DcMotor.Direction dir, double p, double f) {
        motor.setDirection(dir);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setVelocityPIDFCoefficients(p, 0, 0, f);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void updateLocalization() {
        if (gamepad1.backWasPressed()) {
            if (Math.abs(velX) <= 1.0 && Math.abs(velY) <= 1.0 && Math.abs(headingVel) <= 5.0) {
                Pinpoint.resetPosAndIMU();
                resetTimer.reset();
                resetPending = true;
            } else {
                gamepad1.rumble(150);
            }
        }

        if (resetPending && resetTimer.milliseconds() >= 500) {
            Pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, START_X, START_Y, AngleUnit.DEGREES, START_HEADING));
            resetPending = false;
            velX = velY = headingVel = 0;
        }

        Pinpoint.update();
        pinpointOK = Pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
        if (pinpointOK) {
            double px = Pinpoint.getPosX(DistanceUnit.INCH);
            double py = Pinpoint.getPosY(DistanceUnit.INCH);
            double yaw = Pinpoint.getHeading(AngleUnit.DEGREES);
            double vx = Pinpoint.getVelX(DistanceUnit.INCH);
            double vy = Pinpoint.getVelY(DistanceUnit.INCH);
            double hv = Pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);

            if (Double.isFinite(px) && Double.isFinite(py) && Double.isFinite(yaw)) {
                robotX = px;
                robotY = py;
                robotYaw = yaw;
                if (Double.isFinite(vx) && Double.isFinite(vy)) {
                    velX += VEL_SMOOTHING * (vx - velX);
                    velY += VEL_SMOOTHING * (vy - velY);
                }
                if (Double.isFinite(hv)) {
                    headingVel += VEL_SMOOTHING * (hv - headingVel);
                }
            }
        }
    }

    private void updateTogglesAndGoal() {
        // Driver 1 Toggles (write immediately on state change)
        if (gamepad1.aWasPressed()) {
            turretOn = !turretOn;
            if (!turretOn) {
                Turret.setPosition(0.5);
                lastTurretPos = 0.5;
            }
        }

        if (gamepad1.bWasPressed()) {
            shooterOn = !shooterOn;
            if (!shooterOn) {
                Shooter.setPower(0); // Coasts down with zero current draw
                lastShooterVel = Double.NaN;
            }
        }

        if (gamepad1.leftBumperWasPressed()) {
            flowerOn = !flowerOn;
            FlowerMechanism.setPosition(flowerOn ? 0.425 : 0.0);
        }

        // Driver 2 Live Trims
        if (gamepad2.dpadRightWasPressed()) headingOffset -= 5;
        if (gamepad2.dpadLeftWasPressed()) headingOffset += 5;
        if (gamepad2.dpadDownWasPressed()) velocityOffset -= 20;
        if (gamepad2.dpadUpWasPressed()) velocityOffset += 20;
    }

    private void computeAiming() {
        // Skip all trig and aiming math when both turret and shooter are off
        if (!turretOn && !shooterOn) return;

        if (!pinpointOK) {
            targetTicks = 0.5;
            velCorrection = 0;
            return;
        }

        // Update dynamic Goal X & Y only when aiming is active
        double baseGoalX = (robotX > 3.0) ? BASE_GOAL_X_POS
                : (robotX < -3.0) ? BASE_GOAL_X_NEG
                : (goalX >= 0 ? BASE_GOAL_X_POS : BASE_GOAL_X_NEG);

        double yOffset = robotY - BASE_GOAL_Y;
        double maxWallDist = (yOffset >= 0) ? (FIELD_HALF_WIDTH - BASE_GOAL_Y) : (FIELD_HALF_WIDTH + BASE_GOAL_Y);
        double sideRatio = clamp(yOffset / maxWallDist, -1.0, 1.0);
        double sideShift = sideRatio * MAX_SIDE_SHIFT;
        double depthShift = Math.abs(sideRatio) * MAX_X_SIDE_SHIFT;

        goalY = (baseGoalX < 0) ? (BASE_GOAL_Y - sideShift) : (BASE_GOAL_Y + sideShift);
        goalX = (baseGoalX < 0) ? (baseGoalX + depthShift) : (baseGoalX - depthShift);

        double yawRad = Math.toRadians(robotYaw);
        double turretX = robotX + TURRET_OFFSET_FORWARD * Math.cos(yawRad);
        double turretY = robotY + TURRET_OFFSET_FORWARD * Math.sin(yawRad);

        // Fast Euclidean distances (avoids slow Math.hypot software checks)
        double dx = goalX - turretX;
        double dy = goalY - turretY;
        distance = Math.sqrt(dx * dx + dy * dy);

        double vDeltaX = dx - velX * SHOT_TIME;
        double vDeltaY = dy - velY * SHOT_TIME;
        double virtualDist = Math.sqrt(vDeltaX * vDeltaX + vDeltaY * vDeltaY);

        // Shooter Velocity
        if (shooterOn) {
            double sideVelTrim = (robotX > 0) ? POS_SIDE_VEL_TRIM : 0;
            targetVel = (SHOOTER_SLOPE * distance) + SHOOTER_BASE_VEL + sideVelTrim;
            velCorrection = clamp((virtualDist - distance) / SHOT_TIME, -MAX_VEL_CORRECTION, MAX_VEL_CORRECTION);
        }

        // Turret Angle
        if (turretOn) {
            double targetAngle = AngleUnit.DEGREES.normalize(Math.toDegrees(Math.atan2(vDeltaY, vDeltaX)) + 180);
            double leadAngle = clamp(TURRET_KF * headingVel, -15, 15);
            double rawTurret = (((targetAngle - leadAngle) - robotYaw) % 360 + 360) % 360;

            boolean inRange = rawTurret <= SERVO_MAX_RANGE;
            double sideHeadTrim = (robotX > 0) ? POS_SIDE_HEADING_TRIM : 0;
            double normalized = inRange
                    ? (rawTurret + headingOffset + sideHeadTrim)
                    : (rawTurret >= (SERVO_MAX_RANGE + 360) / 2.0 ? 0 : SERVO_MAX_RANGE);

            targetTicks = clamp(normalized / SERVO_MAX_RANGE, 0.0, 1.0);

            if (!inRange && !wasOutOfRange) gamepad1.rumble(150);
            wasOutOfRange = !inRange;
        }
    }

    private void actuate() {
        // 1. Robot-Centric Omni Drive
        double y = deadband(gamepad1.left_stick_y);
        double x = deadband(gamepad1.left_stick_x);
        double rx = deadband(gamepad1.right_stick_x);

        double fl = (y - x) - rx;
        double bl = (y + x) - rx;
        double fr = (y + x) + rx;
        double br = (y - x) + rx;
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(bl)), Math.max(Math.abs(fr), Math.abs(br))));

        setDrivePower(FL, 0, fl / max);
        setDrivePower(BL, 1, bl / max);
        setDrivePower(FR, 2, fr / max);
        setDrivePower(BR, 3, br / max);

        // 2. Transfer (only sends mode/command when trigger state changes)
        boolean isFeeding = gamepad1.right_trigger > 0.1;
        if (wasFeeding == null || isFeeding != wasFeeding) {
            if (isFeeding) {
                Transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Transfer.setVelocity(TRANSFER_FEED_VEL);
            } else {
                Transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Transfer.setPower(TRANSFER_IDLE_POWER);
            }
            wasFeeding = isFeeding;
        }

        // 3. Intake (cuts power completely at 0 so PID doesn't burn battery holding position)
        double intakeCmd = isFeeding ? INTAKE_VEL
                : gamepad1.right_bumper ? -INTAKE_VEL
                : deadband(gamepad1.left_trigger) * INTAKE_VEL;
        if (hasChanged(intakeCmd, lastIntakeVel, 15.0)) {
            if (intakeCmd == 0) {
                Intake.setPower(0);
            } else {
                Intake.setVelocity(intakeCmd);
            }
            lastIntakeVel = intakeCmd;
        }

        // 4. Turret & Shooter (deadbands prevent servo hunting & PID current spikes)
        if (turretOn && hasChanged(targetTicks, lastTurretPos, 0.002)) {
            Turret.setPosition(targetTicks);
            lastTurretPos = targetTicks;
        }

        if (shooterOn) {
            double cmdVel = clamp(targetVel + velCorrection + velocityOffset, 0, MAX_SHOOTER_VEL);
            if (hasChanged(cmdVel, lastShooterVel, 8.0)) {
                Shooter.setVelocity(cmdVel);
                lastShooterVel = cmdVel;
            }
        }
    }

    private void setDrivePower(DcMotor motor, int idx, double power) {
        if (hasChanged(power, lastDrive[idx], 0.01)) {
            motor.setPower(power);
            lastDrive[idx] = power;
        }
    }

    private boolean hasChanged(double current, double last, double eps) {
        if (Double.isNaN(last)) return true;
        if (current == 0) return last != 0;
        return Math.abs(current - last) > eps;
    }

    private void updateTelemetry() {
        double cmdVel = shooterOn ? clamp(targetVel + velCorrection + velocityOffset, 0, MAX_SHOOTER_VEL) : 0;
        double actVel = shooterOn ? Shooter.getVelocity() : 0; // Avoids Hub read when shooter is off

        telemetry.addData("Loop Time", "%.1f ms (%.0f Hz)", loopMs, 1000.0 / Math.max(0.1, loopMs));
        telemetry.addData("Pose (X, Y, Yaw)", "%.1f, %.1f, %.1f°", robotX, robotY, robotYaw);
        telemetry.addData("Goal (X, Y) | Dist", "%.1f, %.1f | %.1f in", goalX, goalY, distance);
        telemetry.addData("Shooter Vel (Act / Cmd)", "%.0f / %.0f", actVel, cmdVel);
        telemetry.addData("Trims (Vel / Head)", "%d / %d", velocityOffset, headingOffset);
        telemetry.addData("Pinpoint OK", pinpointOK);
        telemetry.update();
    }

    private double deadband(double val) {
        return Math.abs(val) < STICK_DEADBAND ? 0 : val;
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}