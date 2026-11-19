package org.firstinspires.ftc.teamcode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class TurretShooter {

    // Cubic Shooter Curve (-0.00595183x^3 + 1.01756x^2 - 50.807x + 2230.21)
    public static final double SHOOTER_A = -0.00595183;
    public static final double SHOOTER_B = 1.01756;
    public static final double SHOOTER_C = -50.807;
    public static final double SHOOTER_D = 2230.21;
    public static final double POS_SIDE_VEL_TRIM = -40;
    public static final double MAX_SHOOTER_VEL = 1700.0;
    public static final double SHOT_TIME = 0.65;
    public static final double MAX_VEL_CORRECTION = 300.0;

    // Goal & Turret Geometry
    public static final double BASE_GOAL_X_NEG = -10.0;
    public static final double BASE_GOAL_X_POS = 10.0;
    public static final double BASE_GOAL_Y = 15.0;
    public static final double MAX_SIDE_SHIFT = 6.0;
    public static final double MAX_X_SIDE_SHIFT = 4.0;
    public static final double FIELD_HALF_WIDTH = 72.0;

    public static final double TURRET_OFFSET_FORWARD = -2.0;
    public static final double SERVO_MAX_RANGE = 355.0;
    public static final double TURRET_KF = 0.275;
    public static final double POS_SIDE_HEADING_TRIM = -3.0;

    private final RobotHardware hw;

    public boolean turretOn = false;
    public boolean shooterOn = false;
    public boolean turretInRange = true; // Exposes whether turret is safely aligned
    public int headingOffset = 0;
    public int velocityOffset = 0;

    public double goalX = BASE_GOAL_X_NEG;
    public double goalY = BASE_GOAL_Y;
    public double distance = 0;
    public double targetVel = 0;
    public double velCorrection = 0;
    public double targetTicks = 0.52;

    private boolean wasOutOfRange = false;
    private boolean lastShooterOff = false;
    private double lastTurretPos = Double.NaN;
    private double lastShooterVel = Double.NaN;

    public TurretShooter(RobotHardware hw) {
        this.hw = hw;
    }

    public void toggleTurret() { turretOn = !turretOn; }
    public void toggleShooter() { shooterOn = !shooterOn; }
    public void adjustHeading(int delta) { headingOffset += delta; }
    public void adjustVelocity(int delta) { velocityOffset += delta; }

    public double getCommandedVelocity() {
        return shooterOn ? clamp(targetVel + velCorrection + velocityOffset, 0, MAX_SHOOTER_VEL) : 0;
    }

    public double getActualVelocity() {
        return shooterOn ? hw.Shooter.getVelocity() : 0;
    }

    /** Updates aiming math and hardware; returns true on the frame the turret hits its travel limit. */
    public boolean update(Localization loc) {
        boolean triggerRumble = false;

        if (!loc.pinpointOK) {
            targetTicks = 0.5;
            velCorrection = 0;
            turretInRange = true;
        } else {
            // 1. Dynamic Goal X & Y
            double baseGoalX = (loc.x > 3.0) ? BASE_GOAL_X_POS
                    : (loc.x < -3.0) ? BASE_GOAL_X_NEG
                    : (goalX >= 0 ? BASE_GOAL_X_POS : BASE_GOAL_X_NEG);

            double yOffset = loc.y - BASE_GOAL_Y;
            double maxWallDist = (yOffset >= 0) ? (FIELD_HALF_WIDTH - BASE_GOAL_Y) : (FIELD_HALF_WIDTH + BASE_GOAL_Y);
            double sideRatio = clamp(yOffset / maxWallDist, -1.0, 1.0);
            double sideShift = sideRatio * MAX_SIDE_SHIFT;
            double depthShift = Math.abs(sideRatio) * MAX_X_SIDE_SHIFT;

            goalY = (baseGoalX < 0) ? (BASE_GOAL_Y - sideShift) : (BASE_GOAL_Y + sideShift);
            goalX = (baseGoalX < 0) ? (baseGoalX + depthShift) : (baseGoalX - depthShift);

            // 2. Turret Position & Distances
            double yawRad = Math.toRadians(loc.yaw);
            double turretX = loc.x + TURRET_OFFSET_FORWARD * Math.cos(yawRad);
            double turretY = loc.y + TURRET_OFFSET_FORWARD * Math.sin(yawRad);

            double dx = goalX - turretX;
            double dy = goalY - turretY;
            distance = Math.sqrt(dx * dx + dy * dy);

            double vDeltaX = dx - loc.velX * SHOT_TIME;
            double vDeltaY = dy - loc.velY * SHOT_TIME;
            double virtualDist = Math.sqrt(vDeltaX * vDeltaX + vDeltaY * vDeltaY);

            // 3. Shooter Velocity
            double sideVelTrim = (loc.x > 0) ? POS_SIDE_VEL_TRIM : 0;
            double baseCurveVel = ((SHOOTER_A * distance + SHOOTER_B) * distance + SHOOTER_C) * distance + SHOOTER_D;
            targetVel = baseCurveVel + sideVelTrim;
            velCorrection = clamp((virtualDist - distance) / SHOT_TIME, -MAX_VEL_CORRECTION, MAX_VEL_CORRECTION);

            // 4. Turret Angle
            double targetAngle = AngleUnit.DEGREES.normalize(Math.toDegrees(Math.atan2(vDeltaY, vDeltaX)) + 180);
            double leadAngle = clamp(TURRET_KF * loc.headingVel, -15, 15);
            double rawTurret = (((targetAngle - leadAngle) - loc.yaw) % 360 + 360) % 360;

            boolean inRange = rawTurret <= SERVO_MAX_RANGE;
            turretInRange = inRange; // Store alignment state

            double sideHeadTrim = (loc.x > 0) ? POS_SIDE_HEADING_TRIM : 0;
            double normalized = inRange
                    ? (rawTurret + headingOffset + sideHeadTrim)
                    : (rawTurret >= (SERVO_MAX_RANGE + 360) / 2.0 ? 0 : SERVO_MAX_RANGE);

            targetTicks = clamp(normalized / SERVO_MAX_RANGE, 0.0, 1.0);

            boolean outOfRange = turretOn && !inRange;
            if (outOfRange && !wasOutOfRange) triggerRumble = true;
            wasOutOfRange = outOfRange;
        }

        // 5. Actuate Turret & Shooter
        double turretCmd = turretOn ? targetTicks : 0.52;
        if (hw.needsWrite(turretCmd, lastTurretPos, 0.002, 6)) {
            hw.Turret.setPosition(turretCmd);
            lastTurretPos = turretCmd;
        }

        if (shooterOn) {
            double cmdVel = getCommandedVelocity();
            if (hw.needsWrite(cmdVel, lastShooterVel, 5.0, 8)) {
                hw.Shooter.setVelocity(cmdVel);
                lastShooterVel = cmdVel;
                lastShooterOff = false;
            }
        } else if (hw.refreshSlot == 8 || !lastShooterOff) {
            hw.Shooter.setPower(0);
            lastShooterOff = true;
            lastShooterVel = Double.NaN;
        }

        return triggerRumble;
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}