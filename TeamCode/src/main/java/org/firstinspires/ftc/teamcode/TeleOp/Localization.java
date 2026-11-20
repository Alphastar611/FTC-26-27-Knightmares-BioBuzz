package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

public class Localization {

    // Default target values for manual re-localization via Back button
    public static final double DEFAULT_START_X = -59.0;
    public static final double DEFAULT_START_Y = 70.0;
    public static final double DEFAULT_START_HEADING = -90.0;
    public static final double VEL_SMOOTHING = 0.5;

    private final RobotHardware Hardware;
    private final ElapsedTime resetTimer = new ElapsedTime();
    private boolean resetPending = false;

    public double x = DEFAULT_START_X;
    public double y = DEFAULT_START_Y;
    public double yaw = DEFAULT_START_HEADING;
    public double velX = 0, velY = 0, headingVel = 0;
    public boolean pinpointOK = false;

    // 1. Standalone Constructor (Used if running TeleOp without Auto)
    public Localization(RobotHardware hw) {
        this.Hardware = hw;
        resetPose();
    }

    // 2. Auto Handoff Constructor (Sets transferred auto pose instantly without wiping it)
    public Localization(RobotHardware hw, double initialX, double initialY, double initialYaw) {
        this.Hardware = hw;
        this.x = initialX;
        this.y = initialY;
        this.yaw = initialYaw;

        Hardware.Pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, initialX, initialY, AngleUnit.DEGREES, initialYaw));
        resetPending = false;
    }

    // Manual re-localization trigger (resets IMU and snaps back to DEFAULT constants after 500ms)
    public void resetPose() {
        Hardware.Pinpoint.resetPosAndIMU();
        resetTimer.reset();
        resetPending = true;
    }

    public boolean isStationary() {
        return Math.abs(velX) <= 1.0 && Math.abs(velY) <= 1.0 && Math.abs(headingVel) <= 5.0;
    }

    public void update() {
        // If driver pressed Back to re-localize, apply DEFAULT starting coordinates after 500ms
        if (resetPending && resetTimer.milliseconds() >= 500) {
            Hardware.Pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, DEFAULT_START_X, DEFAULT_START_Y, AngleUnit.DEGREES, DEFAULT_START_HEADING));
            resetPending = false;
            velX = velY = headingVel = 0;
        }

        try {
            Hardware.Pinpoint.update();
            pinpointOK = Hardware.Pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
            if (pinpointOK) {
                double px = Hardware.Pinpoint.getPosX(DistanceUnit.INCH);
                double py = Hardware.Pinpoint.getPosY(DistanceUnit.INCH);
                double heading = Hardware.Pinpoint.getHeading(AngleUnit.DEGREES);
                double vx = Hardware.Pinpoint.getVelX(DistanceUnit.INCH);
                double vy = Hardware.Pinpoint.getVelY(DistanceUnit.INCH);
                double hv = Hardware.Pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);

                if (Double.isFinite(px) && Double.isFinite(py) && Double.isFinite(heading)) {
                    x = px;
                    y = py;
                    yaw = heading;
                    if (Double.isFinite(vx) && Double.isFinite(vy)) {
                        velX += VEL_SMOOTHING * (vx - velX);
                        velY += VEL_SMOOTHING * (vy - velY);
                    }
                    if (Double.isFinite(hv)) {
                        headingVel += VEL_SMOOTHING * (hv - headingVel);
                    }
                }
            }
        } catch (Exception ignored) {
            pinpointOK = false;
        }
    }
}