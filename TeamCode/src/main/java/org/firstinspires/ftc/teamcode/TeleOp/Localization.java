package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

public class Localization {

    public static final double START_X = -59.0;
    public static final double START_Y = 70.0;
    public static final double START_HEADING = -90.0;
    public static final double VEL_SMOOTHING = 0.5;

    private final RobotHardware hw;
    private final ElapsedTime resetTimer = new ElapsedTime();
    private boolean resetPending = false;

    public double x = START_X;
    public double y = START_Y;
    public double yaw = START_HEADING;
    public double velX = 0, velY = 0, headingVel = 0;
    public boolean pinpointOK = false;

    public Localization(RobotHardware hw) {
        this.hw = hw;
        resetPose();
    }

    public void resetPose() {
        hw.Pinpoint.resetPosAndIMU();
        resetTimer.reset();
        resetPending = true;
    }

    public boolean isStationary() {
        return Math.abs(velX) <= 1.0 && Math.abs(velY) <= 1.0 && Math.abs(headingVel) <= 5.0;
    }

    public void update() {
        if (resetPending && resetTimer.milliseconds() >= 500) {
            hw.Pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, START_X, START_Y, AngleUnit.DEGREES, START_HEADING));
            resetPending = false;
            velX = velY = headingVel = 0;
        }

        try {
            hw.Pinpoint.update();
            pinpointOK = hw.Pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
            if (pinpointOK) {
                double px = hw.Pinpoint.getPosX(DistanceUnit.INCH);
                double py = hw.Pinpoint.getPosY(DistanceUnit.INCH);
                double heading = hw.Pinpoint.getHeading(AngleUnit.DEGREES);
                double vx = hw.Pinpoint.getVelX(DistanceUnit.INCH);
                double vy = hw.Pinpoint.getVelY(DistanceUnit.INCH);
                double hv = hw.Pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);

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