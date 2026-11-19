package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RobotHardware {

    public DcMotor FL, BL, FR, BR;
    public DcMotorEx Shooter, Intake, Transfer;
    public ServoImplEx Turret;
    public Servo FlowerMechanism;
    public GoBildaPinpointDriver Pinpoint;
    public List<LynxModule> hubs;

    // Staggered refresh counter (updates 1 device every 30ms to avoid bus spikes)
    public int refreshSlot = -1;
    private int nextRefreshSlot = 0;
    private final ElapsedTime refreshTimer = new ElapsedTime();

    public void init(HardwareMap hwMap) {
        Shooter = hwMap.get(DcMotorEx.class, "Shooter");
        Intake = hwMap.get(DcMotorEx.class, "Intake");
        Transfer = hwMap.get(DcMotorEx.class, "Transfer");
        FL = hwMap.get(DcMotor.class, "FL");
        BL = hwMap.get(DcMotor.class, "BL");
        FR = hwMap.get(DcMotor.class, "FR");
        BR = hwMap.get(DcMotor.class, "BR");
        Turret = hwMap.get(ServoImplEx.class, "Turret");
        FlowerMechanism = hwMap.get(Servo.class, "Flower Mechanism");
        Pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");

        hubs = hwMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Pinpoint setup
        Pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        Pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        Pinpoint.setOffsets(0, -2.5, DistanceUnit.INCH);

        // Velocity motors
        setupVelocityMotor(Shooter, DcMotor.Direction.REVERSE, 50, 14);
        setupVelocityMotor(Intake, DcMotor.Direction.REVERSE, 30, 16);
        setupVelocityMotor(Transfer, DcMotor.Direction.FORWARD, 30, 14);

        // Drive motors
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
        FlowerMechanism.setPosition(0.05);

        refreshTimer.reset();
    }

    private void setupVelocityMotor(DcMotorEx motor, DcMotor.Direction dir, double p, double f) {
        motor.setDirection(dir);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setVelocityPIDFCoefficients(p, 0, 0, f);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void beginLoop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        refreshSlot = -1;
        if (refreshTimer.milliseconds() >= 30) {
            nextRefreshSlot = (nextRefreshSlot + 1) % 9;
            refreshSlot = nextRefreshSlot;
            refreshTimer.reset();
        }
    }

    public boolean needsWrite(double current, double last, double eps, int slot) {
        if (refreshSlot == slot || Double.isNaN(last)) return true;
        if (current == 0) return last != 0;
        return Math.abs(current - last) > eps;
    }
}