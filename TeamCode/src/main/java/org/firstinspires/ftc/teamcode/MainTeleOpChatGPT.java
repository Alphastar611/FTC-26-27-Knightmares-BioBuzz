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

@TeleOp(name = "MainTeleOp (100% NOT AI)")
public class MainTeleOpChatGPT extends LinearOpMode {

    // Hardware
    private DcMotor Shooter;
    private DcMotor FR;
    private DcMotor BR;
    private Servo FlowerMechanism;
    private DcMotor Intake;
    private Servo Turret;
    private DcMotor FL;
    private DcMotor BL;
    private DcMotor Transfer;
    private GoBildaPinpointDriver Pinpoint;

    // Driver inputs
    double Y;
    double X;
    double RX;
    final double Stick_Deadband = 0.05;

    // Switches
    boolean Turret_Switch;
    boolean Flower_Switch;
    boolean Shooter_Switch;

    // Localization
    double Robot_Yaw;
    double Robot_X;
    double Robot_Y;
    double Robot_X_Velocity = 0;
    double Robot_Y_Velocity = 0;
    double Robot_Heading_Velocity = 0;

    final double Start_X = -59;
    final double Start_Y = 70;
    final double Start_Heading = -90;

    // Pinpoint reliability
    boolean Pinpoint_OK = false;
    GoBildaPinpointDriver.DeviceStatus Pinpoint_Status;
    boolean hasGoodPose = false;
    boolean acceptNextPose = true;
    int rejectedFrames = 0;
    int consecutiveRejects = 0;
    int pinpointErrors = 0;

    final double Max_Plausible_Speed = 120;
    final double Max_Heading_Velocity = 1500;
    final double Max_Pose_Jump = 12;
    final double Max_Heading_Jump = 45;
    final double Absolute_Max_Pose_Jump = 100;
    final int Max_Consecutive_Rejects = 5;
    final double Velocity_Filter_Alpha = 0.5;

    final double Reset_Max_Speed = 1;
    final double Reset_Max_Heading_Velocity = 5;

    // If Pinpoint stops producing trusted frames, stop using stale automatic aiming
    // after this many milliseconds.
    final double Pinpoint_Aim_Timeout_Ms = 250;
    private final ElapsedTime pinpointGoodFrameTimer = new ElapsedTime();

    // Loop diagnostics
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime refreshTimer = new ElapsedTime();
    double loopMs = 0;
    double maxLoopMs = 0;
    int stallCount = 0;
    final double Stall_Threshold_Ms = 50;
    boolean firstLoop = true;
    boolean forceWrite = false;
    final double Refresh_Interval_Ms = 200;

    // Goal
    int Goal_X;
    int Goal_Y;
    final int Base_Goal_Y = 18;
    final double Goal_Switch_Band = 3;

    // Turret
    double Target_Ticks;
    double Servo_Max_Range;
    double Target_Heading = 0;
    double Turret_KF;
    double Turret_KF2;
    boolean Turret_In_Range;
    boolean wasTurretOutOfRange = false;
    double Normalized_Target_Heading;
    double Target = 0;
    double targetVelocity = 0;
    final double Turret_Offset_Forward = -2;
    int headingStep = 5;
    double Turret_X;
    double Turret_Y;
    double yawRad = 0;
    double Delta_X;
    double Delta_Y;

    // Shooter
    final int VelocityStep = 20;
    int VelocityOffset;

    // Hard limits for commanded shooter velocity.
    // Adjust these to the safe tested operating range of your shooter.
    final double Min_Shooter_Velocity = 0;
    final double Max_Shooter_Velocity = 3000;

    // Shoot-on-the-move
    final double Shot_Time = 0.65;
    final double Max_Velocity_Correction = 300;
    double KDT = 1;
    double Virtual_Goal_X = 0;
    double Virtual_Goal_Y = 0;
    double Virtual_Delta_X = 0;
    double Virtual_Delta_Y = 0;
    double virtualDistance = 0;
    int headingOffset = 0;
    double dh_dt = 0;
    double distance = 0;
    double velocityCorrection = 0;

    // Command caching
    private final double[] lastDrive = {
            Double.NaN, Double.NaN, Double.NaN, Double.NaN
    };
    private double lastIntakeVel = Double.NaN;
    private double lastTransferVel = Double.NaN;
    private double lastTransferPower = Double.NaN;
    private double lastShooterVel = Double.NaN;
    private double lastShooterPower = Double.NaN;
    private double lastTurretPos = Double.NaN;
    private double lastFlowerPos = Double.NaN;

    private boolean transferUsingEncoder = true;
    private boolean shooterUsingEncoder = true;

    // Misc
    final boolean BrakeModeEnabled = false;
    private final ElapsedTime resetTimer = new ElapsedTime();
    private boolean resetPending = false;
    private static final double RESET_DELAY_MS = 500;

    private void Init() {
        // Pinpoint
        Pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );
        Pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        Pinpoint.setOffsets(0, -2.5, DistanceUnit.INCH);
        Pinpoint.resetPosAndIMU();

        resetTimer.reset();
        resetPending = true;
        acceptNextPose = true;

        // Shooter
        Shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ((DcMotorEx) Shooter).setVelocityPIDFCoefficients(50, 0, 0, 14);
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterUsingEncoder = true;
        Shooter.setDirection(DcMotor.Direction.REVERSE);
        Shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Intake
        Intake.setDirection(DcMotor.Direction.REVERSE);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ((DcMotorEx) Intake).setVelocityPIDFCoefficients(30, 0, 0, 16);
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Transfer
        Transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ((DcMotorEx) Transfer).setVelocityPIDFCoefficients(30, 0, 0, 14);
        Transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transferUsingEncoder = true;

        // Turret
        ((ServoImplEx) Turret).setPwmRange(new PwmControl.PwmRange(500, 2500));
        Turret.setDirection(Servo.Direction.REVERSE);

        // Flower
        FlowerMechanism.setDirection(Servo.Direction.FORWARD);

        // Drive train
        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.FORWARD);

        DcMotor.ZeroPowerBehavior driveZeroPower =
                BrakeModeEnabled
                        ? DcMotor.ZeroPowerBehavior.BRAKE
                        : DcMotor.ZeroPowerBehavior.FLOAT;

        FR.setZeroPowerBehavior(driveZeroPower);
        BR.setZeroPowerBehavior(driveZeroPower);
        FL.setZeroPowerBehavior(driveZeroPower);
        BL.setZeroPowerBehavior(driveZeroPower);

        // State
        Flower_Switch = false;
        Shooter_Switch = false;
        Turret_Switch = false;
        Turret_In_Range = true;

        Robot_X = Start_X;
        Robot_Y = Start_Y;
        Robot_Yaw = Start_Heading;

        Delta_X = 0;
        Delta_Y = 0;

        Goal_X = -8;
        Goal_Y = Base_Goal_Y;

        Target = 0;
        Turret_KF = 0.275;
        Normalized_Target_Heading = 0;

        Servo_Max_Range = 355;
        Target_Ticks = 0.5;

        pinpointGoodFrameTimer.reset();
    }

    private void Telemetry2() {
        telemetry.addData("Robot X", Robot_X);
        telemetry.addData("Robot Y", Robot_Y);
        telemetry.addData("Robot Yaw", Robot_Yaw);
        telemetry.addData("Goal X", Goal_X);
        telemetry.addData("Goal Y", Goal_Y);
        telemetry.addData("Delta X", Delta_X);
        telemetry.addData("Delta Y", Delta_Y);
        telemetry.addData("Distance from goal", distance);
        telemetry.addData("Shooter Velocity", ((DcMotorEx) Shooter).getVelocity());
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Commanded Shooter Velocity",
                clampShooterVelocity(targetVelocity + velocityCorrection + VelocityOffset));
        telemetry.addData("Heading Offset", headingOffset);
        telemetry.addData("Velocity Offset", VelocityOffset);
        telemetry.addData("Pinpoint OK", Pinpoint_OK);
        telemetry.addData("Pinpoint Status", Pinpoint_Status);
        telemetry.addData("Rejected Frames", rejectedFrames);
        telemetry.addData("Pinpoint Errors", pinpointErrors);
        telemetry.addData("Consecutive Rejects", consecutiveRejects);
        telemetry.addData("Loop ms", loopMs);
        telemetry.addData("Max Loop ms", maxLoopMs);
        telemetry.addData("Stall Count", stallCount);
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        Shooter = hardwareMap.get(DcMotor.class, "Shooter");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");
        FlowerMechanism = hardwareMap.get(Servo.class, "Flower Mechanism");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Turret = hardwareMap.get(Servo.class, "Turret");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BL = hardwareMap.get(DcMotor.class, "BL");
        Transfer = hardwareMap.get(DcMotor.class, "Transfer");
        Pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        Init();
        waitForStart();

        if (isStopRequested()) {
            return;
        }

        loopTimer.reset();
        refreshTimer.reset();

        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            UpdateLoopDiagnostics();
            ReadInputs();
            UpdateGoalAndToggles();
            ComputeAiming();
            Actuate();
            Telemetry2();
        }
    }

    private void UpdateLoopDiagnostics() {
        loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        if (firstLoop) {
            firstLoop = false;
        } else {
            if (loopMs > maxLoopMs) {
                maxLoopMs = loopMs;
            }

            if (loopMs > Stall_Threshold_Ms) {
                stallCount++;
            }
        }

        if (gamepad2.backWasPressed()) {
            maxLoopMs = 0;
            stallCount = 0;
            rejectedFrames = 0;
            pinpointErrors = 0;
            consecutiveRejects = 0;
        }

        forceWrite = refreshTimer.milliseconds() >= Refresh_Interval_Ms;

        if (forceWrite) {
            refreshTimer.reset();
        }
    }

    private void ReadInputs() {
        // Manual pose reset
        if (gamepad1.backWasPressed()) {
            boolean stationary =
                    Math.abs(Robot_X_Velocity) <= Reset_Max_Speed
                            && Math.abs(Robot_Y_Velocity) <= Reset_Max_Speed
                            && Math.abs(Robot_Heading_Velocity) <= Reset_Max_Heading_Velocity;

            if (stationary) {
                Pinpoint.resetPosAndIMU();
                resetTimer.reset();
                resetPending = true;
                acceptNextPose = true;
                hasGoodPose = false;
                consecutiveRejects = 0;
            } else {
                gamepad1.rumble(150);
            }
        }

        if (resetPending && resetTimer.milliseconds() >= RESET_DELAY_MS) {
            Pinpoint.setPosition(
                    new Pose2D(
                            DistanceUnit.INCH,
                            Start_X,
                            Start_Y,
                            AngleUnit.DEGREES,
                            Start_Heading
                    )
            );

            resetPending = false;
            acceptNextPose = true;
            hasGoodPose = false;
            Robot_X_Velocity = 0;
            Robot_Y_Velocity = 0;
            Robot_Heading_Velocity = 0;
        }

        // Drive sticks
        Y = Deadband(gamepad1.left_stick_y);
        X = Deadband(gamepad1.left_stick_x);
        RX = Deadband(gamepad1.right_stick_x);

        // Pinpoint
        Pinpoint_OK = false;

        try {
            Pinpoint.update();
            Pinpoint_Status = Pinpoint.getDeviceStatus();

            if (Pinpoint_Status == GoBildaPinpointDriver.DeviceStatus.READY) {
                AcceptPinpointFrame(
                        Pinpoint.getPosX(DistanceUnit.INCH),
                        Pinpoint.getPosY(DistanceUnit.INCH),
                        Pinpoint.getHeading(AngleUnit.DEGREES),
                        Pinpoint.getVelX(DistanceUnit.INCH),
                        Pinpoint.getVelY(DistanceUnit.INCH),
                        Pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES)
                );
            }
        } catch (Exception e) {
            pinpointErrors++;
        }
    }

    private void AcceptPinpointFrame(
            double px,
            double py,
            double yaw,
            double vx,
            double vy,
            double hv
    ) {
        // Correct finite-value semantics:
        // true means the value is actually finite.
        if (isNotFinite(px)
                || isNotFinite(py)
                || isNotFinite(yaw)
                || isNotFinite(vx)
                || isNotFinite(vy)
                || isNotFinite(hv)) {
            rejectedFrames++;
            return;
        }

        boolean jumpy = false;

        if (hasGoodPose && !acceptNextPose) {
            double jump = Math.hypot(px - Robot_X, py - Robot_Y);
            double headingJump =
                    Math.abs(AngleUnit.DEGREES.normalize(yaw - Robot_Yaw));

            // Absolute sanity boundary: never accept an enormous single-frame jump.
            if (jump > Absolute_Max_Pose_Jump) {
                rejectedFrames++;
                consecutiveRejects++;
                return;
            }

            jumpy =
                    jump > Max_Pose_Jump
                            || headingJump > Max_Heading_Jump;
        }

        if (jumpy && consecutiveRejects < Max_Consecutive_Rejects) {
            consecutiveRejects++;
            rejectedFrames++;
            return;
        }

        consecutiveRejects = 0;
        acceptNextPose = false;
        hasGoodPose = true;

        Robot_X = px;
        Robot_Y = py;
        Robot_Yaw = yaw;

        if (Math.abs(vx) <= Max_Plausible_Speed
                && Math.abs(vy) <= Max_Plausible_Speed) {

            Robot_X_Velocity =
                    Velocity_Filter_Alpha * vx
                            + (1 - Velocity_Filter_Alpha) * Robot_X_Velocity;

            Robot_Y_Velocity =
                    Velocity_Filter_Alpha * vy
                            + (1 - Velocity_Filter_Alpha) * Robot_Y_Velocity;
        }

        if (Math.abs(hv) <= Max_Heading_Velocity) {
            // Light filtering on heading velocity too.
            Robot_Heading_Velocity =
                    Velocity_Filter_Alpha * hv
                            + (1 - Velocity_Filter_Alpha)
                            * Robot_Heading_Velocity;
        }

        Pinpoint_OK = true;
        pinpointGoodFrameTimer.reset();
    }

    private boolean isNotFinite(double v) {
        return !Double.isFinite(v);
    }

    private double Deadband(double value) {
        return Math.abs(value) < Stick_Deadband ? 0 : value;
    }

    private void UpdateGoalAndToggles() {
        // Automatic goal switching with hysteresis.
        if (Robot_X > Goal_Switch_Band) {
            Goal_X = 10;
        } else if (Robot_X < -Goal_Switch_Band) {
            Goal_X = -6;
        }

        if (gamepad2.dpadRightWasPressed()) {
            headingOffset -= headingStep;
        } else if (gamepad2.dpadLeftWasPressed()) {
            headingOffset += headingStep;
        }

        if (gamepad2.dpadDownWasPressed()) {
            VelocityOffset -= VelocityStep;
        } else if (gamepad2.dpadUpWasPressed()) {
            VelocityOffset += VelocityStep;
        }

        if (gamepad1.aWasPressed()) {
            Turret_Switch = !Turret_Switch;
        }

        if (gamepad1.leftBumperWasPressed()) {
            Flower_Switch = !Flower_Switch;
        }

        if (gamepad1.bWasPressed()) {
            Shooter_Switch = !Shooter_Switch;
        }
    }

    private void ComputeAiming() {
        // A trusted Pinpoint frame is required to update the aiming solution.
        if (!Pinpoint_OK) {
            // Do not immediately replace the last solution with bad data.
            // However, don't hold stale automatic aiming forever.
            if (pinpointGoodFrameTimer.milliseconds() >= Pinpoint_Aim_Timeout_Ms) {
                Target_Ticks = 0.5;
                velocityCorrection = 0;
            }
            return;
        }

        yawRad = Math.toRadians(Robot_Yaw);

        Turret_X =
                Robot_X
                        + Turret_Offset_Forward * Math.cos(yawRad);

        Turret_Y =
                Robot_Y
                        + Turret_Offset_Forward * Math.sin(yawRad);

        Delta_X = Goal_X - Turret_X;
        Delta_Y = Goal_Y - Turret_Y;

        distance = Math.hypot(Delta_X, Delta_Y);

        // Shoot-on-the-move virtual goal.
        Virtual_Goal_X =
                Goal_X - Robot_X_Velocity * Shot_Time;

        Virtual_Goal_Y =
                Goal_Y - Robot_Y_Velocity * Shot_Time;

        Virtual_Delta_X =
                Virtual_Goal_X - Turret_X;

        Virtual_Delta_Y =
                Virtual_Goal_Y - Turret_Y;

        virtualDistance =
                Math.hypot(Virtual_Delta_X, Virtual_Delta_Y);

        // Preserve the original shooter calibration based on real distance.
        targetVelocity =
                (3.64541 * distance) + 1340;

        dh_dt =
                (virtualDistance - distance) / Shot_Time;

        velocityCorrection =
                Math.max(
                        -Max_Velocity_Correction,
                        Math.min(
                                Max_Velocity_Correction,
                                dh_dt * KDT
                        )
                );

        // Aiming at the virtual goal.
        Target =
                AngleUnit.DEGREES.normalize(
                        Math.toDegrees(
                                Math.atan2(
                                        Virtual_Delta_Y,
                                        Virtual_Delta_X
                                )
                        ) + 180
                );

        Turret_KF2 =
                Math.max(
                        -15,
                        Math.min(
                                15,
                                Turret_KF * Robot_Heading_Velocity
                        )
                );

        Target_Heading = Target - Turret_KF2;

        double rawTurretAngle =
                ((Target_Heading - Robot_Yaw) % 360 + 360) % 360;

        Turret_In_Range =
                rawTurretAngle <= Servo_Max_Range;

        if (Turret_In_Range) {
            Normalized_Target_Heading =
                    rawTurretAngle + headingOffset;
        } else {
            // In the dead zone, park at whichever travel end is closer.
            double deadZoneMid =
                    (Servo_Max_Range + 360) / 2;

            Normalized_Target_Heading =
                    rawTurretAngle >= deadZoneMid
                            ? 0
                            : Servo_Max_Range;
        }

        // Final turret actuator safety clamp.
        Target_Ticks =
                clamp(
                        Normalized_Target_Heading / Servo_Max_Range,
                        0.0,
                        1.0
                );
    }

    private void Actuate() {
        // Omni-Wheel drive normalization.
        double fl = (Y - X) - RX;
        double bl = (Y + X) - RX;
        double fr = (Y + X) + RX;
        double br = (Y - X) + RX;

        double max =
                Math.max(
                        1.0,
                        Math.max(
                                Math.max(Math.abs(fl), Math.abs(bl)),
                                Math.max(Math.abs(fr), Math.abs(br))
                        )
                );

        SetDrive(FL, 0, fl / max);
        SetDrive(BL, 1, bl / max);
        SetDrive(FR, 2, fr / max);
        SetDrive(BR, 3, br / max);

        // Intake + Transfer
        if (gamepad1.right_trigger > 0.1) {
            SetTransferVelocity();
            SetIntakeVelocity(2400);
        } else if (gamepad1.right_bumper) {
            SetTransferPower();
            SetIntakeVelocity(-2400);
        } else {
            SetTransferPower();
            SetIntakeVelocity(gamepad1.left_trigger * 2400);
        }

        // Turret
        SetServo(
                Turret,
                true,
                Turret_Switch ? Target_Ticks : 0.5
        );

        // Shooter
        if (Shooter_Switch) {
            double finalShooterVelocity =
                    clampShooterVelocity(
                            targetVelocity
                                    + velocityCorrection
                                    + VelocityOffset
                    );

            SetShooterVelocity(finalShooterVelocity);
        } else {
            SetShooterOff();
        }

        // Flower
        SetServo(
                FlowerMechanism,
                false,
                Flower_Switch ? 0.425 : 0
        );

        // One rumble when turret first leaves its valid travel range.
        boolean turretOutOfRange =
                Turret_Switch && !Turret_In_Range;

        if (turretOutOfRange && !wasTurretOutOfRange) {
            gamepad1.rumble(150);
        }

        wasTurretOutOfRange = turretOutOfRange;
    }

    // Cached hub writes

    private boolean NeedsWrite(
            double newV,
            double lastV,
            double eps
    ) {
        if (forceWrite || Double.isNaN(lastV)) {
            return true;
        }

        if (newV == 0) {
            return lastV != 0;
        }

        return Math.abs(newV - lastV) > eps;
    }

    private void SetDrive(
            DcMotor motor,
            int idx,
            double power
    ) {
        power = clamp(power, -1.0, 1.0);

        if (NeedsWrite(power, lastDrive[idx], 0.01)) {
            motor.setPower(power);
            lastDrive[idx] = power;
        }
    }

    private void SetIntakeVelocity(double v) {
        if (NeedsWrite(v, lastIntakeVel, 5)) {
            ((DcMotorEx) Intake).setVelocity(v);
            lastIntakeVel = v;
        }
    }

    private void SetShooterVelocity(double v) {
        v = clampShooterVelocity(v);

        SetShooterEncoderMode(true);

        if (NeedsWrite(v, lastShooterVel, 2)) {
            ((DcMotorEx) Shooter).setVelocity(v);
            lastShooterVel = v;
            lastShooterPower = Double.NaN;
        }
    }

    private void SetShooterOff() {
        SetShooterEncoderMode(false);

        if (NeedsWrite(0, lastShooterPower, 0.001)) {
            Shooter.setPower(0);
            lastShooterPower = 0;
            lastShooterVel = Double.NaN;
        }
    }

    private void SetShooterEncoderMode(boolean useEncoder) {
        if (useEncoder == shooterUsingEncoder) {
            return;
        }

        Shooter.setMode(
                useEncoder
                        ? DcMotor.RunMode.RUN_USING_ENCODER
                        : DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        shooterUsingEncoder = useEncoder;
        lastShooterVel = Double.NaN;
        lastShooterPower = Double.NaN;
    }

    private void SetServo(
            Servo servo,
            boolean isTurret,
            double pos
    ) {
        // Servo output is always constrained to the SDK-safe range.
        pos = clamp(pos, 0.0, 1.0);

        double last =
                isTurret
                        ? lastTurretPos
                        : lastFlowerPos;

        if (NeedsWrite(pos, last, 0.001)) {
            servo.setPosition(pos);

            if (isTurret) {
                lastTurretPos = pos;
            } else {
                lastFlowerPos = pos;
            }
        }
    }

    private void SetTransferVelocity() {
        SetTransferEncoderMode(true);

        if (NeedsWrite(2800, lastTransferVel, 1)) {
            ((DcMotorEx) Transfer).setVelocity(2800);
            lastTransferVel = 2800;
            lastTransferPower = Double.NaN;
        }
    }

    private void SetTransferPower() {
        SetTransferEncoderMode(false);

        if (NeedsWrite(-0.125, lastTransferPower, 0.001)) {
            Transfer.setPower(-0.125);
            lastTransferPower = -0.125;
            lastTransferVel = Double.NaN;
        }
    }

    private void SetTransferEncoderMode(boolean useEncoder) {
        if (useEncoder == transferUsingEncoder) {
            return;
        }

        Transfer.setMode(
                useEncoder
                        ? DcMotor.RunMode.RUN_USING_ENCODER
                        : DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        transferUsingEncoder = useEncoder;
        lastTransferVel = Double.NaN;
        lastTransferPower = Double.NaN;
    }

    private double clamp(
            double value,
            double min,
            double max
    ) {
        return Math.max(min, Math.min(max, value));
    }

    private double clampShooterVelocity(double value) {
        return clamp(
                value,
                Min_Shooter_Velocity,
                Max_Shooter_Velocity
        );
    }
}
