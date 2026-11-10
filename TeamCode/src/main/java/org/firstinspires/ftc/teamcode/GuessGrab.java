package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;


@Autonomous(name = "GuessGrab", group = "Autonomous")
public class GuessGrab extends LinearOpMode {

    private final ElapsedTime pathTimer = new ElapsedTime();
    final int shooterVelocity = 1460;
    final int intakeVelocity = 2400;
    final int transferVelocity = 2800;
    final double flowerUp = 0.025;
    final double flowerDown = 0.3875;
    private Follower follower;
    private DcMotor Shooter;
    private DcMotor Intake;
    private DcMotor Transfer;
    private Servo FlowerMechanism;
    private Servo turret;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(54.39, 8.1789, 90);
    private final Pose shoot1 = poseFactory.of(35.4602, 12.3255, 60);
    private final Pose guessGrab = poseFactory.of(58.5689, 107.9968, 104.9536);
    private final Pose guessGrabControl1 = poseFactory.of(77.9751, 38.0105, 0);
    private final Pose guessGrabControl2 = poseFactory.of(32.7428, 24.1482, 0);
    private final Pose guessGrabControl3 = poseFactory.of(89.9972, 84.9434, 0);
    private final Pose guessGrabControl4 = poseFactory.of(43.8004, 83.5634, 0);
    private final Pose guessGrabControl5 = poseFactory.of(64.0801, 87.3618, 0);
    private final Pose shoot2Start = poseFactory.of(58.5689, 107.9968, 90);
    private final Pose shoot2 = poseFactory.of(31.1991, 130.1789, 300);
    private final Pose parkStart = poseFactory.of(31.1991, 130.1789, 300);
    private final Pose park = poseFactory.of(11.6808, 121.3973, 270);

    public Path shoot1() {
        return Paths.line(start, shoot1).linear(start, shoot1);
    }

    public Path guessGrab() {
        return Paths.curve(shoot1, guessGrabControl1, guessGrabControl2, guessGrabControl3, guessGrabControl4, guessGrabControl5, guessGrab).tangent();
    }

    public Path shoot2() {
        return Paths.line(shoot2Start, shoot2).linear(shoot2Start, shoot2);
    }

    public Path park() {
        return Paths.line(parkStart, park).linear(parkStart, park);
    }
    private Command followWithTimeout(Path path, double timeoutSec) {
        return sequential(
                instant(pathTimer::reset),
                follow(follower, path).until(() -> pathTimer.seconds() > timeoutSec)
        );
    }
    private Command stopShooting() {
        return parallel(
                instant(() -> Intake.setPower(0)),
                instant(() -> Transfer.setPower(0)),
                instant(() -> Shooter.setPower(0))
        );
    }
    private Command startShooting(){
        return parallel(
                instant(() -> ((DcMotorEx) Intake).setVelocity(intakeVelocity)),
                instant(this::transferFeed)
        );
    }
    private Command shooterOn() {
        return instant(() -> ((DcMotorEx) Shooter).setVelocity(shooterVelocity));
    }

    private void transferHoldBack() {
        Transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Transfer.setPower(-0.2);
    }

    private void transferFeed() {
        // Feeding the shooter: encoder mode, velocity control
        if (Transfer.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            Transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        ((DcMotorEx) Transfer).setVelocity(transferVelocity);
    }

    private void transferStop() {
        Transfer.setPower(0);
    }

// ---------- Commands ----------

    private Command intakeOn() {
        return instant(() -> {
            ((DcMotorEx) Intake).setVelocity(intakeVelocity);
            transferHoldBack();
        });
    }

    private Command intakeOff() {
        return instant(() -> {
            Intake.setPower(0);
            transferStop();
        });
    }
    private Command flowerDown(){
        return instant(() -> FlowerMechanism.setPosition(flowerDown));
    }
    private Command flowerUp(){
        return instant(() -> FlowerMechanism.setPosition(flowerUp));
    }

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                // Shoot Pre-Load //
                shooterOn(),
                follow(follower, shoot1()),
                waitMs(2000),
                startShooting(),
                waitMs(1500),
                stopShooting(),
                // Shoot Pre-Load //

                // Wait for tip and then guess intake //
                intakeOn(),
                waitMs(5000),
                follow(follower, guessGrab()),
                shooterOn(),
                // Wait for tip and then guess intake //

                // Shoot for the second time //
                follow(follower, shoot2()),
                waitMs(1500),
                startShooting(),
                waitMs(2000),
                stopShooting(),
                // Shoot for the second time //

                // Park //
                follow(follower,park())
                // Park //
        );
    }

    @Override
    public void runOpMode() {

        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        Shooter = hardwareMap.get(DcMotor.class, "Shooter");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Transfer = hardwareMap.get(DcMotor.class, "Transfer");
        FlowerMechanism = hardwareMap.get(Servo.class, "Flower Mechanism");
        turret = hardwareMap.get(Servo.class, "Turret");

        // Shooter //
        Shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ((DcMotorEx) Shooter).setVelocityPIDFCoefficients(50, 0, 0, 15);
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shooter.setDirection(DcMotor.Direction.REVERSE);
        Shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Shooter //


        // Intake //
        Intake.setDirection(DcMotor.Direction.REVERSE);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ((DcMotorEx) Intake).setVelocityPIDFCoefficients(30, 0, 0, 16);
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Intake //


        // Transfer //
        Transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ((DcMotorEx) Transfer).setVelocityPIDFCoefficients(30, 0, 0, 14);
        Transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Transfer //

        turret.setPosition(0.5);

        FlowerMechanism.setPosition(0.025);

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());
            telemetry.addData("Shooter Velocity", ((DcMotorEx) Shooter).getVelocity());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }
}