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


@Autonomous(name = "TwoTipUpGarden", group = "Autonomous")
public class TwoTipUpGarden extends LinearOpMode {

    private final ElapsedTime pathTimer = new ElapsedTime();
    final int shooterVelocity = 1440;
    final int intakeVelocity = 2400;
    final int transferVelocity = 2800;
    final double flowerUp = 0.025;
    final double flowerDown = 0.3875;
    private Follower follower;
    private DcMotor Shooter;
    private DcMotor Intake;
    private DcMotor Transfer;
    private Servo FlowerMechanism;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(54.9267, 8.1789, 180);
    private final Pose scorePreload = poseFactory.of(56, 12, 180);
    private final Pose shootToGardenStart = poseFactory.of(56, 12, 180);
    private final Pose shootToGarden = poseFactory.of(18.8123, 21.3458, 270);
    private final Pose gardenGrabStart = poseFactory.of(18.8123, 21.3458, 270);
    private final Pose gardenGrab = poseFactory.of(8.1748, 8.6881, 270);
    private final Pose score2Start = poseFactory.of(8.1748, 8.6881, 270);
    private final Pose score2 = poseFactory.of(60, 130, 270);
    private final Pose score2Control1 = poseFactory.of(29.5643, 53.4624, 0);
    private final Pose score2Control2 = poseFactory.of(18.6022, 105.7942, 0);
    private final Pose scoreToFlowerStart = poseFactory.of(60, 130, 270);
    private final Pose scoreToFlower = poseFactory.of(20, 46.75, 180);
    private final Pose scoreToFlowerControl1 = poseFactory.of(16.549, 97.3057, 0);
    private final Pose flowerGrab1Start = poseFactory.of(20, 46.75, 180);
    private final Pose flowerGrab1 = poseFactory.of(12, 46.75, 180);
    private final Pose score3Start = poseFactory.of(12, 46.75, 180);
    private final Pose score3 = poseFactory.of(60, 130, 270);
    private final Pose score3Control1 = poseFactory.of(43.4134, 58.9438, 0);
    private final Pose score3Control2 = poseFactory.of(0, 82.2269, 0);
    private final Pose scoreToFlower2Start = poseFactory.of(60, 130, 270);
    private final Pose scoreToFlower2 = poseFactory.of(46.75, 108, 90);
    private final Pose flowerGrab2Start = poseFactory.of(46.75, 108, 90);
    private final Pose flowerGrab2 = poseFactory.of(46.75, 130, 90);
    private final Pose score4BufferStart = poseFactory.of(46.75, 130, 90);
    private final Pose score4Buffer = poseFactory.of(58, 110, 270);
    private final Pose score4Start = poseFactory.of(58, 110, 270);
    private final Pose score4 = poseFactory.of(60, 130, 270);
    private final Pose parkStart = poseFactory.of(60, 130, 270);
    private final Pose park = poseFactory.of(10, 118, 180);
    private final Pose parkControl1 = poseFactory.of(41.7276, 106.2799, 0);

    public Path scorePreload() {
        return Paths.line(start, scorePreload).linear(start, scorePreload);
    }

    public Path scoreToGarden() {
        return Paths.line(shootToGardenStart, shootToGarden).linear(shootToGardenStart, shootToGarden);
    }

    public Path gardenGrab() {
        return Paths.line(gardenGrabStart, gardenGrab).linear(gardenGrabStart, gardenGrab);
    }

    public Path score2() {
        return Paths.curve(score2Start, score2Control1, score2Control2, score2).linear(score2Start, score2);
    }

    public Path scoreToFlower() {
        return Paths.curve(scoreToFlowerStart, scoreToFlowerControl1, scoreToFlower).linear(scoreToFlowerStart, scoreToFlower);
    }

    public Path flowerGrab1() {
        return Paths.line(flowerGrab1Start, flowerGrab1).linear(flowerGrab1Start, flowerGrab1);
    }

    public Path score3() {
        return Paths.curve(score3Start, score3Control1, score3Control2, score3).linear(score3Start, score3);
    }

    public Path scoreToFlower2() {
        return Paths.line(scoreToFlower2Start, scoreToFlower2).linear(scoreToFlower2Start, scoreToFlower2);
    }

    public Path flowerGrab2() {
        return Paths.line(flowerGrab2Start, flowerGrab2).linear(flowerGrab2Start, flowerGrab2);
    }

    public Path score4Buffer() {
        return Paths.line(score4BufferStart, score4Buffer).linear(score4BufferStart, score4Buffer);
    }

    public Path score4() {
        return Paths.line(score4Start, score4).linear(score4Start, score4);
    }

    public Path park() {
        return Paths.curve(parkStart, parkControl1, park).linear(parkStart, park);
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
                follow(follower, scorePreload()),
                waitMs(1500),
                startShooting(),
                waitMs(1000),
                stopShooting(),
            // Shoot Pre-Load //

            // Drive to garden //
                follow(follower, scoreToGarden()),
                intakeOn(),
                waitMs(250),
            // Drive to garden //

            // Grab from garden //
                followWithTimeout(gardenGrab(), 1.25),
                waitMs(500),
                shooterOn(),
            // Grab from garden //

            // Shoot for the second time //
                follow(follower, score2()),
                waitMs(250),
                startShooting(),
                waitMs(1000),
                stopShooting(),
            // Shoot for the second time //

            // Drive to the first flower //
                follow(follower, scoreToFlower()),
                flowerDown(),
                intakeOn(),
                waitMs(500),
            // Drive to the first flower //

            // Grab out of the flower //
                followWithTimeout(flowerGrab1(),1),
                waitMs(1000),
                parallel (
                    intakeOff(),
                    shooterOn()
                ),
            // Grab out of the flower //

            // Score for the third time //
                follow(follower,score3()),
                waitMs(250),
                flowerUp(),
                startShooting(),
                waitMs(1000),
                stopShooting(),
            // Score for the third time //

            // Drive to the second flower //
                follow(follower, scoreToFlower2()),
                flowerDown(),
                waitMs(250),
                intakeOn(),
            // Drive to the second flower //

            // Grab out of the second flower //
                followWithTimeout(flowerGrab2(),1.25),
                waitMs(1000),
                parallel (
                    intakeOff(),
                    shooterOn()
                ),
            // Grab out of the second flower //

            // Score for the fourth time //
                follow(follower, score4Buffer()),
                waitMs(250),
                flowerUp(),
                follow(follower, score4()),
                waitMs(250),
                startShooting(),
                waitMs(1000),
                stopShooting(),
            // Score for the fourth time //

            // Park //
                follow(follower, park())
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
        Servo turret = hardwareMap.get(Servo.class, "Turret");

        // Shooter //
        Shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ((DcMotorEx) Shooter).setVelocityPIDFCoefficients(100, 0, 0, 15);
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