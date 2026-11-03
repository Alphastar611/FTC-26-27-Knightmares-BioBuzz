package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import static com.pedropathing.api.Paths.*;

import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;




@Autonomous
public class BlueSideUp extends OpMode {
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    // Poses
    private final Pose startPose = poseFactory.of(-70, 14, 0);
    private final Pose scorePose1 = poseFactory.of(-66, 10, 0);
    private final Pose flowerPose1 = poseFactory.of(-24, 56, 90);
    private final Pose controlPose = poseFactory.of(30, 60, 180);
    private final Pose controlPose2 = poseFactory.of(42, 6, 180);
    private final Pose controlPose3 = poseFactory.of(20, 18, 180);
    private final Pose scorePose2 = poseFactory.of(58, 6, 180);
    private final Pose flowerPose2 = poseFactory.of(54, 18, 0);
    private final Pose parkPose = poseFactory.of(66, 68, 0);

    // Path methods
    private Path startToScore() {
        return line(startPose, scorePose1).linear(startPose, scorePose1);
    }
    private Path park(){
        return line(scorePose1, parkPose).linear(scorePose1, parkPose);
    }
    private Path flower1(){
        return line(scorePose1, flowerPose1).linear(scorePose1, flowerPose1);
    }
    private Path score2(){
        return curve(flowerPose1,controlPose,controlPose2, scorePose2).linear(flowerPose1, scorePose2);
    }
    private Path flower2(){
        return curve(scorePose2,controlPose3, flowerPose2).linear(scorePose2, flowerPose2);
    }
    private Path score3(){
        return line(flowerPose2, scorePose2).linear(flowerPose2, scorePose2);
    }
    private Command autoRoutine() {
        return sequential(
                follow(follower, startToScore()),
                follow(follower, flower1()),
                follow(follower, score2()),
                follow(follower, flower2()),
                follow(follower, score3()),
                follow(follower, park())
        );
    }
    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
    }
    @Override
    public void start() {
        schedule(autoRoutine());
    }
    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
        // add your other methods needed in the loop here
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }
}