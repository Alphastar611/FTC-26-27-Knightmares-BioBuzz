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



//Command myFirstCommand = Command.build()
//    .setStart(() -> {
//        // executed on start
//    })
//    .setExecute(() -> {
//        // executed on execute
//    })
//    .setDone(() -> {
//        // return true to end the command
//    })
//    .setEnd(endCondition -> {
//        // executed on end
//    })
//    .requiring(/* requirements */)
//    .setInterruptedBehavior(/* interrupted behavior */)
//    .setBlockedBehavior(/* blocked behavior */)
//    .setConflictBehavior(/* conflict behavior */)
//    .setPriority(/* priority */);


@Autonomous
public class BlueSideUp  extends  OpMode{
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose = p.of(24, 24, 0);
    private final Pose park = p.of(48, 48, 90);

    //poses from before

    private Path park() {
        return line(startPose, park).linear(startPose, park);
    }
    // other poses...
    private final Pose controlPose = p.of(36, 60, 45);







    @Override
    public void init() {
    Scheduler.reset();

    follower = Constants.create(hardwareMap);
    follower.setPose(startPose);
    }

    @Override
    public void loop() {
    }
}


