import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import subsystems.Hang;
import subsystems.Intake;

@Autonomous
public class SampleAutoPedro extends LinearOpMode {

    private Follower follower;
    Hang hang;
    Intake intake;

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(-36, -61.5, Math.toRadians(90));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(-51, -56.5, Math.toRadians(45));
    //private final Pose scorePose = new Pose(-36, -61.5, Math.toRadians(45));

    enum AutoStates{PRELOAD, DONE}
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        hang=new Hang(hardwareMap);
        hang.setPtoEngaged(false);
        intake=new Intake(hardwareMap);
        intake.transferPos();

        follower.setStartingPose(startPose);

        PathChain scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();



        StateMachine autoMachine=new StateMachineBuilder()
                .state(AutoStates.PRELOAD)
                .onEnter(()->follower.followPath(scorePreload, true))
                .transition(()->follower.atParametricEnd(), AutoStates.DONE)
                .state(AutoStates.DONE)
                .onEnter(()->telemetry.addLine("Done"))
                .build();
        telemetry.addData("zero power accel multiplier", FollowerConstants.zeroPowerAccelerationMultiplier);
        telemetry.update();
        waitForStart();
        autoMachine.start();
        while (opModeIsActive()){
            autoMachine.update();
            follower.update();
            follower.telemetryDebug(telemetry);
            intake.update();

            telemetry.addData("Path State", autoMachine.getState());
            telemetry.addData("Position", follower.getPose().toString());
            telemetry.update();
        }
    }
}
