package oldAutos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import subsystems.Hang;
import subsystems.Intake;
import subsystems.Outtake;

@Autonomous
@Disabled
public class SampleAutoPedro extends LinearOpMode {

    private Follower follower;
    Hang hang;
    Intake intake;
    Outtake outtake;

    /** Start Pose of our robot */
    private final Pose subPose = new Pose(-14, -9, Math.toRadians(0));
    private final Pose presubPose = new Pose(-51, -9, Math.toRadians(0));
    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(-50, -54, Math.toRadians(38));
    private final Pose startPose = new Pose(-36, -61.5, Math.toRadians(90));
    private final Pose sample1 = new Pose(-44, -49, Math.toRadians(90));
    private final Pose sample2 = new Pose(-53, -49, Math.toRadians(90));
    private final Pose sample3 = new Pose(-50, -45, Math.toRadians(122));

    public enum SampleStates {
        IDLE, EXTEND, SENSORWAIT, SENSE, RETRACT, OPENCOVER, WAIT, CLOSE, LIFT, PARTIALFLIP, SCORE, AUTOWAIT, OPEN, LOWERLIFT, EJECTFLIP, EJECTLIDOPEN
    }
    public static Intake.SampleColor allianceColor= Intake.SampleColor.BLUE;
    Intake.SampleColor currentSense= Intake.SampleColor.NONE;
    public static int maxExtend=400;

    public static boolean extendPressed=false;
    public static boolean scorePressed=false;
    public static boolean known=true;

    enum AutoStates{PRELOAD, WAIT, OPENCLAW1,
        TOSAMPLE1, INTAKE1, SCORESAMPLE1,WAIT2,OPENCLAW2,
        TOSAMPLE2, INTAKE2,SCORESAMPLE2, WAIT3,OPENCLAW3,
        TOSAMPLE3, INTAKE3,SCORESAMPLE3,WAIT4,OPENCLAW4,
        DONE}
    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        LynxModule controlhub = null;
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            if (hub.isParent()){
                controlhub=hub;
                break;
            }
        }

        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        hang=new Hang(hardwareMap);
        hang.setPtoEngaged(false);
        intake=new Intake(hardwareMap);
        intake.transferPos();
        outtake=new Outtake(hardwareMap);


        follower.setStartingPose(startPose);
        StateMachine sampleMachine = new StateMachineBuilder()
                .state(SampleStates.IDLE)
                .onEnter(() -> {
                    intake.transferPos();
                    intake.setIntakePower(0);
                })
                .transition(() -> extendPressed)
                .state(SampleStates.EXTEND)
                .onEnter(()->{
                    intake.intakePos(maxExtend);
                    extendPressed=false;
                })
                .transition(()->intake.isSampleIntaked())
                .state(SampleStates.SENSORWAIT)
                .onEnter(()->intake.intakePos())
                .transitionTimed(0.05)
                .transition(()->known, SampleStates.RETRACT)
                .state(SampleStates.SENSE)
                .transition(() -> {
                    currentSense=intake.getColor();
                    return currentSense == Intake.SampleColor.YELLOW || currentSense== allianceColor;
                }, SampleStates.RETRACT)
                .transition(()->currentSense == Intake.SampleColor.NONE, SampleStates.EXTEND)
                .transition(() -> currentSense != Intake.SampleColor.YELLOW && currentSense != allianceColor, SampleStates.EJECTFLIP)

                .state(SampleStates.EJECTFLIP, true)
                .onEnter(() -> {
                    intake.eject();
                })
                .transitionTimed(0.2, SampleStates.EJECTLIDOPEN)

                .state(SampleStates.EJECTLIDOPEN, true)
                .onEnter(() -> {
                    intake.setCover(false);
                })
                .transitionTimed(0.4, SampleStates.EXTEND)

                .state(SampleStates.RETRACT)
                .onEnter(()->{
                    intake.transferPos();
                    if (!known){
                        intake.setIntakePower(-1);
                    }
                    intake.setCover(true);
                    outtake.transferPos();
                    outtake.openClaw();
                })
                .transitionTimed(0.01)
                .state(SampleStates.OPENCOVER)
                .onEnter(() -> {
                    intake.setCover(false);
                    intake.setIntakePower(0.05);
                    outtake.openClaw();
                })
                .transition(()-> intake.isRetracted())

                .state(SampleStates.WAIT)
                .onEnter(() -> {
                    intake.setIntakePower(0.4);
                })
                .transitionTimed(0.3)

                .state(SampleStates.CLOSE)
                .onEnter(() -> {
                    outtake.closeClaw();
                    intake.setIntakePower(0.4);
                })
                .transitionTimed(0.3)

                .state(SampleStates.LIFT)
                .onEnter(() -> {
                    outtake.setTargetPos(970);
                    intake.setIntakePower(-0.5);
                })
                .transitionTimed(0.3)

                .state(SampleStates.PARTIALFLIP)
                .onEnter(()->outtake.partialSampleFlip())
                .transition(()->outtake.getCachedPos()>900)

                .state(SampleStates.SCORE)
                .onEnter(()->outtake.specGrab())
                .transition(() -> scorePressed)

                .state(SampleStates.AUTOWAIT)
                .transitionTimed(0.4)

                .state(SampleStates.OPEN)
                .onEnter(() -> {
                    outtake.openClaw();
                    scorePressed=false;
                })
                .transitionTimed(0.3)
                .onExit(() -> {
                    outtake.setTargetPos(0);
                    outtake.transferPos();
                })
                .state(SampleStates.LOWERLIFT)
                .loop(()->{
                    if (outtake.getFlipAnalog()>1.937 && outtake.isRetracted()){
                        outtake.openClaw();
                    }
                })
                .transition(()->extendPressed && outtake.getFlipAnalog()>1.937, SampleStates.IDLE)
                .transition(() -> outtake.isRetracted() && outtake.getFlipAnalog()>1.937, SampleStates.IDLE)
                .onExit(()->outtake.openClaw())
                .build();
        PathChain scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .setZeroPowerAccelerationMultiplier(2.5)
                .build();
        PathChain scoretoSamp1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(sample1)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), sample1.getHeading())
                .setZeroPowerAccelerationMultiplier(4)
                .build();
        PathChain samp1toScore = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample1), new Point(scorePose)))
                .setLinearHeadingInterpolation(sample1.getHeading(), scorePose.getHeading())
                .build();
        PathChain scoretoSamp2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(sample2)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), sample2.getHeading())
                .build();
        PathChain samp2toScore = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample2), new Point(scorePose)))
                .setLinearHeadingInterpolation(sample2.getHeading(), scorePose.getHeading())

                .build();
        PathChain scoretoSamp3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(sample3)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), sample3.getHeading())
                .build();
        PathChain samp3toScore = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample3), new Point(scorePose)))
                .setLinearHeadingInterpolation(sample3.getHeading(), scorePose.getHeading())
                .build();




        StateMachine autoMachine=new StateMachineBuilder()
                .state(AutoStates.PRELOAD)
                .onEnter(()->{
                    follower.followPath(scorePreload, true);
                    sampleMachine.setState(SampleStates.LIFT);
                })
                .transition(()->follower.atParametricEnd())
                .state(AutoStates.WAIT)
                .transitionTimed(0.2)
                .state(AutoStates.OPENCLAW1)
                .onEnter(()->scorePressed=true)
                .transition(()->sampleMachine.getState()==SampleStates.LOWERLIFT)

                .state(AutoStates.TOSAMPLE1)
                .onEnter(()->{
                    follower.followPath(scoretoSamp1, true);
                })
                .transitionTimed(0.3)
                .state(AutoStates.INTAKE1)
                .onEnter(()->{
                    extendPressed=true;
                })
                .transition(()->sampleMachine.getState()==SampleStates.RETRACT)

                .state(AutoStates.SCORESAMPLE1)
                .onEnter(()->follower.followPath(samp1toScore, true))
                .transition(()->follower.atParametricEnd())
                .state(AutoStates.WAIT2)
                .transitionTimed(0.05)
                .state(AutoStates.OPENCLAW2)
                .onEnter(()->scorePressed=true)
                .transition(()->sampleMachine.getState()==SampleStates.LOWERLIFT)

                .state(AutoStates.TOSAMPLE2)
                .onEnter(()->{
                    follower.followPath(scoretoSamp2, true);
                })
                .transitionTimed(0.3)
                .state(AutoStates.INTAKE2)
                .onEnter(()->{
                    extendPressed=true;
                })
                .transition(()->sampleMachine.getState()==SampleStates.RETRACT)

                .state(AutoStates.SCORESAMPLE2)
                .onEnter(()->follower.followPath(samp2toScore, true))
                .transition(()->follower.atParametricEnd())
                .state(AutoStates.WAIT3)
                .transitionTimed(0.05)
                .state(AutoStates.OPENCLAW3)
                .onEnter(()->scorePressed=true)
                .transition(()->sampleMachine.getState()==SampleStates.LOWERLIFT)

                .state(AutoStates.TOSAMPLE3)
                .onEnter(()->{
                    follower.followPath(scoretoSamp3, true);
                })
                .transitionTimed(0.5)
                .state(AutoStates.INTAKE3)
                .onEnter(()->{
                    extendPressed=true;
                })
                .transition(()->sampleMachine.getState()==SampleStates.RETRACT)

                .state(AutoStates.SCORESAMPLE3)
                .onEnter(()->follower.followPath(samp3toScore, true))
                .transition(()->follower.atParametricEnd())
                .state(AutoStates.WAIT4)
                .transitionTimed(0.05)
                .state(AutoStates.OPENCLAW4)
                .onEnter(()->scorePressed=true)
                .transition(()->sampleMachine.getState()==SampleStates.LOWERLIFT)

                .state(AutoStates.DONE)
                .onEnter(()->telemetry.addLine("Done"))
                .build();
        telemetry.addData("zero power accel multiplier", FollowerConstants.zeroPowerAccelerationMultiplier);
        telemetry.update();
        outtake.closeClaw();
        intake.setIntakeFlip(0.3);
        waitForStart();
        autoMachine.start();
        sampleMachine.start();
        while (opModeIsActive()){
            if (!(controlhub==null)) {
                controlhub.clearBulkCache();
                telemetry.addLine("bulk reading only chub");
            }else{
                for (LynxModule hub:allHubs){
                    hub.clearBulkCache();
                }
            }
            autoMachine.update();
            sampleMachine.update();
            follower.update();
            follower.telemetryDebug(telemetry);
            intake.update();
            outtake.update();

            telemetry.addData("Path State", autoMachine.getState());
            telemetry.addData("Sample State", sampleMachine.getState());

            telemetry.addData("Position", follower.getPose().toString());
            telemetry.update();
        }
    }
}
