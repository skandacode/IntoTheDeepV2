import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
public class SpecAuto6 extends LinearOpMode {

    private Follower follower;
    Hang hang;
    Intake intake;
    Outtake outtake;

    /** Start Pose of our robot */
    private final Pose specGrab = new Pose(28, -55, Math.toRadians(90));
    private final Pose prespecGrab = new Pose(28, -47, Math.toRadians(90));
    private final Pose preIntake2 = new Pose(22, -42, Math.toRadians(33));
    private final Pose preIntake3 = new Pose(22, -42, Math.toRadians(25));
    private final Pose prescorePose = new Pose(-9, -45, Math.toRadians(90));
    private final Pose preIntake1 = new Pose(18, -46, Math.toRadians(40));
    private final Pose preloadScorePose = new Pose(-15, -27.5, Math.toRadians(90));
    private final Pose scorePose1 = new Pose(-12, -27.6, Math.toRadians(90));
    private final Pose scorePose2 = new Pose(-9, -27.4, Math.toRadians(90));
    private final Pose scorePose3 = new Pose(-6, -26.5, Math.toRadians(90));
    private final Pose scorePose4 = new Pose(-8, -26.5, Math.toRadians(90));

    private final Pose startPose = new Pose(-2, -61.5, Math.toRadians(90));
    private final Pose sample1 = new Pose(20, -37, Math.toRadians(38));
    private final Pose sample2 = new Pose(30, -39, Math.toRadians(33));
    private final Pose sample3 = new Pose(35.5, -36, Math.toRadians(27));
    private final Pose sampleReverse = new Pose(26, -42, Math.toRadians(-45));
    private final Pose park = new Pose(35, -55, Math.toRadians(0));
    private final Pose bucket = new Pose(-60, -61, Math.toRadians(0));

    public enum SampleStates {
        IDLE, EXTEND, RETRACT, OPENCOVER, WAIT, CLOSE, LIFT, PARTIALFLIP, SCORE, AUTOWAIT, OPEN, LOWERLIFT, EJECTFLIP, EJECTLIDOPEN
    }
    public enum SpecimenScoreStates {IDLE, C, INTAKEPOS, INTAKE, FULLYOPEN, CLOSE_CLAW, HOLD, SCORE, OPENCLAW, CLOSEBEFORERETRACT, RESET, RETRACT}

    public static int maxExtend=400;
    public static boolean extendPressed=false;
    public static boolean sampleScorePressed =false;


    public static boolean closedPressed=false;
    public static boolean specimenScoredPressed =false;
    public static boolean continueSpecScored=true;

    enum AutoStates{depositPosPreload, pauseToDepo, scorePreload,
        intakePreExtend1, intakeExtend1, intakeExtend1Pos, intakeReversePos1, intakeReverse1,
        intakePreExtend2, intakeExtend2, intakeExtend2Pos, intakeReversePos2, intakeReverse2,
        intakePreExtend3, intakeExtend3, intakeExtend3Pos, intakeReversePos3, intakeReverse3,
        prespecPos1,specPos1, closeClaw1, depositPos1, score1,
        prespecPos2, specPos2, closeClaw2, depositPos2, score2,
        prespecPos3, specPos3, closeClaw3, depositPos3, score3,
        prespecPos4, specPos4, closeClaw4, depositPos4, score4,
        prespecPos5, specPos5, closeClaw5, bucket, dropLift, openClaw, park, DONE}
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
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
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

                .state(SampleStates.RETRACT)
                .onEnter(()->{
                    intake.transferPos();
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
                .transition(() -> sampleScorePressed)

                .state(SampleStates.AUTOWAIT)
                .transitionTimed(0.4)

                .state(SampleStates.OPEN)
                .onEnter(() -> {
                    outtake.openClaw();
                    sampleScorePressed =false;
                })
                .transitionTimed(0.5)
                .onExit(() -> {
                    outtake.setTargetPos(0);
                    outtake.transferPos();
                })
                .state(SampleStates.LOWERLIFT)
                .loop(()->{
                    if (outtake.getFlipAnalog()>Outtake.axonAnalogFlipThresh && outtake.isRetracted()){
                        outtake.openClaw();
                    }
                })
                .transition(()->extendPressed && outtake.getFlipAnalog()>Outtake.axonAnalogFlipThresh, SampleStates.IDLE)
                .transition(() -> outtake.isRetracted() && outtake.getFlipAnalog()>Outtake.axonAnalogFlipThresh, SampleStates.IDLE)
                .onExit(()->outtake.openClaw())
                .build();

        StateMachine specimenScorer = new StateMachineBuilder()
                .state(SpecimenScoreStates.IDLE)

                .state(SpecimenScoreStates.C)
                .onEnter(()->{
                    outtake.closeClaw();
                    outtake.setTargetPos(200);
                })
                .transitionTimed(0.3)

                .state(SpecimenScoreStates.INTAKEPOS)
                .onEnter(() -> {
                    outtake.specGrab();
                    outtake.setTargetPos(200);
                })
                .transitionTimed(0.6)
                .state(SpecimenScoreStates.INTAKE)
                .onEnter(() -> {
                    outtake.setTargetPos(0);
                    outtake.openClaw();
                })
                .transitionTimed(0.6)
                .state(SpecimenScoreStates.FULLYOPEN)
                .onEnter(()->outtake.openClawWide())
                .transition(() -> closedPressed)
                .state(SpecimenScoreStates.CLOSE_CLAW)
                .onEnter(() -> {
                    outtake.closeClaw();
                    closedPressed=false;
                })
                .transitionTimed(0.2)
                .state(SpecimenScoreStates.HOLD)
                .loop(() -> outtake.specHold())
                .transition(() -> (outtake.atTarget() && specimenScoredPressed))
                .state(SpecimenScoreStates.SCORE)
                .loop(() -> {
                    outtake.specScore();
                    specimenScoredPressed =false;
                })
                .transitionTimed(0.3)
                .state(SpecimenScoreStates.OPENCLAW)
                .onEnter(() -> outtake.openClaw())
                .transitionTimed(0.3)
                .state(SpecimenScoreStates.CLOSEBEFORERETRACT)
                .onEnter(() -> outtake.closeClaw())
                .transition(()->continueSpecScored, SpecimenScoreStates.INTAKE, ()->{
                    outtake.specGrab();
                    outtake.setTargetPos(0);
                })
                .transitionTimed(0.3)
                .state(SpecimenScoreStates.RESET)
                .onEnter(() -> {
                    double curr=outtake.getSetPoint();
                    outtake.transferPos();
                    outtake.setTargetPos((int) curr);
                })
                .transitionTimed(0.3)
                .state(SpecimenScoreStates.RETRACT)
                .onEnter(() -> {
                    outtake.transferPos();
                    outtake.setTargetPos(0);
                })
                .transition(() -> outtake.isRetracted() && outtake.getFlipAnalog()>Outtake.axonAnalogFlipThresh, SpecimenScoreStates.IDLE)
                .onExit(()->outtake.openClaw())
                .build();


        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


        PathChain scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(preloadScorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), preloadScorePose.getHeading())
                .setZeroPowerAccelerationMultiplier(2.2)
                .build();
        PathChain preloadScoretointake1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(preloadScorePose), new Point(preIntake1), new Point(preIntake1)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                .build();
        PathChain intake1toreverse = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample1), new Point(sampleReverse)))
                .setLinearHeadingInterpolation(sample1.getHeading(), sampleReverse.getHeading())
                .build();
        PathChain reversetopreIntake = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sampleReverse), new Point(preIntake2)))
                .setLinearHeadingInterpolation(sampleReverse.getHeading(), preIntake2.getHeading())
                .build();
        PathChain preIntaketointake1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(preIntake1), new Point(sample1)))
                .setLinearHeadingInterpolation(preIntake1.getHeading(), sample1.getHeading())
                .build();
        PathChain preIntaketointake2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(preIntake2), new Point(sample2)))
                .setLinearHeadingInterpolation(preIntake2.getHeading(), sample2.getHeading())
                .build();
        PathChain intake2toreverse = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample2), new Point(sampleReverse)))
                .setLinearHeadingInterpolation(sample2.getHeading(), sampleReverse.getHeading())
                .build();
        PathChain preIntaketointake3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(preIntake3), new Point(sample3)))
                .setLinearHeadingInterpolation(preIntake3.getHeading(), sample3.getHeading())
                .build();
        PathChain intake3toreverse = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample3), new Point(sampleReverse)))
                .setLinearHeadingInterpolation(sample3.getHeading(), sampleReverse.getHeading())
                .build();
        PathChain reversetopregrab = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sampleReverse), new Point(prespecGrab)))
                .setLinearHeadingInterpolation(sampleReverse.getHeading(), prespecGrab.getHeading())
                .build();
        PathChain pregrabtograb = follower.pathBuilder()
                .addPath(new BezierLine(new Point(prespecGrab), new Point(specGrab)))
                .setLinearHeadingInterpolation(prespecGrab.getHeading(), specGrab.getHeading())
                .build();
        PathChain grabtoscore = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(specGrab), new Point(prescorePose), new Point(scorePose1)))
                .setLinearHeadingInterpolation(specGrab.getHeading(), scorePose1.getHeading())
                .setZeroPowerAccelerationMultiplier(1.8)
                .build();
        PathChain grabtoscore2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(specGrab), new Point(prescorePose), new Point(scorePose2)))
                .setLinearHeadingInterpolation(specGrab.getHeading(), scorePose2.getHeading())
                .setZeroPowerAccelerationMultiplier(1.8)
                .build();
        PathChain grabtoscore3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(specGrab), new Point(prescorePose), new Point(scorePose3)))
                .setLinearHeadingInterpolation(specGrab.getHeading(), scorePose3.getHeading())
                .setZeroPowerAccelerationMultiplier(1.8)
                .build();
        PathChain grabtoscore4 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(specGrab), new Point(prescorePose), new Point(scorePose4)))
                .setLinearHeadingInterpolation(specGrab.getHeading(), scorePose4.getHeading())
                .setZeroPowerAccelerationMultiplier(1.8)
                .build();
        PathChain grabtobucket = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(specGrab), new Point(bucket)))
                .setZeroPowerAccelerationMultiplier(5.7)
                .setReversed(true)
                .build();
        PathChain scoretopregrab = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose1), new Point(prespecGrab)))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), prespecGrab.getHeading())
                .build();
        PathChain score2topregrab = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose2), new Point(prespecGrab)))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), prespecGrab.getHeading())
                .build();
        PathChain score3topregrab = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose3), new Point(prespecGrab)))
                .setLinearHeadingInterpolation(scorePose3.getHeading(), prespecGrab.getHeading())
                .build();
        PathChain score4topregrab = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose4), new Point(prespecGrab)))
                .setLinearHeadingInterpolation(scorePose4.getHeading(), prespecGrab.getHeading())
                .build();
        PathChain buckettopark = follower.pathBuilder()
                .addPath(new BezierLine(new Point(bucket), new Point(park)))
                .build();



        StateMachine autoMachine=new StateMachineBuilder()
                .state(AutoStates.depositPosPreload)
                .onEnter(()->{
                    follower.followPath(scorePreload, true);
                    outtake.setRail(0.64);
                })
                .transitionTimed(1.3)
                .state(AutoStates.scorePreload)
                .onEnter(()->specimenScoredPressed=true)
                .transition(()->specimenScorer.getState()== SpecimenScoreStates.OPENCLAW)
                .state(AutoStates.intakePreExtend1)
                .onEnter(()->{
                    follower.followPath(preloadScoretointake1, true);
                })
                .transitionTimed(0.9)
                .state(AutoStates.intakeExtend1)
                .onEnter(()->{
                    intake.intakePos(200);
                    intake.setIntakePower(1);
                })
                .transitionTimed(0.2)
                .transition(()->intake.isSampleIntaked(), AutoStates.intakeReversePos1)
                .state(AutoStates.intakeExtend1Pos)
                .onEnter(()->{
                    intake.intakePos(500);
                })
                .transitionTimed(0.6)
                .transition(()->intake.isSampleIntaked(), AutoStates.intakeReversePos1)

                .state(AutoStates.intakeReversePos1)
                .onEnter(()->{
                    intake.intakePos(300);
                    follower.followPath(intake1toreverse, true);
                })
                .transitionTimed(0.6)
                .state(AutoStates.intakeReverse1)
                .onEnter(()->{
                    intake.intakePos(300);
                    intake.intakeEject();
                    intake.setIntakePower(-1);
                })
                .loop(()->intake.setIntakePower(-1))
                .transitionTimed(0.3)
                .state(AutoStates.intakePreExtend2)
                .onEnter(()->{
                    follower.followPath(reversetopreIntake, true);
                    intake.intakePos(300);
                })
                .transitionTimed(0.35)
                .state(AutoStates.intakeExtend2)
                .onEnter(()->{
                    intake.intakePos(375);
                    intake.setIntakePower(1);
                })
                .transitionTimed(0.05)
                .state(AutoStates.intakeExtend2Pos)
                .onEnter(()->follower.followPath(preIntaketointake2, true))
                .transitionTimed(0.7)
                .state(AutoStates.intakeReversePos2)
                .onEnter(()->follower.followPath(intake2toreverse, true))
                .transitionTimed(0.6)
                .state(AutoStates.intakeReverse2)
                .onEnter(()->{
                    intake.intakePos(330);
                    intake.setIntakePower(-0.7);
                })
                .loop(()->intake.setIntakePower(-1))
                .transitionTimed(0.25)
                .state(AutoStates.intakePreExtend3)
                .onEnter(()->{
                    follower.followPath(reversetopreIntake, true);
                })
                .transitionTimed(0.1)
                .state(AutoStates.intakeExtend3)
                .onEnter(()->{
                    intake.intakePos(430);
                    intake.setIntakePower(1);
                })
                .transitionTimed(0.2)
                .state(AutoStates.intakeExtend3Pos)
                .onEnter(()->follower.followPath(preIntaketointake3, true))
                .transitionTimed(0.7)
                .state(AutoStates.intakeReversePos3)
                .onEnter(()->follower.followPath(intake3toreverse, true))
                .transitionTimed(0.7)
                .state(AutoStates.intakeReverse3)
                .onEnter(()->intake.setIntakePower(-0.7))
                .loop(()->intake.setIntakePower(-0.7))
                .transitionTimed(0.3)
                .state(AutoStates.prespecPos1)
                .onEnter(()->{
                    follower.followPath(reversetopregrab, true);
                    intake.transferPos();
                    intake.setIntakePower(0);
                })
                .transitionTimed(0.7)
                .state(AutoStates.specPos1)
                .onEnter(()->{
                    follower.followPath(pregrabtograb, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.2)
                .state(AutoStates.closeClaw1)
                .onEnter(()->{
                    closedPressed=true;
                })
                .transitionTimed(0.1)
                .state(AutoStates.depositPos1)
                .onEnter(()->{
                    follower.followPath(grabtoscore, true);
                })
                .transitionTimed(1.6)
                .state(AutoStates.score1)
                .onEnter(()->specimenScoredPressed=true)
                .transition(()->specimenScorer.getState()== SpecimenScoreStates.OPENCLAW)
                .state(AutoStates.prespecPos2)
                .onEnter(()->{
                    follower.followPath(scoretopregrab, true);
                })
                .transitionTimed(0.7)
                .state(AutoStates.specPos2)
                .onEnter(()->{
                    follower.followPath(pregrabtograb, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.2)
                .state(AutoStates.closeClaw2)
                .onEnter(()->closedPressed=true)
                .transitionTimed(0.1)
                .state(AutoStates.depositPos2)
                .onEnter(()->{
                    follower.followPath(grabtoscore2, true);
                })
                .transitionTimed(1.5)
                .state(AutoStates.score2)
                .onEnter(()->specimenScoredPressed=true)
                .transition(()->specimenScorer.getState()== SpecimenScoreStates.OPENCLAW)
                .state(AutoStates.prespecPos3)
                .onEnter(()->{
                    follower.followPath(score2topregrab, true);
                })
                .transitionTimed(0.8)
                .state(AutoStates.specPos3)
                .onEnter(()->{
                    follower.followPath(pregrabtograb, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.2)
                .state(AutoStates.closeClaw3)
                .onEnter(()->closedPressed=true)
                .transitionTimed(0.1)
                .state(AutoStates.depositPos3)
                .onEnter(()->{
                    follower.followPath(grabtoscore3, true);
                })
                .transitionTimed(1.5)
                .state(AutoStates.score3)
                .onEnter(()->specimenScoredPressed=true)
                .transition(()->specimenScorer.getState()== SpecimenScoreStates.OPENCLAW)
                .state(AutoStates.prespecPos4)
                .onEnter(()->{
                    follower.followPath(score3topregrab, true);
                })
                .transitionTimed(1)
                .state(AutoStates.specPos4)
                .onEnter(()->{
                    follower.followPath(pregrabtograb, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.2)
                .state(AutoStates.closeClaw4)
                .onEnter(()->closedPressed=true)
                .transitionTimed(0.1)
                .state(AutoStates.depositPos4)
                .onEnter(()->{
                    follower.followPath(grabtoscore4, true);
                })
                .transitionTimed(1.6)
                .state(AutoStates.score4)
                .onEnter(()->specimenScoredPressed=true)
                .transition(()->specimenScorer.getState()== SpecimenScoreStates.OPENCLAW)
                .state(AutoStates.prespecPos5)
                .onEnter(()->{
                    follower.followPath(score4topregrab, true);
                })
                .transitionTimed(1)
                .state(AutoStates.specPos5)
                .onEnter(()->{
                    follower.followPath(pregrabtograb, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.2)
                .state(AutoStates.closeClaw5)
                .onEnter(()->outtake.closeClaw())
                .transitionTimed(0.1)
                .state(AutoStates.bucket)
                .onEnter(()->{
                    follower.followPath(grabtobucket, true);
                    outtake.setTargetPos(1280);
                })
                .transitionTimed(2.2)

                .state(AutoStates.dropLift)
                .onEnter(()->outtake.setTargetPos(1240))
                .transitionTimed(0.2)
                .state(AutoStates.openClaw)
                .onEnter(()-> {
                    outtake.openClaw();
                    outtake.setTargetPos((int)(outtake.getSetPoint()-40));
                })
                .transitionTimed(0.4)
                .state(AutoStates.park)
                .onEnter(()->{
                    outtake.transferPos();
                    follower.followPath(buckettopark,true);
                })
                .transitionTimed(2)
                .state(AutoStates.DONE)
                .onEnter(()->telemetry.addLine("Done"))
                .build();
        telemetry.addData("zero power accel multiplier", FollowerConstants.zeroPowerAccelerationMultiplier);
        telemetry.update();
        outtake.closeClaw();
        intake.transferPos();
        outtake.setTargetPos(0);
        while (opModeInInit()) {
            if (!(controlhub==null)) {
                controlhub.clearBulkCache();
                telemetry.addLine("bulk reading only chub");
            }else{
                for (LynxModule hub:allHubs){
                    hub.clearBulkCache();
                }
            }
            follower.updatePose();
            intake.update();
            outtake.update();
            telemetry.addData("Outtake motor power", outtake.currPower);

            telemetry.addData("Pose", follower.getPose().toString());
            telemetry.update();
        }
        waitForStart();
        autoMachine.start();
        sampleMachine.start();
        specimenScorer.start();
        specimenScorer.setState(SpecimenScoreStates.HOLD);
        outtake.specHold();
        follower.update();
        Pose prevPose=follower.getPose();
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
            specimenScorer.update();
            follower.update();
            //follower.telemetryDebug(telemetry);
            intake.update();
            outtake.update();

            System.out.println(follower.getPose().toString());
            Pose currPose=follower.getPose();
            if (normalize(Math.toDegrees(prevPose.getHeading()),Math.toDegrees(currPose.getHeading()))>20){
                System.out.println("PROBABLE JUMP");
            }
            prevPose=currPose;

            telemetry.addData("Path State", autoMachine.getState());
            telemetry.addData("Sample State", sampleMachine.getState());
            telemetry.addData("Outtake Pos", outtake.getCachedPos());

            telemetry.addData("Position", follower.getPose().toString());

            telemetry.update();
        }
    }
    private double normalize(double a, double b){
        double c=Math.abs(a-b);
        if (c>180){
            return c-180;
        }else{
            return c;
        }
    }
}
