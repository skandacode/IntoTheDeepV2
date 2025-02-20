package Old_autos;

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
public class SampleAutoPedroPreExtend extends LinearOpMode {

    private Follower follower;
    Hang hang;
    Intake intake;
    Outtake outtake;

    /** Start Pose of our robot */
    private final Pose subPose = new Pose(-14, -9, Math.toRadians(0));
    private final Pose substrafe = new Pose(-14, 0, Math.toRadians(0));
    private final Pose subPose2 = new Pose(-14, -6, Math.toRadians(0));
    private final Pose substrafe2 = new Pose(-14, 3, Math.toRadians(0));
    private final Pose subPose3 = new Pose(-14, -3, Math.toRadians(0));
    private final Pose substrafe3 = new Pose(-14, 5, Math.toRadians(0));
    private final Pose presubPose = new Pose(-51, -9, Math.toRadians(0));
    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(-58.5, -48, Math.toRadians(80));
    private final Pose scorePosepreload = new Pose(-53, -51, Math.toRadians(67
    ));
    private final Pose startPose = new Pose(-36, -61.5, Math.toRadians(90));
    private final Pose sample1 = new Pose(-52, -50, Math.toRadians(67));
    private final Pose sample2 = new Pose(-58.5, -49, Math.toRadians(80));
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
    public static int count=0;

    enum AutoStates{PRELOAD, WAIT, OPENCLAW1,
        TOSAMPLE1, INTAKE1, SCORESAMPLE1,WAIT2,OPENCLAW2,
        TOSAMPLE2, INTAKE2,SCORESAMPLE2, WAIT3,OPENCLAW3,
        TOSAMPLE3, INTAKE3,SCORESAMPLE3,WAIT4,OPENCLAW4,
        TOSUB1, EXTENDSUB1, INTAKESUB1, CHOOSE_STATE, RETRACTSUB1, REEXTEND1, REINTAKE1, STRAFE1, TOSCORESUB1, WAITSUB1, OPENCLAWSUB1,
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
                .loop(()->{
                    if (outtake.getFlipAnalog()>1.937 && outtake.isRetracted()){
                        outtake.openClaw();
                    }
                })
                .transition(()->intake.isSampleIntaked())
                .transitionTimed(2)
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
                    outtake.setRail(0.5);
                    outtake.setFlip(0.69);
                })
                .transitionTimed(waitTime(known))

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
                .onEnter(()->outtake.sampleScore())

                .transition(() -> scorePressed)

                .state(SampleStates.AUTOWAIT)

                .transitionTimed(0.2)

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
        StateMachine groundSampleMachine = new StateMachineBuilder()
                .state(SampleStates.IDLE)
                .onEnter(() -> {
                    intake.transferPos();
                    intake.setIntakePower(0);
                })
                .transition(() -> known)
                .state(SampleStates.EXTEND)
                .onEnter(()->{
                    intake.intakePos(maxExtend);
                    extendPressed=false;
                })
                .loop(()->{
                    if (outtake.getFlipAnalog()>1.937 && outtake.isRetracted()){
                        outtake.openClaw();
                    }
                })
                .transition(()->intake.isSampleIntaked())
                .transitionTimed(2)
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
                    outtake.setRail(0.5);
                    outtake.setFlip(0.69);
                })
                .transitionTimed(waitTime(known))

                .state(SampleStates.CLOSE)
                .onEnter(() -> {
                    outtake.closeClaw();
                    intake.setIntakePower(0.4);
                })
                .transitionTimed(0.3)

                .state(SampleStates.LIFT)
                .onEnter(() -> {
                    outtake.setTargetPos(970);
                    intake.intakePos(maxExtend-200);
                    intake.setIntakePower(0);
                })
                .transitionTimed(0.3)

                .state(SampleStates.PARTIALFLIP)
                .onEnter(()->outtake.partialSampleFlip())

                .transition(()->outtake.getCachedPos()>900)

                .state(SampleStates.SCORE)
                .onEnter(()->outtake.sampleScore())

                .transition(() -> scorePressed)

                .state(SampleStates.AUTOWAIT)
                .transitionTimed(0.2)

                .state(SampleStates.OPEN)
                .onEnter(() -> {
                    outtake.openClaw();
                    scorePressed=false;
                })
                .transitionTimed(0.4)
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
                .transition(()->outtake.getFlipAnalog()>1.937, SampleStates.IDLE)
                .transition(() -> outtake.isRetracted() && outtake.getFlipAnalog()>1.937, SampleStates.IDLE)
                .onExit(()->outtake.openClaw())
                .build();

        PathChain scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePosepreload)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePosepreload.getHeading())
                .setZeroPowerAccelerationMultiplier(1.5)
                .build();
        PathChain scoretoSamp1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePosepreload), new Point(sample1)))
                .setLinearHeadingInterpolation(scorePosepreload.getHeading(), sample1.getHeading())
                .setZeroPowerAccelerationMultiplier(1.5)
                .build();
        PathChain samp1toScore = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample1), new Point(scorePose)))
                .setLinearHeadingInterpolation(sample1.getHeading(), scorePose.getHeading())
                .setZeroPowerAccelerationMultiplier(1.5)
                .build();
        PathChain scoretoSamp2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(sample2)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), sample2.getHeading())
                .setZeroPowerAccelerationMultiplier(1.5)
                .build();
        PathChain samp2toScore = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample2), new Point(scorePose)))
                .setLinearHeadingInterpolation(sample2.getHeading(), scorePose.getHeading())
                .setZeroPowerAccelerationMultiplier(1.5)
                .build();
        PathChain scoretoSamp3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(sample3)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), sample3.getHeading())
                .setZeroPowerAccelerationMultiplier(1.5)
                .build();
        PathChain samp3toScore = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample3), new Point(scorePose)))
                .setLinearHeadingInterpolation(sample3.getHeading(), scorePose.getHeading())
                .setZeroPowerAccelerationMultiplier(1.5)
                .build();
        PathChain scoretosub = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(presubPose),new Point(subPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), subPose.getHeading())
                .setZeroPowerAccelerationMultiplier(4.5)
                .build();
        PathChain subtoscore = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(subPose), new Point(presubPose),new Point(scorePose)))
                .setLinearHeadingInterpolation(subPose.getHeading(), scorePose.getHeading())
                .setZeroPowerAccelerationMultiplier(4.5)
                .build();
        PathChain subtostrafe = follower.pathBuilder()
                .addPath(new BezierLine(new Point(subPose), new Point(substrafe)))
                .setLinearHeadingInterpolation(subPose.getHeading(), substrafe.getHeading())
                .setZeroPowerAccelerationMultiplier(4.5)
                .build();
        PathChain scoretosub2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(presubPose),new Point(subPose2)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), subPose2.getHeading())
                .setZeroPowerAccelerationMultiplier(4.5)
                .build();
        PathChain sub2toscore = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(subPose2), new Point(presubPose),new Point(scorePose)))
                .setLinearHeadingInterpolation(subPose2.getHeading(), scorePose.getHeading())
                .setZeroPowerAccelerationMultiplier(4.5)
                .build();
        PathChain sub2tostrafe2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(subPose2), new Point(substrafe2)))
                .setLinearHeadingInterpolation(subPose2.getHeading(), substrafe2.getHeading())
                .setZeroPowerAccelerationMultiplier(4.5)
                .build();
        PathChain scoretosub3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(presubPose),new Point(subPose3)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), subPose3.getHeading())
                .setZeroPowerAccelerationMultiplier(4.5)
                .build();
        PathChain sub3toscore = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(subPose3), new Point(presubPose),new Point(scorePose)))
                .setLinearHeadingInterpolation(subPose3.getHeading(), scorePose.getHeading())
                .setZeroPowerAccelerationMultiplier(4.5)
                .build();
        PathChain sub3tostrafe3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(subPose3), new Point(substrafe3)))
                .setLinearHeadingInterpolation(subPose3.getHeading(), substrafe3.getHeading())
                .setZeroPowerAccelerationMultiplier(4.5)
                .build();




        StateMachine autoMachine=new StateMachineBuilder()
                .state(AutoStates.PRELOAD)
                .onEnter(()->{
                    follower.followPath(scorePreload, true);
                    groundSampleMachine.setState(SampleStates.LIFT);
                })
                .transitionTimed(0.7)
                .state(AutoStates.WAIT)
                .transitionTimed(0.2)
                .state(AutoStates.OPENCLAW1)
                .onEnter(()->scorePressed=true)
                .transition(()->groundSampleMachine.getState()== SampleStates.LOWERLIFT)

                .state(AutoStates.TOSAMPLE1)
                .onEnter(()->{
                    follower.followPath(scoretoSamp1, true);
                })
                .transitionTimed(0.3)
                .transition(()->groundSampleMachine.getState()== SampleStates.RETRACT)

                .state(AutoStates.INTAKE1)
                .transition(()->groundSampleMachine.getState()== SampleStates.RETRACT)
                .transitionTimed(1, AutoStates.INTAKE1, ()->intake.setIntakePower(-1))

                .state(AutoStates.SCORESAMPLE1)
                .onEnter(()->follower.followPath(samp1toScore, true))
                .transition(()->follower.atParametricEnd())
                .state(AutoStates.WAIT2)
                .transitionTimed(0.05)
                .state(AutoStates.OPENCLAW2)
                .onEnter(()->{
                    scorePressed=true;
                })
                .onEnter(()->scorePressed=true)
                .transition(()->groundSampleMachine.getState()== SampleStates.LOWERLIFT)

                .state(AutoStates.TOSAMPLE2)
                .onEnter(()->{
                    follower.followPath(scoretoSamp2, true);
                })
                .transitionTimed(0.3)
                .transition(()->groundSampleMachine.getState()== SampleStates.RETRACT)
                .state(AutoStates.INTAKE2)

                .transition(()->groundSampleMachine.getState()== SampleStates.RETRACT)
                .transitionTimed(1, AutoStates.INTAKE2, ()->intake.setIntakePower(-1))

                .state(AutoStates.SCORESAMPLE2)
                .onEnter(()->follower.followPath(samp2toScore, true))
                .transition(()->follower.atParametricEnd())
                .state(AutoStates.WAIT3)
                .transitionTimed(0.05)
                .state(AutoStates.OPENCLAW3)
                .onEnter(()->{
                    scorePressed=true;
                })
                .transition(()->groundSampleMachine.getState()== SampleStates.LOWERLIFT)

                .state(AutoStates.TOSAMPLE3)
                .onEnter(()->{
                    follower.followPath(scoretoSamp3, true);
                })
                .transitionTimed(0.5)
                .transition(()->groundSampleMachine.getState()== SampleStates.RETRACT)
                .state(AutoStates.INTAKE3)

                .transition(()->groundSampleMachine.getState()== SampleStates.RETRACT)
                .transitionTimed(1, AutoStates.INTAKE3, ()->intake.setIntakePower(-1))
                .state(AutoStates.SCORESAMPLE3)
                .onEnter(()->follower.followPath(samp3toScore, true))
                .transition(()->follower.atParametricEnd())
                .state(AutoStates.WAIT4)
                .transitionTimed(0.05)
                .state(AutoStates.OPENCLAW4)
                .onEnter(()->scorePressed=true)
                .transition(()->groundSampleMachine.getState()== SampleStates.LOWERLIFT)

                .state(AutoStates.TOSUB1)
                .onEnter(()->{
                    if (count==0){
                        follower.followPath(scoretosub);
                    }else if (count==1){
                        follower.followPath(scoretosub2);
                    }else{
                        follower.followPath(scoretosub3);
                    }
                    known=false;
                })
                .transitionTimed(0.95)

                .state(AutoStates.EXTENDSUB1)
                .onEnter(()->intake.setTargetPos(maxExtend))
                .transitionTimed(0.5)

                .state(AutoStates.INTAKESUB1)
                .onEnter(()->extendPressed=true)
                .transition(()->sampleMachine.getState()== SampleStates.RETRACT, AutoStates.TOSCORESUB1)
                .transitionTimed(0.7)

                .state(AutoStates.CHOOSE_STATE)
                .transition(()->intake.getFlipAnalog()<1.87, AutoStates.RETRACTSUB1)
                .transition(()->sampleMachine.getState()== SampleStates.RETRACT, AutoStates.TOSCORESUB1)
                .transitionTimed(1, AutoStates.STRAFE1)

                .state(AutoStates.RETRACTSUB1)
                .onEnter(()->{
                    if (sampleMachine.getState()== SampleStates.EXTEND){
                        sampleMachine.setState(SampleStates.IDLE);
                        intake.setTargetPos(100);
                        intake.setIntakeFlip(0.43);
                        intake.setIntakePower(-0.1);
                    }
                })
                .transitionTimed(0.3, AutoStates.INTAKESUB1)
                .transition(()->sampleMachine.getState()== SampleStates.RETRACT, AutoStates.TOSCORESUB1)

                .state(AutoStates.STRAFE1)
                .onEnter(()->{
                    if (count==0){
                        follower.followPath(subtostrafe);
                    } else if (count==1) {
                        follower.followPath(sub2tostrafe2);
                    }else{
                        follower.followPath(sub3tostrafe3);
                    }
                    count++;
                })
                .transition(()->sampleMachine.getState()== SampleStates.RETRACT, AutoStates.TOSCORESUB1)

                .state(AutoStates.TOSCORESUB1)
                .onEnter(()->{
                    if (count==0){
                        follower.followPath(subtoscore);
                    } else if (count==1) {
                        follower.followPath(sub2toscore);
                    }else{
                        follower.followPath(sub3toscore);
                    }
                    count++;
                })
                .transition(()->follower.atParametricEnd())

                .state(AutoStates.WAITSUB1)
                .transitionTimed(0.1)

                .state(AutoStates.OPENCLAWSUB1)
                .onEnter(()->scorePressed=true)
                .transition(()->sampleMachine.getState()== SampleStates.LOWERLIFT, AutoStates.TOSUB1)

               // .state(AutoStates.DONE)
                //.onEnter(()->telemetry.addLine("Done"))
                .build();
        telemetry.addData("zero power accel multiplier", FollowerConstants.zeroPowerAccelerationMultiplier);
        telemetry.update();
        outtake.closeClaw();
        while (opModeInInit()) {
            if (gamepad1.a) {
                allianceColor = Intake.SampleColor.BLUE;
                gamepad1.setLedColor(0, 0, 1, 1000);
            }
            if (gamepad1.b) {
                allianceColor = Intake.SampleColor.RED;
                gamepad1.setLedColor(1, 0, 0, 1000);
            }
            telemetry.addData("Alliance Color", allianceColor.toString());
            telemetry.update();
        }
        waitForStart();
        autoMachine.start();
        sampleMachine.start();
        groundSampleMachine.start();
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
            groundSampleMachine.update();
            follower.update();
            follower.telemetryDebug(telemetry);
            intake.update();
            outtake.update();
            System.out.println(follower.getPose().toString());
            telemetry.addData("Path State", autoMachine.getState());
            telemetry.addData("Sample State", sampleMachine.getState());

            telemetry.addData("Position", follower.getPose().toString());
            telemetry.update();
        }
    }
    private double waitTime(boolean k){
        if (k){
            return 0.2;
        }else{
            return 0.4;
        }
    }
}
