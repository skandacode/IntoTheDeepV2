package currentAutos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
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

@Autonomous(preselectTeleOp = "AutomatedTeleop")
public class SampleAutoPedro8Maybe extends LinearOpMode {

    private Follower follower;
    Hang hang;
    Intake intake;
    Outtake outtake;
    StateMachine autoMachine;


    private final Pose subPose = new Pose(-15.5, -3, Math.toRadians(0));
    private final Pose substrafe = new Pose(-17.7, -3, Math.toRadians(15));
    private final Pose subPose2 = new Pose(-15.5, -6, Math.toRadians(5));
    private final Pose substrafe2 = new Pose(-17.7, -4, Math.toRadians(-10));
    private final Pose subPose3 = new Pose(-15.5, -8, Math.toRadians(5));
    private final Pose substrafe3 = new Pose(-17.7, 2, Math.toRadians(15));
    private final Pose presubPose = new Pose(-30, -9, Math.toRadians(0));


    private final Pose presubPoseto = new Pose(-50, -9, Math.toRadians(0));



    private final Pose scorePose = new Pose(-57.5, -53.5, Math.toRadians(75));
    private final Pose scorePosepreload = new Pose(-56, -55.4, Math.toRadians(67));
    private final Pose scorePosesub = new Pose(-58.5, -56.5, Math.toRadians(60));
    private final Pose startPose = new Pose(-36, -61.5, Math.toRadians(90));
    private final Pose sample1 = new Pose(-52, -49.5, Math.toRadians(70));
    private final Pose sample2 = new Pose(-58.5, -49, Math.toRadians(80));
    private final Pose sample3 = new Pose(-55, -48, Math.toRadians(113));

    public enum SampleStates {
        IDLE, EXTEND, SENSORWAIT, SENSE, RETRACT, PULSEEJECT, OPENCOVER, WAIT, CLOSE, LIFT, FULLYCLOSE, PARTIALFLIP, SCORE, AUTOWAIT, OPEN, LOWERLIFT, EJECTFLIP, EJECTLIDOPEN
    }
    public static Intake.SampleColor allianceColor= Intake.SampleColor.BLUE;
    Intake.SampleColor currentSense= Intake.SampleColor.NONE;
    public static int maxExtend=420;

    private boolean extendPressed=false;
    private boolean scorePressed=false;
    private boolean known=true;
    private int count=0;

    enum AutoStates{PRELOAD, WAIT, OPENCLAW1,
        TOSAMPLE1, INTAKE1, SCORESAMPLE1,WAIT2,OPENCLAW2,
        TOSAMPLE2, INTAKE2,SCORESAMPLE2, WAIT3,OPENCLAW3,
        TOSAMPLE3, INTAKE3,SCORESAMPLE3,WAIT4,OPENCLAW4,WAITTORETRACT,
        TOSUB1, EXTENDSUB1, DROPEJECT, INTAKESUB1, CHOOSE_STATE, RETRACTSUB1, PULSEREVERSE, STRAFE1, TOSCORESUB1, WAITSUB1, EXTENDTOSCORE1, OPENCLAWSUB1,
        DONE}

    @Override
    public void runOpMode() throws InterruptedException {
        extendPressed=false;
        scorePressed=false;
        known=true;
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
                    intake.setIntakePower(1);
                })
                .transition(()->intake.isSampleIntaked())
                .transitionTimed(1)

                .state(SampleStates.SENSORWAIT)
                .onEnter(()->intake.intakePos())
                .transitionTimed(0.01)
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
                .transitionTimed(0.5, SampleStates.EXTEND)

                .state(SampleStates.RETRACT)
                .onEnter(()->{
                    intake.transferPos();
                    intake.setCover(true);
                    outtake.transferPos();
                    outtake.openClaw();
                })
                .transitionTimed(0.01)

                .state(SampleStates.PULSEEJECT)
                .onEnter(()->{
                    intake.setCover(false);
                    outtake.openClaw();
                    if (!known){
                        intake.setIntakePower(-1);
                    }
                })
                .transitionTimed(0.022)

                .state(SampleStates.OPENCOVER)
                .onEnter(() -> {
                    intake.setIntakePower(0.25);
                })
                .transition(()-> intake.isRetracted())
                .transitionTimed(1)

                .state(SampleStates.WAIT)
                .onEnter(() -> {
                    intake.setIntakePower(1);
                })
                .transitionTimed(waitTime(known))
                .onExit(()->outtake.setForTransfer())


                .state(SampleStates.CLOSE)
                .onEnter(() -> {
                    outtake.closeClaw();
                    intake.setIntakePower(1);
                })
                .transitionTimed(0.2)

                .state(SampleStates.LIFT)
                .onEnter(() -> {
                    outtake.setTargetPos(1250);
                    if (known){
                        intake.intakePos(maxExtend-70);
                        intake.setIntakePower(0);
                    }else{
                        intake.setIntakePower(-0.5);
                    }
                    if (known){
                        intake.setTargetPos(250);
                    }
                })
                .transitionTimed(0.35)
                .onExit(()->outtake.closeClaw())

                .state(SampleStates.FULLYCLOSE)
                .onEnter(()->outtake.closeClaw())
                .transitionTimed(0.1)

                .state(SampleStates.PARTIALFLIP)
                .onEnter(()->{
                    outtake.partialSampleFlip();
                })
                .transition(()->outtake.getCachedPos()>900)

                .state(SampleStates.SCORE)
                .onEnter(()->{
                    outtake.sampleScore();
                })
                .transition(() -> scorePressed)

                .state(SampleStates.AUTOWAIT)
                .transitionTimed(waitTime(known))

                .state(SampleStates.OPEN)
                .onEnter(() -> {
                    outtake.openClaw();
                    scorePressed=false;
                })
                .transitionTimed(0.2)
                .onExit(() -> {
                    outtake.setTargetPos(0);
                    outtake.transferPos();
                })
                .state(SampleStates.LOWERLIFT)
                .transitionTimed(0.4, SampleStates.IDLE, ()->outtake.openClaw())
                .transition(()->extendPressed, SampleStates.IDLE)
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
                .addPath(new BezierCurve(new Point(scorePose), new Point(presubPoseto),new Point(subPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), subPose.getHeading())
                .setZeroPowerAccelerationMultiplier(4)
                .build();
        PathChain subtoscore = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(subPose), new Point(presubPose),new Point(scorePosesub)))
                .setLinearHeadingInterpolation(subPose.getHeading(), scorePosesub.getHeading())
                .setZeroPowerAccelerationMultiplier(4.5)
                .build();
        PathChain subtostrafe = follower.pathBuilder()
                .addPath(new BezierLine(new Point(subPose), new Point(substrafe)))
                .setLinearHeadingInterpolation(subPose.getHeading(), substrafe.getHeading())
                .setZeroPowerAccelerationMultiplier(4.5)
                .build();
        PathChain scoretosub2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(presubPoseto),new Point(subPose2)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), subPose2.getHeading())
                .setZeroPowerAccelerationMultiplier(4.5)
                .build();
        PathChain sub2toscore = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(subPose2), new Point(presubPose),new Point(scorePosesub)))
                .setLinearHeadingInterpolation(subPose2.getHeading(), scorePosesub.getHeading())
                .setZeroPowerAccelerationMultiplier(4.5)
                .build();
        PathChain sub2tostrafe2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(subPose2), new Point(substrafe2)))
                .setLinearHeadingInterpolation(subPose2.getHeading(), substrafe2.getHeading())
                .setZeroPowerAccelerationMultiplier(4.5)
                .build();
        PathChain scoretosub3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(presubPoseto),new Point(subPose3)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), subPose3.getHeading())
                .setZeroPowerAccelerationMultiplier(4.5)
                .build();
        PathChain sub3toscore = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(subPose3), new Point(presubPose),new Point(scorePosesub)))
                .setLinearHeadingInterpolation(subPose3.getHeading(), scorePosesub.getHeading())
                .setZeroPowerAccelerationMultiplier(4.5)
                .build();
        PathChain sub3tostrafe3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(subPose3), new Point(substrafe3)))
                .setLinearHeadingInterpolation(subPose3.getHeading(), substrafe3.getHeading())
                .setZeroPowerAccelerationMultiplier(4.5)
                .build();




        autoMachine=new StateMachineBuilder()
                .state(AutoStates.PRELOAD)
                .onEnter(()->{
                    follower.followPath(scorePreload, true);
                    sampleMachine.setState(SampleStates.LIFT);
                })
                .transitionTimed(1)
                .state(AutoStates.OPENCLAW1)
                .onEnter(()->{
                    scorePressed=true;
                    outtake.setTargetPos((int)outtake.getSetPoint()-30);
                })
                .transitionTimed(0.15)

                .state(AutoStates.TOSAMPLE1)
                .onEnter(()->{
                    follower.followPath(scoretoSamp1, true);
                })
                .transitionTimed(0.01)
                .state(AutoStates.INTAKE1)
                .onEnter(()->{
                    extendPressed=true;
                })
                .transition(()->sampleMachine.getState()== SampleStates.RETRACT)
                .transitionTimed(1, AutoStates.INTAKE1, ()->{
                    intake.setIntakePower(-1);
                    extendPressed=false;
                })

                .state(AutoStates.SCORESAMPLE1)
                .onEnter(()->follower.followPath(samp1toScore, true))
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.35)
                .state(AutoStates.WAIT2)
                .transitionTimed(0.1)
                .state(AutoStates.OPENCLAW2)
                .onEnter(()->{
                    scorePressed=true;
                })
                .onEnter(()->scorePressed=true)
                .transition(()->sampleMachine.getState()== SampleStates.LOWERLIFT)

                .state(AutoStates.TOSAMPLE2)
                .onEnter(()->{
                    follower.followPath(scoretoSamp2, true);
                })
                .transitionTimed(0.05)
                .state(AutoStates.INTAKE2)
                .onEnter(()->{
                    extendPressed=true;
                })
                .transition(()->sampleMachine.getState()== SampleStates.RETRACT)
                .transitionTimed(1, AutoStates.INTAKE2, ()->{
                    intake.setIntakePower(-1);
                    extendPressed=false;
                })

                .state(AutoStates.SCORESAMPLE2)
                .onEnter(()->follower.followPath(samp2toScore, true))
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.4)
                .state(AutoStates.WAIT3)
                .transitionTimed(0.1)
                .state(AutoStates.OPENCLAW3)
                .onEnter(()->{
                    scorePressed=true;
                })
                .transition(()->sampleMachine.getState()== SampleStates.LOWERLIFT)

                .state(AutoStates.TOSAMPLE3)
                .onEnter(()->{
                    follower.followPath(scoretoSamp3, true);
                })
                .transitionTimed(0.4)
                .state(AutoStates.INTAKE3)
                .onEnter(()->{
                    extendPressed=true;
                })
                .transition(()->sampleMachine.getState()== SampleStates.RETRACT)
                .transitionTimed(2.3, AutoStates.INTAKE3, ()->{
                    intake.setIntakePower(-1);
                    extendPressed=false;
                })
                .state(AutoStates.SCORESAMPLE3)
                .onEnter(()->{
                    follower.followPath(samp3toScore, true);
                })
                .loop(()->{
                    intake.transferPos();
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.3)
                .state(AutoStates.WAIT4)
                .loop(()->{
                    intake.transferPos();
                })
                .transitionTimed(0.21)
                .state(AutoStates.OPENCLAW4)
                .onEnter(()->{
                    scorePressed=true;
                    known=false;
                    hang.setLatchPos(Hang.LatchPositions.TOUCHBAR);
                })
                .loop(()->{
                    intake.transferPos();
                })
                .transition(()->sampleMachine.getState()==SampleStates.OPEN)
                .state(AutoStates.WAITTORETRACT)
                .loop(()->{
                    intake.transferPos();
                })
                .transitionTimed(0.15)

                .state(AutoStates.TOSUB1)
                .onEnter(()->{
                    if (count==0){
                        follower.followPath(scoretosub);
                        System.out.println("scoretosub");
                    }else if (count==1){
                        follower.followPath(scoretosub2);
                        System.out.println("scoretosub2");
                    }else{
                        follower.followPath(scoretosub3);
                        System.out.println("scoretosub3");
                    }
                })
                .transitionTimed(0.1)

                .state(AutoStates.EXTENDSUB1)
                .loop(()->intake.setTargetPos(100))
                .transitionTimed(1.75)

                .state(AutoStates.DROPEJECT)
                .onEnter(()->{
                    intake.intakePos(150);
                    intake.setIntakePower(-1);
                })
                .transitionTimed(0.2)

                .state(AutoStates.INTAKESUB1)
                .onEnter(()->extendPressed=true)
                .transition(()->sampleMachine.getState()== SampleStates.RETRACT, AutoStates.TOSCORESUB1)
                .transitionTimed(0.7)
                .transition(()->intake.isJammed(), AutoStates.PULSEREVERSE)

                .state(AutoStates.CHOOSE_STATE)
                .transition(()->intake.getFlipAnalog()<Intake.dropThresh && sampleMachine.getState()==SampleStates.EXTEND, AutoStates.RETRACTSUB1, () -> System.out.println("On top of sample"))
                .transition(()->sampleMachine.getState()== SampleStates.RETRACT, AutoStates.TOSCORESUB1, ()->System.out.println("Sample detected"))
                .transition(()->intake.isJammed(), AutoStates.PULSEREVERSE, () -> System.out.println("Intake jammed"))
                .transitionTimed(0.01, AutoStates.STRAFE1)

                .state(AutoStates.RETRACTSUB1)
                .onEnter(()->{
                    if (sampleMachine.getState()== SampleStates.EXTEND){
                        sampleMachine.setState(SampleStates.IDLE);
                        intake.setTargetPos(100);
                        intake.setIntakeFlip(0.43);
                        extendPressed=false;
                        System.out.println("retracted");
                    }

                })
                .transitionTimed(0.4, AutoStates.INTAKESUB1, ()->{
                    System.out.println("Reextended");
                    intake.setTargetPos(430);
                })
                .transition(()->sampleMachine.getState()== SampleStates.RETRACT, AutoStates.TOSCORESUB1)

                .state(AutoStates.PULSEREVERSE)
                .onEnter(()->{
                    intake.setIntakePower(-1);
                    System.out.println("Ejecting");
                })
                .transitionTimed(0.1, AutoStates.INTAKESUB1, ()->intake.setIntakePower(1))

                .state(AutoStates.STRAFE1)
                .onEnter(()->{
                    intake.setTargetPos(200);
                    if (count==0){
                        follower.followPath(subtostrafe);
                        System.out.println("subtostrafe");
                    } else if (count==1) {
                        follower.followPath(sub2tostrafe2);
                        System.out.println("sub2tostrafe2");
                    }else{
                        follower.followPath(sub3tostrafe3);
                        System.out.println("sub3tostrafe3");
                    }
                    count++;
                })
                .transition(()->sampleMachine.getState()== SampleStates.RETRACT, AutoStates.TOSCORESUB1, ()->intake.setTargetPos(420))
                .transitionTimed(1.2, AutoStates.INTAKESUB1)

                .state(AutoStates.TOSCORESUB1)
                .onEnter(()->{
                    PathChain dynamicPath = follower.pathBuilder()
                            .addPath(new BezierCurve(new Point(follower.getPose()), new Point(presubPose),new Point(scorePosesub)))
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), scorePosesub.getHeading())
                            .setZeroPowerAccelerationMultiplier(2.5)
                            .build();
                    follower.followPath(dynamicPath);
                    extendPressed=false;
                })
                .loop(()->{
                    if (outtake.getSetPoint() != 0){
                        intake.setTargetPos(400);
                    }
                })
                .transitionTimed(2.6)
                .transition(()->follower.atParametricEnd())

                .state(AutoStates.OPENCLAWSUB1)
                .onEnter(()->{
                    scorePressed=true;
                    outtake.setTargetPos((int)outtake.getSetPoint()-30);
                })
                .transitionTimed(0.2, AutoStates.TOSUB1)
                .transition(()->sampleMachine.getState()== SampleStates.LOWERLIFT, AutoStates.TOSUB1)
                .build();
        outtake.closeClaw();
        outtake.setMotors(-0.1);
        outtake.setTargetPos(0);
        intake.setIntakeFlip(0.3);
        hang.setLatchPos(Hang.LatchPositions.FULLY_RETRACTED);
        extendPressed=false;
        scorePressed=false;
        known=true;
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
            if (gamepad1.a) {
                allianceColor = Intake.SampleColor.BLUE;
                gamepad1.setLedColor(0, 0, 1, 1000);
            }
            if (gamepad1.b) {
                allianceColor = Intake.SampleColor.RED;
                gamepad1.setLedColor(1, 0, 0, 1000);
            }
            telemetry.addData("Pose", follower.getPose().toString());
            telemetry.addData("Alliance Color", allianceColor.toString());
            telemetry.addData("Extend Pressed", extendPressed);
            telemetry.addData("Score Pressed", scorePressed);

            telemetry.update();
        }
        waitForStart();
        autoMachine.start();
        sampleMachine.start();
        long prevLoop = System.nanoTime();
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
            follower.update();
            intake.update();
            outtake.update();
            //System.out.println(follower.getPose().toString());
            telemetry.addData("Path State", autoMachine.getState());
            telemetry.addData("Sample State", sampleMachine.getState());

            telemetry.addData("Position", follower.getPose().toString());
            Pose currPose=follower.getPose();
            double j=normalize(Math.toDegrees(prevPose.getHeading()),Math.toDegrees(currPose.getHeading()));
            if (j>20 && j<180){
                System.out.println("PROBABLE JUMP");
                System.out.println("Prev: "+prevPose);
                System.out.println("Curr: "+currPose);

            }
            prevPose=currPose;
            long currLoop = System.nanoTime();
            telemetry.addData("Ms per loop", (currLoop - prevLoop) / 1000000);
            prevLoop = currLoop;

            //follower.telemetryDebug(telemetry);
            telemetry.addData("Intake jamming", intake.isJammed());
            telemetry.update();

        }
    }
    private double waitTime(boolean k){
        if (k){
            return 0.09;
        }else{
            return 0.65;
        }
    }
    private double normalize(double a, double b){
        a=a%180;
        b=b%180;
        double c=Math.abs(a-b);
        if (c>180){
            return c-180;
        }else{
            return c;
        }
    }
}
