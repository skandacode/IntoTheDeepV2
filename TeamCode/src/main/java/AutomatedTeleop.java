import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import java.util.List;

import subsystems.Drivetrain;
import subsystems.Hang;
import subsystems.Intake;
import subsystems.Outtake;

@Config
@TeleOp
public class AutomatedTeleop extends LinearOpMode {
    Drivetrain drive;
    Outtake outtake;
    Intake intake;
    public enum SampleStates {
        IDLE, EXTEND, SENSORWAIT, SENSE, RETRACT, OPENCOVER, WAIT, CLOSE, LIFT, FULLYCLOSE, PARTIALFLIP, SCORE, AUTOWAIT, OPEN, LOWERLIFT, EJECTFLIP, EJECTLIDOPEN
    }
    public enum SpecimenScoreStates {IDLE, C, INTAKEPOS, INTAKE, CLOSE_CLAW, HOLD, SCORE, OPENCLAW, CLOSEBEFORERETRACT, RESET, RETRACT}
    enum hangStates{
        LIFTOUTTAKE, READY, PULLDOWN1, PULLDOWN2
    }
    public static Intake.SampleColor allianceColor= Intake.SampleColor.BLUE;
    Intake.SampleColor currentSense= Intake.SampleColor.NONE;
    public static int hangPos=100;
    public static int maxExtend=480;
    public static boolean lowBucket=false;
    public static int lowBucketPos=690;

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
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new Drivetrain(hardwareMap, telemetry,  FtcDashboard.getInstance());
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        StateMachine sampleMachine = new StateMachineBuilder()
                .state(SampleStates.IDLE)
                .onEnter(() -> {
                    intake.transferPos();
                    intake.setIntakePower(0);
                })
                .loop(()->{
                    intake.setTargetPos((int)(maxExtend*gamepad1.left_trigger));
                })
                .transition(() -> gamepad1.right_bumper && gamepad1.left_trigger>0.1 && intake.getCachedExtendoPos()>100)
                .state(SampleStates.EXTEND)
                .onEnter(()->intake.intakePos())
                .loop(()->{
                    if (gamepad1.options){
                        intake.setIntakePower(-1);
                    }else{
                        intake.setIntakePower(1);
                    }
                    intake.setTargetPos((int)(maxExtend*gamepad1.left_trigger));
                })
                .transition(()->intake.isSampleIntaked())
                .transition(()->!gamepad1.right_bumper, SampleStates.IDLE)
                .transition(()->gamepad1.left_trigger<0.1, SampleStates.IDLE)
                .state(SampleStates.SENSORWAIT)
                .onEnter(()->intake.intakePos())
                .transitionTimed(0.01)
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
                    intake.setIntakePower(-1);
                    intake.setCover(true);
                    outtake.transferPos();
                    outtake.openClaw();
                })
                .transitionTimed(0.025)
                .state(SampleStates.OPENCOVER)
                .onEnter(() -> {
                    intake.setCover(false);
                    intake.setIntakePower(0.1);
                    outtake.openClaw();
                })
                .transition(()-> intake.isRetracted())
                .transition(()->gamepad1.x, SampleStates.IDLE)

                .state(SampleStates.WAIT)
                .onEnter(() -> {
                    intake.setIntakePower(1);
                })
                .transitionTimed(0.2, ()->outtake.setForTransfer())

                .state(SampleStates.CLOSE)
                .onEnter(() -> {
                    outtake.partialCloseClaw();
                    intake.setIntakePower(1);
                })
                .transitionTimed(0.2)

                .state(SampleStates.LIFT)
                .onEnter(() -> {
                    if (lowBucket){
                        outtake.setTargetPos(lowBucketPos);
                    }else{
                        outtake.setTargetPos(1280);
                    }
                    intake.setIntakePower(-0.5);
                })
                .transition(()->gamepad1.left_trigger>0.3)//drop to human player
                .transitionTimed(0.35)
                .onExit(()->outtake.closeClaw())

                .state(SampleStates.FULLYCLOSE)
                .onEnter(()->outtake.closeClaw())
                .transition(()->gamepad1.left_trigger>0.3)//drop to human player
                .transitionTimed(0.05)

                .state(SampleStates.PARTIALFLIP)
                .onEnter(()->{
                    outtake.partialSampleFlip();
                    if (lowBucket){
                        outtake.setTargetPos(lowBucketPos);
                    }
                })
                .transition(()->gamepad1.left_trigger>0.3)
                .transition(()->(outtake.getCachedPos()>900 && !lowBucket) || (outtake.getCachedPos()>lowBucketPos-70 && lowBucket))

                .state(SampleStates.SCORE)
                .onEnter(()->{
                    outtake.sampleScore();
                    if (lowBucket){
                        outtake.setTargetPos(lowBucketPos);
                    }
                })
                .loop(() -> {
                    if (gamepad1.left_trigger > 0.3) {
                        outtake.setTargetPos(50);
                    }
                    if (gamepad1.dpad_right) {
                        outtake.sampleFlat();
                    }
                    if (gamepad1.dpad_left) {
                        outtake.sampleRegular();
                    }
                })
                .transition(() -> gamepad1.left_bumper)
                .state(SampleStates.OPEN)
                .onEnter(() -> outtake.openClaw())
                .transitionTimed(0.5)
                .transition(()->gamepad1.left_trigger>0.1 || (gamepad1.left_stick_y<-0.9 && !gamepad1.right_bumper))
                .onExit(() -> {
                    outtake.setTargetPos(0);
                    outtake.transferPos();
                })
                .state(SampleStates.LOWERLIFT)
                .transitionTimed(0.4, SampleStates.IDLE)
                .onExit(()->outtake.openClaw())
                .build();
        StateMachine specimenScorer = new StateMachineBuilder()
                .state(SpecimenScoreStates.IDLE)
                .transition(() -> gamepad1.dpad_up && outtake.getSetPoint()==0)
                .state(SpecimenScoreStates.C)
                .onEnter(()->{
                    outtake.closeClaw();
                    outtake.setTargetPos(200);
                })
                .transitionTimed(0.15)

                .state(SpecimenScoreStates.INTAKEPOS)
                .onEnter(() -> {
                    outtake.specGrab();
                    outtake.setTargetPos(200);
                })
                .transitionTimed(0.6)
                .state(SpecimenScoreStates.INTAKE)
                .onEnter(() -> {
                    outtake.setTargetPos(0);
                    outtake.openClawWide();
                })
                .transition(() -> gamepad1.dpad_down)
                .state(SpecimenScoreStates.CLOSE_CLAW)
                .onEnter(() -> outtake.closeClaw())
                .transitionTimed(0.3)
                .state(SpecimenScoreStates.HOLD)
                .onEnter(() -> outtake.specHold())
                .transition(() -> (outtake.atTarget() && gamepad1.left_bumper))
                .state(SpecimenScoreStates.OPENCLAW)
                .onEnter(() -> outtake.openClaw())
                .transitionTimed(0.3)
                .state(SpecimenScoreStates.CLOSEBEFORERETRACT)
                .onEnter(() -> outtake.closeClaw())
                .transition(()->gamepad1.left_bumper, SpecimenScoreStates.C, ()->{
                    outtake.specGrab();
                    outtake.setTargetPos(0);
                })
                .transitionTimed(0.1)
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
                .transition(() -> (outtake.isRetracted() || gamepad1.dpad_up), SpecimenScoreStates.IDLE)
                .onExit(()->outtake.openClaw())
                .build();
        while (opModeInInit()){
            if (gamepad1.a){
                allianceColor = Intake.SampleColor.BLUE;
                gamepad1.setLedColor(0, 0, 1, 1000);
            }
            if (gamepad1.b){
                allianceColor = Intake.SampleColor.RED;
                gamepad1.setLedColor(1, 0, 0, 1000);
            }
            telemetry.addData("Alliance Color", allianceColor.toString());
            telemetry.update();
        }
        waitForStart();
        intake.setCover(false);
        drive.setPtoEngaged(false);
        drive.setLatchPos(Hang.LatchPositions.EXTENDED);
        intake.transferPos();
        outtake.transferPos();
        outtake.openClaw();
        sampleMachine.start();
        specimenScorer.start();
        long prevLoop = System.nanoTime();
        while (opModeIsActive() && !gamepad1.left_stick_button) {
            if (!(controlhub==null)) {
                controlhub.clearBulkCache();
                telemetry.addLine("bulk reading only chub");
            }else{
                for (LynxModule hub:allHubs){
                    hub.clearBulkCache();
                }
            }
            if (!gamepad1.right_bumper){
                drive.setWeightedPowers(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, 0);
            }else{
                drive.setWeightedPowers(-gamepad1.left_stick_y/5, -gamepad1.left_stick_x/5, -gamepad1.right_stick_x/5, 0);
            }

            if (gamepad1.touchpad && sampleMachine.getState()== SampleStates.IDLE && specimenScorer.getState()== SpecimenScoreStates.IDLE){
                intake.setIntakePower(0.5);
                intake.setExtendoPower(-0.3);
                sampleMachine.setState(SampleStates.WAIT);
            }
            sampleMachine.update();
            specimenScorer.update();
            intake.update();
            outtake.update();
            if (gamepad1.share){
                telemetry.addData("intake distance", intake.getDistance());
                telemetry.addData("intake color", intake.getColor());
            }
            telemetry.addData("gamepad strafe", gamepad1.left_stick_x);
            telemetry.addData("alliance color", allianceColor.toString());

            telemetry.addData("intake retracted", intake.isRetracted());
            telemetry.addData("outtake retracted", outtake.isRetracted());

            telemetry.addData("State sample", sampleMachine.getState());
            telemetry.addData("Specimen sample", specimenScorer.getState());

            telemetry.addData("Outtake Pos", outtake.getCachedPos());
            telemetry.addData("Extendo Pos", intake.getCachedExtendoPos());
            telemetry.addData("Intake target pos", intake.getTarget());
            telemetry.addData("Outtake target pos", outtake.getSetPoint());
            telemetry.addData("intake end direct", intake.limitSwitch.isPressed());
            telemetry.addData("Outtake analog", outtake.getFlipAnalog());
            if (gamepad2.a){
                lowBucket=true;
                gamepad1.rumble(1000);
            }
            if (gamepad2.b){
                lowBucket=false;
            }
            telemetry.addData("Low Bucket", lowBucket);
            long currLoop = System.nanoTime();
            telemetry.addData("Ms per loop", (currLoop - prevLoop) / 1000000);
            prevLoop = currLoop;

            telemetry.update();
        }
        if (opModeIsActive()){
            sampleMachine.stop();
            specimenScorer.stop();
            StateMachine hangMachine = new StateMachineBuilder()
                    .state(hangStates.LIFTOUTTAKE)
                    .onEnter(()->outtake.setTargetPos(1050))
                    .loop(()->outtake.update())
                    .transitionTimed(0.5)

                    .state(hangStates.READY)
                    .onEnter(()->{
                        intake.setIntakeFlip(0.3);
                        intake.setIntakePower(0);
                        outtake.partialSampleFlip();
                        outtake.setTargetPos(1050);
                    })
                    .loop(()->{
                        if (!gamepad1.right_bumper){
                            drive.setWeightedPowers(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, 0);
                        }else{
                            drive.setWeightedPowers(-gamepad1.left_stick_y/5, -gamepad1.left_stick_x/5, -gamepad1.right_stick_x/5, 0);
                        }
                        outtake.update();
                    })
                    .transition(()-> gamepad1.right_stick_button)

                    .state(hangStates.PULLDOWN1)
                    .onEnter(()->{
                        drive.setLatchPos(Hang.LatchPositions.PULLDOWN);
                        drive.setPtoEngaged(true);
                    })
                    .loop(()->outtake.update())
                    .transitionTimed(4)

                    .state(hangStates.PULLDOWN2)
                    .onEnter(()->outtake.setMotors(0))
                    .loop(()->{
                        if (outtake.getOuttakePosition()>hangPos){
                            drive.setWeightedPowers(0, 0, 0, -1);
                        }else{
                            drive.setWeightedPowers(0, 0, 0, -0.2);
                        }
                        System.out.println(outtake.getOuttakePosition());
                    })
                    .build();

            hangMachine.start();
            while (opModeIsActive()){
                if (!(controlhub==null)) {
                    controlhub.clearBulkCache();
                }else{
                    for (LynxModule hub:allHubs){
                        hub.clearBulkCache();
                    }
                }
                hangMachine.update();
                intake.update();
                telemetry.addData("Outtake Pos", outtake.getCachedPos());

                telemetry.update();
            }
        }
    }
}
