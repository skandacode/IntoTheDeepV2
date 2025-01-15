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
import subsystems.Intake;
import subsystems.Outtake;

@Config
@TeleOp
public class AutomatedTeleop extends LinearOpMode {
    Drivetrain drive;
    Outtake outtake;
    Intake intake;
    public enum SampleStates {
        IDLE, EXTEND, SENSORWAIT, SENSE, RETRACT, OPENCOVER, WAIT, CLOSE, LIFT, PARTIALFLIP, SCORE, OPEN, LOWERLIFT, EJECTFLIP, EJECTLIDOPEN
    }
    public enum SpecimenScoreStates {IDLE, INTAKEPOS, INTAKE, CLOSE_CLAW, HOLD, SCORE, OPENCLAW, RETRACT}

    public static Intake.SampleColor allianceColor= Intake.SampleColor.BLUE;
    Intake.SampleColor currentSense= Intake.SampleColor.NONE;


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

        drive = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        StateMachine sampleMachine = new StateMachineBuilder()
                .state(SampleStates.IDLE)
                .onEnter(() -> {
                    intake.transferPos();
                    intake.setIntakePower(0);
                })
                .loop(()->{
                    intake.setTargetPos((int)(1000*gamepad1.left_trigger));
                })
                .transition(() -> gamepad1.right_bumper && gamepad1.left_trigger>0.1)
                .state(SampleStates.EXTEND)
                .onEnter(()->intake.intakePos())
                .loop(()->{
                    if (gamepad1.options){
                        intake.setIntakePower(-1);
                    }else{
                        intake.setIntakePower(1);
                    }
                    intake.setTargetPos((int)(1000*gamepad1.left_trigger));
                })
                .transition(()->intake.isSampleIntaked())
                .transition(()->!gamepad1.right_bumper, SampleStates.IDLE)
                .transition(()->gamepad1.left_trigger<0.1, SampleStates.IDLE)
                .state(SampleStates.SENSORWAIT)
                .onEnter(()->intake.intakePos())
                .transitionTimed(0.05)
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
                    intake.setCover(true);
                    outtake.transferPos();
                })
                .transitionTimed(0.05)
                .state(SampleStates.OPENCOVER)
                .onEnter(() -> {
                    intake.setCover(false);
                    intake.setIntakePower(0.1);
                    outtake.openClaw();
                })
                .transition(()-> intake.isRetracted())

                .state(SampleStates.WAIT)
                .onEnter(() -> intake.setIntakePower(0.4))
                .transitionTimed(0.4)

                .state(SampleStates.CLOSE)
                .onEnter(() -> {
                    outtake.closeClaw();
                    intake.setIntakePower(0.4);
                })
                .transitionTimed(0.15)

                .state(SampleStates.LIFT)
                .onEnter(() -> {
                    outtake.setTargetPos(1000);
                })
                .transitionTimed(0.3)

                .state(SampleStates.PARTIALFLIP)
                .onEnter(()->outtake.partialSampleFlip())
                .loop(() -> {
                    if (gamepad1.right_trigger > 0.3) {
                        outtake.sampleScore();
                        outtake.setTargetPos(50);
                    }
                })
                .transition(()->gamepad1.left_bumper && outtake.getSetPoint()==50, SampleStates.OPEN)
                .transition(()->outtake.getCachedPos()>900)

                .state(SampleStates.SCORE)
                .onEnter(()->outtake.sampleScore())
                .loop(() -> {
                    if (gamepad1.left_trigger > 0.3) {
                        outtake.setTargetPos(50);
                    }
                })
                .transition(() -> gamepad1.left_bumper)
                .state(SampleStates.OPEN)
                .onEnter(() -> outtake.openClaw())
                .transitionTimed(0.5)
                .transition(()->gamepad1.left_trigger>0.1 || (gamepad1.left_stick_y<-0.8 && !gamepad1.right_bumper))
                .onExit(() -> {
                    outtake.setTargetPos(0);
                    outtake.transferPos();
                })
                .state(SampleStates.LOWERLIFT)
                .transition(()->gamepad1.left_trigger>0.3, SampleStates.IDLE)
                .transition(() -> outtake.isRetracted(), SampleStates.IDLE)
                .build();
        StateMachine specimenScorer = new StateMachineBuilder()
                .state(SpecimenScoreStates.IDLE)
                .transition(() -> gamepad1.dpad_up)
                .state(SpecimenScoreStates.INTAKEPOS)
                .onEnter(() -> {
                    outtake.sampleScore();
                    outtake.closeClaw();
                    outtake.setTargetPos(100);
                })
                .transitionTimed(0.7)
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
                .state(SpecimenScoreStates.SCORE)
                .onEnter(() -> outtake.specScore())
                .transitionTimed(0.5)
                .state(SpecimenScoreStates.OPENCLAW)
                .onEnter(() -> outtake.openClaw())
                .transitionTimed(1)
                .state(SpecimenScoreStates.RETRACT)
                .onEnter(() -> outtake.transferPos())
                .transition(() -> (outtake.isRetracted() || gamepad1.dpad_up), SpecimenScoreStates.IDLE)
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
        drive.setPtoEngaged(false);
        outtake.transferPos();
        sampleMachine.start();
        specimenScorer.start();
        long prevLoop = System.nanoTime();
        while (opModeIsActive()) {
            if (!(controlhub==null)) {
                controlhub.clearBulkCache();
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
            if (gamepad1.x) {
                if (sampleMachine.getState() == SampleStates.EXTEND) {
                    sampleMachine.setState(SampleStates.IDLE);
                    intake.transferPos();
                    intake.setIntakePower(0);
                }
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

            //telemetry.addData("Intake color", Arrays.toString(intake.getRawSensorValues()));
            //telemetry.addData("Intake distance", intake.getDistance());

            telemetry.addData("Outtake Pos", outtake.getCachedPos());
            telemetry.addData("Extendo Pos", intake.getCachedExtendoPos());

            long currLoop = System.nanoTime();
            telemetry.addData("Ms per loop", (currLoop - prevLoop) / 1000000);
            prevLoop = currLoop;

            telemetry.update();
        }
    }
}
