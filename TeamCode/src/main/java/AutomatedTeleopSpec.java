import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import java.util.List;

import subsystems.Drivetrain;
import subsystems.Hang;
import subsystems.Intake;
import subsystems.Outtake;

@Config
@TeleOp
public class AutomatedTeleopSpec extends LinearOpMode {
    Drivetrain drive;
    Outtake outtake;
    Intake intake;
    public enum SampleStates {
        IDLE, EXTEND, SENSORWAIT, SENSE, LIFTUP, RETRACT, OPENCOVER, WAIT, CLOSE, LIFT, FULLYCLOSE, PARTIALFLIP, SCORE, AUTOWAIT, OPEN, LOWERLIFT, EJECTFLIP, EJECTLIDOPEN
    }
    public enum SpecimenScoreStates {IDLE, C, INTAKEPOS, INTAKE, CLOSE_CLAW, HOLD, SCORE, OPENCLAW, CLOSEBEFORERETRACT, RESET, RETRACT}
    enum hangStates{
        LIFTOUTTAKE, READY, PULLDOWN1, PULLDOWN2, COAST
    }
    public static Intake.SampleColor allianceColor= Intake.SampleColor.BLUE;
    Intake.SampleColor currentSense= Intake.SampleColor.NONE;
    public static int hangPos=85;
    public static int maxExtend=480;
    public static boolean lowBucket=false;
    public static int lowBucketPos=500;
    public static double currentThresh = 31;

    public static boolean overfillPos=false;

    double cachedCurrent = 0;

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
                .state(AutomatedTeleopSample.SampleStates.IDLE)
                .onEnter(() -> {
                    intake.transferPos();
                    intake.setIntakePower(0);
                })
                .loop(()->{
                    intake.setTargetPos((int)(maxExtend*gamepad1.left_trigger));
                })
                .transition(() -> gamepad1.right_bumper && gamepad1.left_trigger>0.1 && intake.getCachedExtendoPos()>50)
                .state(AutomatedTeleopSample.SampleStates.EXTEND)
                .onEnter(()->intake.intakePos())
                .loop(()->{
                    if (gamepad1.options){
                        intake.setIntakePower(-0.75);
                    }else{
                        intake.setIntakePower(1);
                    }
                    if (!gamepad1.right_bumper){
                        intake.setCover(false);//TODO:move this somewhere else
                    }
                    intake.setTargetPos((int)(maxExtend*gamepad1.left_trigger));
                })
                .transition(()->intake.isSampleIntaked())
                .transition(()->(gamepad1.left_trigger<0.1 || !gamepad1.right_bumper) && !intake.isSampleIntaked(), AutomatedTeleopSample.SampleStates.IDLE)
                .transition(()->(gamepad1.left_trigger<0.1 || !gamepad1.right_bumper) && intake.isSampleIntaked(), AutomatedTeleopSample.SampleStates.SENSORWAIT, ()->System.out.println("arka released trigger before but moving on"))

                .state(AutomatedTeleopSample.SampleStates.SENSORWAIT)
                .onEnter(()->intake.intakePos())
                .transitionTimed(0.01)
                .state(AutomatedTeleopSample.SampleStates.SENSE)
                .transition(() -> {
                    currentSense=intake.getColor();
                    return currentSense == Intake.SampleColor.YELLOW || currentSense== allianceColor;
                }, AutomatedTeleopSample.SampleStates.LIFTUP)
                .transition(()->currentSense == Intake.SampleColor.NONE, AutomatedTeleopSample.SampleStates.EXTEND)
                .transition(() -> currentSense != Intake.SampleColor.YELLOW && currentSense != allianceColor, AutomatedTeleopSample.SampleStates.EJECTPAUSE)

                .state(AutomatedTeleopSample.SampleStates.EJECTPAUSE, true)
                .onEnter(() -> {
                    intake.pauseEject();//TODO:FIX EJECT
                })
                .transitionTimed(0.2, AutomatedTeleopSample.SampleStates.EJECT)

                .state(AutomatedTeleopSample.SampleStates.EJECT, true)
                .onEnter(()->{
                    intake.eject();
                    intake.setIntakePower(0.8);
                })
                .transitionTimed(0.5, AutomatedTeleopSample.SampleStates.EXTEND)

                .state(AutomatedTeleopSample.SampleStates.LIFTUP)
                .onEnter(()->{
                    intake.liftUP();
                })
                .transitionTimed(0.1)

                .state(AutomatedTeleopSample.SampleStates.RETRACT)
                .onEnter(()->{
                    intake.transferPos();
                    outtake.transferPos();
                    outtake.openClaw();
                    intake.setIntakePower(0.1);
                })
                .transitionTimed(0.1)


                .state(AutomatedTeleopSample.SampleStates.REINTAKE)
                .onEnter(() -> {
                    intake.setIntakePower(0.75);
                    outtake.openClaw();
                })
                .transition(()-> intake.isRetracted())
                .transition(()->gamepad1.x, AutomatedTeleopSample.SampleStates.IDLE)

                .state(AutomatedTeleopSample.SampleStates.WAIT)
                .onEnter(() -> {
                    intake.setIntakePower(0.4);
                    intake.setCover(false);
                })
                .transitionTimed(0.3, ()->outtake.setForTransfer())

                .state(AutomatedTeleopSample.SampleStates.CLOSE)
                .onEnter(() -> {
                    outtake.closeClaw();
                    intake.setIntakePower(1);
                    intake.setCover(false);
                })
                .transitionTimed(0.4)

                .state(AutomatedTeleopSample.SampleStates.LIFT)
                .onEnter(() -> {
                    if (lowBucket){
                        outtake.setTargetPos(lowBucketPos);
                    }else{
                        outtake.setTargetPos(1250);
                    }
                    intake.setIntakePower(0);
                })
                .transitionTimed(0.35)
                .onExit(()->outtake.closeClaw())

                .state(AutomatedTeleopSample.SampleStates.PARTIALFLIP)
                .onEnter(()->{
                    outtake.partialSampleFlip();
                    if (lowBucket){
                        outtake.setTargetPos(lowBucketPos);
                    }
                    intake.setTargetPos(400);
                })
                .transition(()->gamepad1.right_trigger>0.3)
                .transition(()->(outtake.getCachedPos()>900 && !lowBucket) || (outtake.getCachedPos()>lowBucketPos-150 && lowBucket))

                .state(AutomatedTeleopSample.SampleStates.SCORE)
                .loop(() -> {
                    if (gamepad1.right_trigger > 0.3) {
                        outtake.sampleScore();
                        outtake.setTargetPos(50);
                    }
                    else if (lowBucket){
                        outtake.sampleScore();
                        outtake.setTargetPos(lowBucketPos);
                    }
                    else if (overfillPos){
                        outtake.sampleOverfill();

                    }else{
                        outtake.sampleScore();
                    }

                })
                .transition(() -> gamepad1.left_bumper)
                .state(AutomatedTeleopSample.SampleStates.OPEN)
                .onEnter(() -> {
                    outtake.openClaw();
                    if (gamepad1.right_trigger>0.3){
                        outtake.setTargetPos(200);
                    }
                })
                .transitionTimed(0.3)
                .transition(()->gamepad1.left_trigger>0.1 || (gamepad1.left_stick_y<-0.9 && !gamepad1.right_bumper))
                .onExit(() -> {
                    outtake.transferPos();
                    outtake.setTargetPos(0);
                })
                .state(AutomatedTeleopSample.SampleStates.LOWERLIFT)
                .transitionTimed(0.35, AutomatedTeleopSample.SampleStates.IDLE)
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
                .transitionTimed(0.1)

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
                .transition(() -> (gamepad1.left_bumper))
                .state(SpecimenScoreStates.SCORE)
                .onEnter(()->outtake.specScore())
                .transitionTimed(0.3)
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
                .transitionTimed(0.1)
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
        drive.setLatchPos(Hang.LatchPositions.PULLDOWN);
        intake.transferPos();
        outtake.transferPos();
        outtake.openClaw();

        if (outtake.getFlipAnalog()>1.7) {
            outtake.closeClaw();
            ElapsedTime time1 = new ElapsedTime();
            time1.reset();
            while (opModeIsActive() && time1.milliseconds()<500){
                if (!(controlhub==null)) {
                    controlhub.clearBulkCache();
                    telemetry.addLine("bulk reading only chub");
                }else{
                    for (LynxModule hub:allHubs){
                        hub.clearBulkCache();
                    }
                }

                telemetry.addData("outtake analog", outtake.getFlipAnalog());
                telemetry.update();
            }
        }

        outtake.transferPos();

        while (opModeIsActive() && intake.getFlipAnalog()>1.1){
            if (!(controlhub==null)) {
                controlhub.clearBulkCache();
                telemetry.addLine("bulk reading only chub");
            }else{
                for (LynxModule hub:allHubs){
                    hub.clearBulkCache();
                }
            }
            telemetry.addData("intake analog", intake.getFlipAnalog());
            telemetry.update();
        }

        while (opModeIsActive() && outtake.getFlipAnalog()>1.7){
            if (!(controlhub==null)) {
                controlhub.clearBulkCache();
                telemetry.addLine("bulk reading only chub");
            }else{
                for (LynxModule hub:allHubs){
                    hub.clearBulkCache();
                }
            }
            telemetry.addData("outtake analog", outtake.getFlipAnalog());
            telemetry.update();
        }

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
            if (gamepad2.right_trigger>0.8){
                drive.setLatchPos(Hang.LatchPositions.EXTENDED);
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
            if (gamepad1.dpad_right){
                overfillPos=true;
            }
            if (gamepad1.dpad_left){
                overfillPos=false;
            }

            intake.setSweeper(!gamepad1.dpad_right);

            telemetry.addData("Low Bucket", lowBucket);
            telemetry.addData("Overfillpos", overfillPos);

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
                    .loop(()->{
                        if (!gamepad1.right_bumper){
                            drive.setWeightedPowers(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, 0);
                        }else{
                            drive.setWeightedPowers(-gamepad1.left_stick_y/5, -gamepad1.left_stick_x/5, -gamepad1.right_stick_x/5, 0);
                        }
                        outtake.update();
                    })
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
                        drive.setWeightedPowers(0, 0, 0);
                    })
                    .loop(()->outtake.update())
                    .transitionTimed(4)

                    .state(hangStates.PULLDOWN2)
                    .onEnter(()->outtake.setMotors(0))
                    .loop(()->{
                        drive.setWeightedPowers(0, 0, 0, -1);

                        System.out.println(outtake.getOuttakePosition());
                    })
                    .transition(()->getCachedCurrent()>currentThresh || gamepad1.a)
                    .state(hangStates.COAST)
                    .onEnter(()->{
                        drive.setWeightedPowers(0, 0, 0, -0.25);

                    })
                    .loop(()->System.out.println("hi   "+outtake.getOuttakePosition()))
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
                cachedCurrent = drive.getCurrent();
                telemetry.addData("Outtake Pos", outtake.getCachedPos());
                telemetry.addData("Drive current", cachedCurrent);
                System.out.println("Is greater than thresh: "+(getCachedCurrent()>currentThresh));
                System.out.println("Drive current: "+getCachedCurrent());
                System.out.println("Thresh: "+currentThresh);


                telemetry.update();
            }
        }
    }
    public double getCachedCurrent(){
        return cachedCurrent;
    }
}
