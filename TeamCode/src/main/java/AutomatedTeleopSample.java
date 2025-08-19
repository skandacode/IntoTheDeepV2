import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.ftcontrol.panels.Panels;
import com.bylazar.ftcontrol.panels.integration.TelemetryManager;
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
public class AutomatedTeleopSample extends LinearOpMode {
    Drivetrain drive;
    Outtake outtake;
    Intake intake;
    public enum SampleStates {
        IDLE, EXTEND, SENSORWAIT, SENSE, LIFTUP, RETRACT, REINTAKE, WAIT, CLOSE, LIFT, PARTIALFLIP, SCORE, AUTOWAIT, OPEN, LOWERLIFT, EJECTPAUSE, EJECT, EJECTLIDOPEN
    }
    enum hangStates{
        LIFTOUTTAKE, READY, PULLDOWN1, PULLDOWN2, COAST
    }
    public static Intake.SampleColor allianceColor= Intake.SampleColor.BLUE;
    Intake.SampleColor currentSense= Intake.SampleColor.NONE;
    public static int hangPos=85;
    public static int maxExtend=475;
    public static boolean lowBucket=false;
    public static int lowBucketPos=500;
    public static double currentThresh = 31;

    public static boolean overfillPos=false;

    double cachedCurrent = 0;
    TelemetryManager panelsTelemetry = Panels.getTelemetry();

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
                    intake.setTargetPos((int)(maxExtend*gamepad1.left_trigger+150*gamepad1.right_trigger));
                })
                .transition(() -> gamepad1.right_bumper && gamepad1.left_trigger>0.1 && intake.getCachedExtendoPos()>50)
                .state(SampleStates.EXTEND)
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
                    intake.setTargetPos((int)(maxExtend*gamepad1.left_trigger + 150*gamepad1.right_trigger));
                })
                .transition(()->intake.isSampleIntaked())
                .transition(()->(gamepad1.left_trigger<0.1 || !gamepad1.right_bumper) && !intake.isSampleIntaked(), SampleStates.IDLE)
                .transition(()->(gamepad1.left_trigger<0.1 || !gamepad1.right_bumper) && intake.isSampleIntaked(), SampleStates.SENSORWAIT, ()->System.out.println("arka released trigger before but moving on"))

                .state(SampleStates.SENSORWAIT)
                .onEnter(()->intake.intakePos())
                .transitionTimed(0.01)
                .state(SampleStates.SENSE)
                .transition(() -> {
                    currentSense=intake.getColor();
                    return currentSense == Intake.SampleColor.YELLOW || currentSense== allianceColor;
                }, SampleStates.LIFTUP)
                .transition(()->currentSense == Intake.SampleColor.NONE, SampleStates.EXTEND)
                .transition(() -> currentSense != Intake.SampleColor.YELLOW && currentSense != allianceColor, SampleStates.EJECTPAUSE)

                .state(SampleStates.EJECTPAUSE, true)
                .onEnter(() -> {
                    intake.pauseEject();//TODO:FIX EJECT
                })
                .transitionTimed(0.2, SampleStates.EJECT)

                .state(SampleStates.EJECT, true)
                .onEnter(()->{
                    intake.eject();
                    intake.setIntakePower(0.8);
                })
                .transitionTimed(0.5, SampleStates.EXTEND)

                .state(SampleStates.LIFTUP)
                .onEnter(()->{
                    intake.liftUP();
                })
                .transitionTimed(0.1)

                .state(SampleStates.RETRACT)
                .onEnter(()->{
                    intake.transferPos();
                    outtake.transferPos();
                    outtake.openClaw();
                    intake.setIntakePower(0.5);
                })
                .transitionTimed(0.1)


                .state(SampleStates.REINTAKE)
                .onEnter(() -> {
                    intake.setIntakePower(0.75);
                    outtake.openClaw();
                })
                .transition(()-> intake.isRetracted())
                .transition(()->gamepad1.x, SampleStates.IDLE)

                .state(SampleStates.WAIT)
                .onEnter(() -> {
                    intake.setIntakePower(0.4);
                    intake.setCover(false);
                })
                .transitionTimed(0.3, ()->outtake.setForTransfer())

                .state(SampleStates.CLOSE)
                .onEnter(() -> {
                    outtake.closeClaw();
                    intake.setIntakePower(1);
                    intake.setCover(false);
                })
                .transitionTimed(0.4)

                .state(SampleStates.LIFT)
                .onEnter(() -> {
                    if (lowBucket){
                        outtake.setTargetPos(lowBucketPos);
                    }else{
                        outtake.setTargetPos((int) (1250-(50*gamepad1.right_trigger)));
                    }
                    intake.setIntakePower(0);
                })
                .transitionTimed(0.35)
                .onExit(()->outtake.closeClaw())

                .state(SampleStates.PARTIALFLIP)
                .onEnter(()->{
                    outtake.partialSampleFlip();
                    if (lowBucket){
                        outtake.setTargetPos(lowBucketPos);
                    }
                    intake.setTargetPos(400);
                })
                .transition(()->gamepad1.right_trigger>0.3)
                .transition(()->(outtake.getCachedPos()>900 && !lowBucket) || (outtake.getCachedPos()>lowBucketPos-150 && lowBucket))

                .state(SampleStates.SCORE)
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

                    }
                    else if (gamepad1.left_trigger > 0.1) {
                        outtake.sampleScore();
                        outtake.setTargetPos((int) (1050-50*gamepad1.left_trigger));
                    }
                    else{
                        outtake.sampleScore();
                    }

                })
                .transition(() -> gamepad1.left_bumper)
                .state(SampleStates.OPEN)
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
                .state(SampleStates.LOWERLIFT)
                .transitionTimed(0.35, SampleStates.IDLE)
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
            panelsTelemetry.debug("Alliance Color: "+ allianceColor);
            panelsTelemetry.update(telemetry);
        }
        waitForStart();
        intake.setCover(false);
        drive.setPtoEngaged(false);
        drive.setLatchPos(Hang.LatchPositions.PULLDOWN);
        intake.transferPos();

        if (outtake.getFlipAnalog()>1.7) {
            outtake.closeClaw();
            ElapsedTime time1 = new ElapsedTime();
            time1.reset();
            while (opModeIsActive() && time1.milliseconds()<500){
                if (!(controlhub==null)) {
                    controlhub.clearBulkCache();
                    panelsTelemetry.debug("bulk reading only chub");
                }else{
                    for (LynxModule hub:allHubs){
                        hub.clearBulkCache();
                    }
                }
                panelsTelemetry.debug("outtake analog: "+ outtake.getFlipAnalog());
                panelsTelemetry.update(telemetry);
            }
        }

        outtake.transferPos();

        while (opModeIsActive() && intake.getFlipAnalog()>1.1){
            if (!(controlhub==null)) {
                controlhub.clearBulkCache();
                panelsTelemetry.debug("bulk reading only chub");
            }else{
                for (LynxModule hub:allHubs){
                    hub.clearBulkCache();
                }
            }
            panelsTelemetry.debug("intake analog: "+ intake.getFlipAnalog());
            panelsTelemetry.update(telemetry);
        }

        while (opModeIsActive() && outtake.getFlipAnalog()>1.7){
            if (!(controlhub==null)) {
                controlhub.clearBulkCache();
                panelsTelemetry.debug("bulk reading only chub");
            }else{
                for (LynxModule hub:allHubs){
                    hub.clearBulkCache();
                }
            }
            panelsTelemetry.debug("outtake analog: "+ outtake.getFlipAnalog());
            panelsTelemetry.update(telemetry);
        }


        outtake.openClaw();

        sampleMachine.start();
        long prevLoop = System.nanoTime();
        while (opModeIsActive() && !(gamepad1.left_stick_button && intake.getTarget()==0)) {
            if (!(controlhub==null)) {
                controlhub.clearBulkCache();
                panelsTelemetry.debug("bulk reading only chub");
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

            if (gamepad1.touchpad && sampleMachine.getState()== SampleStates.IDLE){
                intake.setIntakePower(0.5);
                intake.setExtendoPower(-0.3);
                sampleMachine.setState(SampleStates.WAIT);
            }
            if (gamepad2.right_trigger>0.8){
                drive.setLatchPos(Hang.LatchPositions.EXTENDED);
            }
            sampleMachine.update();
            intake.update();
            outtake.update();
            if (gamepad1.share){
                panelsTelemetry.debug("intake distance: "+ intake.getDistance());
                panelsTelemetry.debug("intake color: "+ intake.getColor());
            }

            panelsTelemetry.debug("gamepad strafe: "+ gamepad1.left_stick_x);
            panelsTelemetry.debug("alliance color: "+ allianceColor);


            panelsTelemetry.debug("intake retracted: "+ intake.isRetracted());
            panelsTelemetry.debug("outtake retracted: "+ outtake.isRetracted());


            panelsTelemetry.debug("State sample: "+ sampleMachine.getState());


            panelsTelemetry.debug("Outtake Pos: "+ outtake.getCachedPos());
            panelsTelemetry.debug("Extendo Pos: "+ intake.getCachedExtendoPos());
            panelsTelemetry.debug("Intake target pos: "+ intake.getTarget());
            panelsTelemetry.debug("Outtake target pos: "+ outtake.getSetPoint());
            panelsTelemetry.debug("intake end direct: "+ intake.limitSwitch.isPressed());
            panelsTelemetry.debug("Outtake analog: "+ outtake.getFlipAnalog());
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
            intake.setSweeper(!gamepad1.dpad_up);
            panelsTelemetry.debug("Low Bucket: "+ lowBucket);
            panelsTelemetry.debug("Overfillpos: "+ overfillPos);

            long currLoop = System.nanoTime();
            panelsTelemetry.debug("Ms per loop: "+ ((currLoop - prevLoop) / 1000000));
            prevLoop = currLoop;

            panelsTelemetry.update(telemetry);
        }
        if (opModeIsActive()){
            sampleMachine.stop();
            drive.setLatchPos(Hang.LatchPositions.EXTENDED);
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
                panelsTelemetry.debug("Outtake Pos: "+ outtake.getCachedPos());
                panelsTelemetry.debug("Drive current: "+ cachedCurrent);
                panelsTelemetry.debug("Is greater than thresh: "+(getCachedCurrent()>currentThresh));
                panelsTelemetry.debug("Drive current: "+ getCachedCurrent());
                panelsTelemetry.debug("Thresh: "+ currentThresh);
                panelsTelemetry.update(telemetry);
            }
        }
    }
    public double getCachedCurrent(){
        return cachedCurrent;
    }
}
