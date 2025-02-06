import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.List;

import subsystems.Drivetrain;
import subsystems.Intake;
import subsystems.Outtake;
import subsystems.pathing.WayPoint;

@Autonomous
@Config
public class SampleAuto extends LinearOpMode {
    Drivetrain drive;
    Intake intake;
    Outtake outtake;
    public static boolean yPressed=false;
    public static boolean lbPressed=false;
    public static int samp=0;
    public static Intake.SampleColor allianceColor= Intake.SampleColor.BLUE;
    public static Intake.SampleColor opposingColor= Intake.SampleColor.RED;

    enum autoStates {PREBUCKET1, BUCKET1, SCORE1,
        EXTEND1, INTAKE1, PREBUCKET2, BUCKET2, SCORE2,
        EXTEND2, INTAKE2, PREBUCKET3, BUCKET3, SCORE3,
        EXTEND3, INTAKE3, PREBUCKET4, BUCKET4, SCORE4,
        PRESUB, SUB, EXTENDSUB, INTAKESUB, RETRACTSUB, EXTENDAGAIN, INTAKEAGAIN, INTAKESUBSTRAFE, EJECT, PREBUCKETSUB, PREBUCKET5, BUCKET5, SCORE5,
        PREPARK, LIFTOUTTAKE, PARK, TOUCHBAR
    }
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
        samp=0;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        drive = new Drivetrain(hardwareMap, telemetry, dashboard);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);

        WayPoint bucketPos2 = new WayPoint(new Pose2D(DistanceUnit.INCH, -51, -56.5, AngleUnit.DEGREES, 38),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint bucketPos3 = new WayPoint(new Pose2D(DistanceUnit.INCH, -50, -56.5, AngleUnit.DEGREES, 42),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint bucketPos25th = new WayPoint(new Pose2D(DistanceUnit.INCH, -50, -53.5, AngleUnit.DEGREES, 40),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint presub = new WayPoint(new Pose2D(DistanceUnit.INCH, -36, -9, AngleUnit.DEGREES, 0),
                new Pose2D(DistanceUnit.INCH, 5, 5, AngleUnit.DEGREES, 2));
        WayPoint sub = new WayPoint(new Pose2D(DistanceUnit.INCH, -12, -11, AngleUnit.DEGREES, 0),
                new Pose2D(DistanceUnit.INCH, 3, 3, AngleUnit.DEGREES, 2));
        WayPoint subStrafe = new WayPoint(new Pose2D(DistanceUnit.INCH, -12, -6, AngleUnit.DEGREES, 0),
                new Pose2D(DistanceUnit.INCH, 3, 3, AngleUnit.DEGREES, 2));
        WayPoint farStrafe = new WayPoint(new Pose2D(DistanceUnit.INCH, -15, 0, AngleUnit.DEGREES, 0),
                new Pose2D(DistanceUnit.INCH, 3, 3, AngleUnit.DEGREES, 2));


        WayPoint sample1 = new WayPoint(new Pose2D(DistanceUnit.INCH, -43.5, -49, AngleUnit.DEGREES, 89),
                new Pose2D(DistanceUnit.INCH, 5, 5, AngleUnit.DEGREES, 5));
        WayPoint sample2 = new WayPoint(new Pose2D(DistanceUnit.INCH, -53.5, -49, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH, 5, 5, AngleUnit.DEGREES, 5));
        WayPoint sample3 = new WayPoint(new Pose2D(DistanceUnit.INCH, -39, -36, AngleUnit.DEGREES, 157),
                new Pose2D(DistanceUnit.INCH, 5, 5, AngleUnit.DEGREES, 2));


        StateMachine sampleMachine = new StateMachineBuilder()
                .state(AutomatedTeleop.SampleStates.IDLE)
                .onEnter(() -> {
                    intake.transferPos();
                    intake.setIntakePower(0);
                })
                .transition(() -> yPressed)
                .onExit(()->yPressed=false)
                .state(AutomatedTeleop.SampleStates.EXTEND)
                .onEnter(()->{
                    intake.intakePos(1000);
                })
                .transition(()->intake.isSampleIntaked())
                .state(AutomatedTeleop.SampleStates.SENSORWAIT)
                .onEnter(()->intake.intakePos())
                .transitionTimed(0.05)
                .state(AutomatedTeleop.SampleStates.SENSE)
                .transition(() -> {
                    currentSense=intake.getColor();
                    return currentSense == Intake.SampleColor.YELLOW || currentSense== allianceColor;
                }, AutomatedTeleop.SampleStates.RETRACT)
                .transition(()->currentSense == Intake.SampleColor.NONE, AutomatedTeleop.SampleStates.EXTEND)
                .transition(() -> currentSense != Intake.SampleColor.YELLOW && currentSense != allianceColor, AutomatedTeleop.SampleStates.EJECTFLIP)

                .state(AutomatedTeleop.SampleStates.EJECTFLIP, true)
                .onEnter(() -> {
                    intake.eject();
                })
                .transitionTimed(0.2, AutomatedTeleop.SampleStates.EJECTLIDOPEN)

                .state(AutomatedTeleop.SampleStates.EJECTLIDOPEN, true)
                .onEnter(() -> {
                    intake.setCover(false);
                })
                .transitionTimed(0.4, AutomatedTeleop.SampleStates.EXTEND)

                .state(AutomatedTeleop.SampleStates.RETRACT)
                .onEnter(()->{
                    intake.transferPos();
                    intake.setIntakePower(0.3);
                    intake.setCover(true);
                    outtake.transferPos();
                    outtake.openClaw();
                })
                .transitionTimed(0.05)
                .state(AutomatedTeleop.SampleStates.OPENCOVER)
                .onEnter(() -> {
                    intake.setCover(false);
                    intake.setIntakePower(0.05);
                    outtake.openClaw();
                })
                .transition(()-> intake.isRetracted())

                .state(AutomatedTeleop.SampleStates.WAIT)
                .onEnter(() -> {
                    intake.setIntakePower(0.4);
                    outtake.setRail(0.29);
                })
                .transitionTimed(0.4)

                .state(AutomatedTeleop.SampleStates.CLOSE)
                .onEnter(() -> {
                    outtake.closeClaw();
                    intake.setIntakePower(0.2);
                })
                .transitionTimed(0.2)

                .state(AutomatedTeleop.SampleStates.LIFT)
                .onEnter(() -> {
                    outtake.setTargetPos(970);
                    intake.setIntakePower(-1);
                })
                .transitionTimed(0.3)

                .state(AutomatedTeleop.SampleStates.PARTIALFLIP)
                .onEnter(()->outtake.partialSampleFlip())
                .transition(()->outtake.getCachedPos()>950)

                .state(AutomatedTeleop.SampleStates.SCORE)
                .onEnter(()->outtake.sampleScore())
                .transition(() -> lbPressed)
                .onExit(()->lbPressed=false)

                .state(AutomatedTeleop.SampleStates.AUTOWAIT)
                .transitionTimed(0.2)

                .state(AutomatedTeleop.SampleStates.OPEN)
                .onEnter(() -> outtake.openClaw())
                .transitionTimed(0.5)
                .onExit(() -> {
                    outtake.setTargetPos(0);
                    outtake.transferPos();
                })
                .state(AutomatedTeleop.SampleStates.LOWERLIFT)
                .loop(()->{
                    if (outtake.getFlipAnalog()>1.937 && outtake.isRetracted()){
                        outtake.openClaw();
                    }
                })
                .transition(() -> outtake.getFlipAnalog()>1.937, AutomatedTeleop.SampleStates.IDLE)
                .onExit(()->outtake.openClaw())
                .build();

        StateMachine autoMachine = new StateMachineBuilder()
                .state(autoStates.PREBUCKET1)
                .onEnter(() -> drive.setTarget(bucketPos2))
                .transition(()->drive.atTarget() && sampleMachine.getState()== AutomatedTeleop.SampleStates.SCORE)
                .state(autoStates.SCORE1)
                .onEnter(() -> lbPressed = true)
                .transitionTimed(0.7)


                .state(autoStates.INTAKE1)
                .onEnter(() -> drive.setTarget(sample1))
                .transition(() -> drive.atTarget())
                .state(autoStates.EXTEND1)
                .onEnter(() -> yPressed = true)
                .transition(() -> sampleMachine.getState() == AutomatedTeleop.SampleStates.RETRACT)
                .state(autoStates.PREBUCKET2)
                .onEnter(() -> drive.setTarget(bucketPos2))
                .transition(() -> drive.atTarget() && outtake.getCachedPos() > 900)
                .state(autoStates.SCORE2)
                .onEnter(() -> lbPressed = true)
                .transitionTimed(0.7)


                .state(autoStates.INTAKE2)
                .onEnter(() -> drive.setTarget(sample2))
                .transition(()->drive.atTarget())
                .state(autoStates.EXTEND2)
                .onEnter(() -> yPressed = true)
                .transition(() -> sampleMachine.getState() == AutomatedTeleop.SampleStates.RETRACT)
                .state(autoStates.PREBUCKET3)
                .onEnter(() -> drive.setTarget(bucketPos2))
                .transition(() -> drive.atTarget() && outtake.getCachedPos() > 900)
                .state(autoStates.SCORE3)
                .onEnter(() -> lbPressed = true)
                .transitionTimed(0.7)


                .state(autoStates.INTAKE3)
                .onEnter(() -> drive.setTarget(sample3))
                .transition(() -> drive.atTarget())
                .state(autoStates.EXTEND3)
                .onEnter(() -> yPressed = true)
                .transition(() -> sampleMachine.getState() == AutomatedTeleop.SampleStates.RETRACT)
                .state(autoStates.PREBUCKET4)
                .onEnter(() -> {
                    drive.setTarget(bucketPos3);
                    System.out.println("changed");
                })
                .transition(() -> drive.atTarget() && outtake.getCachedPos() > 900)
                .state(autoStates.SCORE4)
                .onEnter(() -> lbPressed = true)
                .transitionTimed(0.7)

                .state(autoStates.PRESUB)
                .onEnter(() -> drive.setTarget(presub))
                .transition(() -> drive.atTarget())
                .state(autoStates.SUB)
                .onEnter(() -> {
                    System.out.println(samp);
                    if (samp==0){
                        drive.setTarget(sub);
                        samp+=1;
                        System.out.println("regular sub");
                    }
                    else{
                        drive.setTarget(subStrafe);
                        System.out.println("sub strafe");
                    }
                })
                .transitionTimed(0.7)
                .state(autoStates.EXTENDSUB)
                .onEnter(() -> {
                    intake.setTargetPos(1100);
                })
                .transitionTimed(0.15)
                .state(autoStates.INTAKESUB)
                .onEnter(() -> {
                    intake.intakePos();
                    intake.setTargetPos(1100);
                })
                .transitionTimed(0.6, autoStates.RETRACTSUB)
                .transition(() -> intake.getColor() == Intake.SampleColor.YELLOW || intake.getColor() == allianceColor, autoStates.PREBUCKETSUB)
                .transition(() -> intake.getColor() == opposingColor, autoStates.EJECT)

                .state(autoStates.RETRACTSUB)
                .onEnter(() -> {
                    intake.transferPos();
                    intake.setIntakePower(-0.5);
                })
                .transition(()->intake.isRetracted(), autoStates.EXTENDAGAIN)

                .state(autoStates.EXTENDAGAIN)
                .onEnter(() -> {
                    intake.setTargetPos(1100);
                    intake.setIntakePower(-1);
                })
                .transitionTimed(0.15)

                .state(autoStates.INTAKEAGAIN)
                .onEnter(() -> {
                    intake.setIntakePower(1);
                    intake.intakePos(1100);
                })
                .transitionTimed(0.7, autoStates.INTAKESUBSTRAFE)
                .transition(() -> intake.getColor() == Intake.SampleColor.YELLOW || intake.getColor() == allianceColor, autoStates.PREBUCKETSUB)
                .transition(() -> intake.getColor() == opposingColor, autoStates.EJECT)

                .state(autoStates.INTAKESUBSTRAFE)
                .onEnter(() -> drive.setTarget(farStrafe))
                .transitionTimed(1, autoStates.RETRACTSUB)
                .transition(() -> intake.getColor() == Intake.SampleColor.YELLOW || intake.getColor() == allianceColor, autoStates.PREBUCKETSUB)
                .transition(() -> intake.getColor() == opposingColor, autoStates.EJECT)

                .state(autoStates.EJECT, true)
                .onEnter(() -> {
                    intake.eject();
                    intake.setCover(false);
                })
                .transitionTimed(0.7, autoStates.EXTENDAGAIN)
                .state(autoStates.PREBUCKETSUB)
                .onEnter(() -> {
                    sampleMachine.setState(AutomatedTeleop.SampleStates.EXTEND);
                    intake.setCover(true);
                    intake.setIntakePower(-1);
                    drive.setTarget(presub);
                })
                .transitionTimed(0.6)
                .state(autoStates.PREBUCKET5)
                .onEnter(() -> drive.setTarget(bucketPos25th))
                .transition(() -> drive.atTarget() && outtake.getCachedPos() > 900)


                .state(autoStates.SCORE5)
                .onEnter(() -> lbPressed = true)
                .transitionTimed(0.7, autoStates.PRESUB)
                .build();

        WayPoint startPoint = new WayPoint(new Pose2D(DistanceUnit.INCH, -36, -61.5, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH, 0.5, 0.5, AngleUnit.DEGREES, 0.5));

        drive.setTarget(startPoint);
        drive.setPosition(startPoint.getPosition());

        intake.transferPos();
        outtake.closeClaw();
        drive.setPtoEngaged(false);
        intake.setIntakePower(0);
        while (opModeInInit()) {
            drive.update();
            telemetry.update();
            if (gamepad1.touchpad) {
                drive.calibrateIMU();
                drive.setPosition(startPoint.getPosition());
            }
            if (gamepad1.a) {
                allianceColor = Intake.SampleColor.BLUE;
                opposingColor = Intake.SampleColor.RED;
                gamepad1.setLedColor(0, 0, 1, 1000);
            }
            if (gamepad1.b) {
                allianceColor = Intake.SampleColor.RED;
                opposingColor = Intake.SampleColor.BLUE;
                gamepad1.setLedColor(1, 0, 0, 1000);
            }
        }
        waitForStart();
        long startTime = System.nanoTime();
        outtake.partialSampleFlip();
        drive.setTarget(startPoint);
        drive.setPosition(startPoint.getPosition());
        sampleMachine.start();
        autoMachine.start();
        sampleMachine.setState(AutomatedTeleop.SampleStates.LIFT);
        long prevLoop = System.nanoTime();
        while (opModeIsActive()) {
            if (!(controlhub==null)) {
                controlhub.clearBulkCache();
            }else{
                for (LynxModule hub:allHubs){
                    hub.clearBulkCache();
                }
            }
            autoMachine.update();
            sampleMachine.update();
            drive.update();
            drive.updatePIDS();
            intake.update();
            outtake.update();
            long currLoop = System.nanoTime();
            telemetry.addData("Outtake position", outtake.getCachedPos());
            telemetry.addData("Ms per loop", (currLoop - prevLoop) / 1000000.0);
            prevLoop = currLoop;
            telemetry.update();
            if (autoMachine.getState() == autoStates.PARK) {
                System.out.println(System.nanoTime() - startTime);
            }
        }
    }
}
