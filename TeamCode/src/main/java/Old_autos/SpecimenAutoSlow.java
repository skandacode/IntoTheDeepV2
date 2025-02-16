package Old_autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
public class SpecimenAutoSlow extends LinearOpMode {
    Drivetrain drive;
    Intake intake;
    Outtake outtake;

    public static boolean closedPressed=false;
    public static boolean scoredPressed=false;
    enum autoStates {
        clawclose, depositPosPreload, pauseToDepo, scorePreload,
        intakePreExtend1, intakeExtend1, intakeExtendStart, intakeExtend1Pos, intakeReversePos1, intakeReverse1,
        intakePreExtend2, intakeExtend2, intakeExtend2Pos, intakeReversePos2, intakeReverse2,
        intakePreExtend3, intakeExtend3, intakeExtend3Pos, intakeReversePos3, intakeReverse3,
        intakeRetract1, intakePos1,intakePos1f2, closeClaw1, depositPos1, depositPos1f2, score1,
        preintake2, intakePos2, intakePos2f2, closeClaw2, depositPos2, depositPos2f2,score2,
        preintake3,intakePos3, intakePos3f2, closeClaw3, depositPos3, depositPos3f2, strafe,score3,
        preintake4,intakePos4, intakePos4f2, closeClaw4, depositPos4, depositPos4f2, score4,
        park
    }
    public enum SpecimenScoreStates {IDLE, C, INTAKEPOS, INTAKE, CLOSE_CLAW, HOLD, SCORE, OPENCLAW, CLOSEBEFORERETRACT, RESET, RETRACT}


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
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        drive = new Drivetrain(hardwareMap, telemetry, dashboard);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);

        WayPoint preintake=new WayPoint(new Pose2D(DistanceUnit.INCH, -5, -38, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH,  3, 3, AngleUnit.DEGREES, 5));
        WayPoint depositPosPreload2=new WayPoint(new Pose2D(DistanceUnit.INCH, 0, -26, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH,  1, 1, AngleUnit.DEGREES, 2));
        WayPoint depositPos1=new WayPoint(new Pose2D(DistanceUnit.INCH, -2, -45, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH,  4, 4, AngleUnit.DEGREES, 3));
        WayPoint depositPos2=new WayPoint(new Pose2D(DistanceUnit.INCH, -2, -24, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH,  1, 1, AngleUnit.DEGREES, 2));
        WayPoint depositPos12=new WayPoint(new Pose2D(DistanceUnit.INCH, -7, -45, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH,  4, 4, AngleUnit.DEGREES, 3));
        WayPoint depositPos22=new WayPoint(new Pose2D(DistanceUnit.INCH, -7, -24, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH,  1, 1, AngleUnit.DEGREES, 2));
        WayPoint depositPos22Strafe=new WayPoint(new Pose2D(DistanceUnit.INCH, -2, -24, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH,  1, 1, AngleUnit.DEGREES, 2));
        WayPoint depositPos13=new WayPoint(new Pose2D(DistanceUnit.INCH, -10, -45, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH,  4, 4, AngleUnit.DEGREES, 3));
        WayPoint depositPos23=new WayPoint(new Pose2D(DistanceUnit.INCH, -10, -24, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH,  1, 1, AngleUnit.DEGREES, 2));
        WayPoint depositPos14=new WayPoint(new Pose2D(DistanceUnit.INCH, -10, -45, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH,  4, 4, AngleUnit.DEGREES, 3));
        WayPoint depositPos24=new WayPoint(new Pose2D(DistanceUnit.INCH, -10, -23, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH,  1, 1, AngleUnit.DEGREES, 2));
        WayPoint intakeExtend1Pos=new WayPoint(new Pose2D(DistanceUnit.INCH, 22, -37, AngleUnit.DEGREES, 34),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 1));
        WayPoint intakeReversePos1=new WayPoint(new Pose2D(DistanceUnit.INCH, 20, -42, AngleUnit.DEGREES, -45),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint intakeExtend2Pos=new WayPoint(new Pose2D(DistanceUnit.INCH, 34, -36, AngleUnit.DEGREES, 34),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint intakeExtend3Pos=new WayPoint(new Pose2D(DistanceUnit.INCH, 41.5, -34.5, AngleUnit.DEGREES, 30),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint intakeReversePos2=new WayPoint(new Pose2D(DistanceUnit.INCH, 20, -42, AngleUnit.DEGREES, -45),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint intakePreExtend1=new WayPoint(new Pose2D(DistanceUnit.INCH, 10, -50, AngleUnit.DEGREES, 37),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint intakePreExtend2=new WayPoint(new Pose2D(DistanceUnit.INCH, 22, -42, AngleUnit.DEGREES, 30),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint specimenGrab=new WayPoint(new Pose2D(DistanceUnit.INCH, 36, -51, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint specimenGrabForward=new WayPoint(new Pose2D(DistanceUnit.INCH, 36, -57, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH, 1, 0.5, AngleUnit.DEGREES, 2));
        WayPoint specimenGrab2=new WayPoint(new Pose2D(DistanceUnit.INCH, 36, -49, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));
        WayPoint specimenGrabForward2=new WayPoint(new Pose2D(DistanceUnit.INCH, 36, -57, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH, 0.5, 0.5, AngleUnit.DEGREES, 2));
        WayPoint specimenGrab3=new WayPoint(new Pose2D(DistanceUnit.INCH, 36, -48, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH, 1, 0.5, AngleUnit.DEGREES, 2));
        WayPoint specimenGrabForward3=new WayPoint(new Pose2D(DistanceUnit.INCH, 36, -57, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH, 0.5, 0.5, AngleUnit.DEGREES, 2));

        WayPoint park=new WayPoint(new Pose2D(DistanceUnit.INCH, 45, -52, AngleUnit.DEGREES, 180),
                new Pose2D(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 2));

        StateMachine specimenMachine = new StateMachineBuilder()
                .state(SpecimenScoreStates.INTAKE)
                .onEnter(() -> {
                    outtake.sampleScore();
                    outtake.setTargetPos(0);
                    outtake.openClawWide();
                })
                .transition(() -> closedPressed)
                .onExit(()->closedPressed=false)
                .state(SpecimenScoreStates.CLOSE_CLAW)
                .onEnter(() -> outtake.closeClaw())
                .transitionTimed(0.3)
                .state(SpecimenScoreStates.HOLD)
                .onEnter(() -> outtake.specHold())
                .transition(() -> (outtake.atTarget() && scoredPressed))
                .onExit(()->scoredPressed=false)
                .state(SpecimenScoreStates.SCORE)
                .onEnter(() -> outtake.specScore())
                .transitionTimed(0.1)
                .state(SpecimenScoreStates.OPENCLAW)
                .onEnter(() -> outtake.openClaw())
                .transitionTimed(0.1)
                .state(SpecimenScoreStates.CLOSEBEFORERETRACT)
                .onEnter(() -> outtake.closeClaw())
                .transitionTimed(0.3, SpecimenScoreStates.INTAKE)
                .build();


        StateMachine autoMachine = new StateMachineBuilder()
                .state(autoStates.depositPosPreload)
                .onEnter(()->{
                    outtake.closeClaw();
                    outtake.specHold();
                })
                .transitionTimed(0.2)
                .state(autoStates.pauseToDepo)
                .onEnter(()->{
                    drive.setTarget(depositPosPreload2);
                })
                .transitionTimed(1.25)
                .state(autoStates.scorePreload)
                .onEnter(()->scoredPressed=true)
                .transition(()->specimenMachine.getState()== SpecimenScoreStates.OPENCLAW)

                .state(autoStates.intakePreExtend1)
                .onEnter(()->drive.setTarget(intakePreExtend1))
                .transitionTimed(0.3)
                .state(autoStates.intakeExtend1Pos)
                .onEnter(()->drive.setTarget(intakeExtend1Pos))
                .transitionTimed(0.2)
                .state(autoStates.intakeExtend1)
                .onEnter(()->{
                    intake.intakePos(900);
                    intake.setIntakePower(1);
                })
                .transitionTimed(0.2)
                .state(autoStates.intakeExtendStart)
                .onEnter(()->intake.setIntakePower(1))
                .transitionTimed(0.6)

                .state(autoStates.intakeReversePos1)
                .onEnter(()->drive.setTarget(intakeReversePos1))
                .transitionTimed(0.5)
                .state(autoStates.intakeReverse1)
                .onEnter(()->intake.setIntakePower(-1))
                .transitionTimed(0.3)
                .state(autoStates.intakePreExtend2)
                .onEnter(()->drive.setTarget(intakePreExtend2))
                .transitionTimed(0.2)
                .state(autoStates.intakeExtend2)
                .onEnter(()->intake.setIntakePower(1))
                .transitionTimed(0.2)
                .state(autoStates.intakeExtend2Pos)
                .onEnter(()->drive.setTarget(intakeExtend2Pos))
                .transitionTimed(0.6)
                .state(autoStates.intakeReversePos2)
                .onEnter(()->drive.setTarget(intakeReversePos1))
                .transitionTimed(0.6)
                .state(autoStates.intakeReverse2)
                .onEnter(()->intake.setIntakePower(-1))
                .transitionTimed(0.4)
                .state(autoStates.intakePreExtend3)
                .onEnter(()->drive.setTarget(intakePreExtend2))
                .transitionTimed(0.2)
                .state(autoStates.intakeExtend3)
                .onEnter(()->intake.setIntakePower(1))
                .transitionTimed(0.2)
                .state(autoStates.intakeExtend3Pos)
                .onEnter(()->drive.setTarget(intakeExtend3Pos))
                .transitionTimed(1)
                .state(autoStates.intakeReversePos3)
                .onEnter(()->drive.setTarget(intakeReversePos1))
                .transitionTimed(0.6)
                .state(autoStates.intakeReverse3)
                .onEnter(()->intake.setIntakePower(-1))
                .transitionTimed(0.4)
                .state((autoStates.intakeRetract1))
                .onEnter(()->intake.transferPos())
                .transitionTimed(0.1)
                .state(autoStates.intakePos1)
                .onEnter(()->drive.setTarget(specimenGrab))
                .transitionTimed(1.5)
                .state(autoStates.intakePos1f2)
                .onEnter(()->drive.setTarget(specimenGrabForward))
                .transitionTimed(0.4)
                .state(autoStates.closeClaw1)
                .onEnter(()-> closedPressed=true)
                .transitionTimed(0.25)
                .state(autoStates.depositPos1)
                .onEnter(()->drive.setTarget(depositPos1))
                .transition(()->drive.atTarget())
                .transitionTimed(0.7)
                .state(autoStates.depositPos1f2)
                .onEnter(()->drive.setTarget(depositPos2))
                .transitionTimed(0.8)
                .state(autoStates.score1)
                .onEnter(()->scoredPressed=true)
                .transition(()->specimenMachine.getState()== SpecimenScoreStates.OPENCLAW)
                .transitionTimed(1)
                .state(autoStates.preintake2)
                .onEnter(()->drive.setTarget(preintake))
                .transitionTimed(0.2)
                .state(autoStates.intakePos2)
                .onEnter(()->drive.setTarget(specimenGrab2))
                .transitionTimed(1.7)
                .state(autoStates.intakePos2f2)
                .onEnter(()->drive.setTarget(specimenGrabForward2))
                .transitionTimed(0.4)
                .state(autoStates.closeClaw2)
                .onEnter(()-> closedPressed=true)
                .transitionTimed(0.3)
                .state(autoStates.depositPos2)
                .onEnter(()->drive.setTarget(depositPos12))
                .transition(()->drive.atTarget())
                .transitionTimed(0.7)
                .state(autoStates.depositPos2f2)
                .onEnter(()->drive.setTarget(depositPos22))
                .transitionTimed(0.8)
                .state(autoStates.score2)
                .onEnter(()->scoredPressed=true)
                .transition(()->specimenMachine.getState()== SpecimenScoreStates.OPENCLAW)

                .transitionTimed(1)
                .state(autoStates.preintake3)
                .onEnter(()->drive.setTarget(preintake))
                .transitionTimed(0.2)
                .state(autoStates.intakePos3)
                .onEnter(()->drive.setTarget(specimenGrab3))
                .transitionTimed(1.7)
                .state(autoStates.intakePos3f2)
                .onEnter(()->drive.setTarget(specimenGrabForward3))
                .transitionTimed(0.4)
                .state(autoStates.closeClaw3)
                .onEnter(()-> closedPressed=true)
                .transitionTimed(0.3)
                .state(autoStates.depositPos3)
                .onEnter(()->drive.setTarget(depositPos13))
                .transition(()->drive.atTarget())
                .transitionTimed(0.7)
                .state(autoStates.depositPos3f2)
                .onEnter(()->drive.setTarget(depositPos23))
                .transitionTimed(0.8)
                .state(autoStates.strafe)
                .onEnter(()->drive.setTarget(depositPos22Strafe))
                .transitionTimed(0.6)
                .state(autoStates.score3)
                .onEnter(()->scoredPressed=true)
                .transition(()->specimenMachine.getState()== SpecimenScoreStates.OPENCLAW)

                .transitionTimed(1)
                .state(autoStates.preintake4)
                .onEnter(()->drive.setTarget(preintake))
                .transitionTimed(0.2)
                .state(autoStates.intakePos4)
                .onEnter(()->drive.setTarget(specimenGrab3))
                .transitionTimed(1.6)
                .state(autoStates.intakePos4f2)
                .onEnter(()->drive.setTarget(specimenGrabForward3))
                .transitionTimed(0.4)
                .state(autoStates.closeClaw4)
                .onEnter(()-> closedPressed=true)
                .transitionTimed(0.3)
                .state(autoStates.depositPos4)
                .onEnter(()->drive.setTarget(depositPos14))
                .transition(()->drive.atTarget())
                .transitionTimed(0.8)
                .state(autoStates.depositPos4f2)
                .onEnter(()->drive.setTarget(depositPos24))
                .transitionTimed(1.4)
                .state(autoStates.score4)
                .onEnter(()->scoredPressed=true)
                .transition(()->specimenMachine.getState()== SpecimenScoreStates.OPENCLAW)

                .transitionTimed(1)
                .state(autoStates.park)
                .onEnter(()->drive.setTarget(park))
                .build();

        WayPoint startPoint=new WayPoint(new Pose2D(DistanceUnit.INCH, 2, -60.5, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH, 0.5, 0.5, AngleUnit.DEGREES, 0.5));


        drive.setTarget(startPoint);
        drive.setPosition(startPoint.getPosition());

        intake.transferPos();
        outtake.closeClaw();
        drive.setPtoEngaged(false);
        intake.setIntakePower(0);
        while (opModeInInit()){
            drive.update();
            telemetry.update();
            if (gamepad1.touchpad){
                drive.calibrateIMU();
                drive.setPosition(startPoint.getPosition());
            }
        }
        waitForStart();
        drive.setTarget(startPoint);
        drive.setPosition(startPoint.getPosition());
        autoMachine.start();
        specimenMachine.start();
        specimenMachine.setState(SpecimenScoreStates.CLOSE_CLAW);
        outtake.specHold();
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
            specimenMachine.update();
            drive.update();
            drive.updatePIDS();
            intake.update();
            outtake.update();
            long currLoop = System.nanoTime();
            telemetry.addData("Outtake position", outtake.getCachedPos());
            telemetry.addData("Ms per loop", (currLoop - prevLoop) / 1000000.0);
            prevLoop = currLoop;
            telemetry.update();
        }
    }
}
