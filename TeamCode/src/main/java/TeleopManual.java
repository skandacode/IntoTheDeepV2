import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;

import subsystems.Drivetrain;
import subsystems.Intake;
import subsystems.Outtake;

@Config
@TeleOp
public class TeleopManual extends LinearOpMode{
    Outtake outtake;
    Intake intake;
    Drivetrain drive;

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
        outtake = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap);
        drive = new Drivetrain(hardwareMap);

        waitForStart();

        long prevloop=System.nanoTime();

        while (opModeIsActive()){
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
                drive.setWeightedPowers(-gamepad1.left_stick_y/3, -gamepad1.left_stick_x/3, -gamepad1.right_stick_x/3, 0);
            }
            if (gamepad1.left_bumper){
                outtake.openClaw();
            }
            if (gamepad1.right_bumper){
                outtake.closeClaw();
            }
            if (gamepad1.y){
                intake.setCover(true);
                intake.intakePos(500);
                outtake.transferPos();
            }
            if (gamepad1.x){
                intake.transferPos();
                outtake.openClaw();
            }
            if (gamepad1.b){
                outtake.sampleScore();
            }
            if (gamepad1.a){
                outtake.specGrab();
            }
            if (gamepad1.dpad_up){
                outtake.specHold();
            }
            if (gamepad1.dpad_down){
                outtake.specScore();
            }
            outtake.update();
            intake.update();

            long currloop=System.nanoTime();
            telemetry.addData("Looptime", ((currloop-prevloop)/1000000)+" ms");
            prevloop=currloop;
            telemetry.update();
        }
    }
}
