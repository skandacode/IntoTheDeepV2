package subsystems.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import subsystems.pathing.WayPoint;
import subsystems.Drivetrain;

@Config
@TeleOp
public class MecanumDrivetrainTesting extends LinearOpMode {
    Drivetrain drive;
    public static boolean feedForwardTuning=false;
    public static double frontPower=0;
    public static double strafePower=0;
    public static double turnPower=0;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry= new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        drive=new Drivetrain(hardwareMap, telemetry, dashboard);

        WayPoint startPoint=new WayPoint(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0),
                new Pose2D(DistanceUnit.INCH, 0.5, 0.5, AngleUnit.DEGREES, 0.5));
        WayPoint point1=new WayPoint(new Pose2D(DistanceUnit.INCH, 20, 0, AngleUnit.DEGREES, 0),
                new Pose2D(DistanceUnit.INCH, 0.5, 0.5, AngleUnit.DEGREES, 0.5));
        WayPoint point2=new WayPoint(new Pose2D(DistanceUnit.INCH, 20, 20, AngleUnit.DEGREES, 0),
                new Pose2D(DistanceUnit.INCH, 0.5, 0.5, AngleUnit.DEGREES, 0.5));
        WayPoint point3=new WayPoint(new Pose2D(DistanceUnit.INCH, 20, 20, AngleUnit.DEGREES, 90),
                new Pose2D(DistanceUnit.INCH, 0.5, 0.5, AngleUnit.DEGREES, 0.5));



        waitForStart();
        drive.setTarget(startPoint);
        drive.setPosition(startPoint.getPosition());
        drive.calibrateIMU();
        while (opModeIsActive()){
            drive.update();
            if (!feedForwardTuning) {
                drive.updatePIDS();
                if (gamepad1.b) {
                    drive.setTarget(startPoint);
                }
                if (gamepad1.a) {
                    drive.setTarget(point1);
                }
                if (gamepad1.x) {
                    drive.setTarget(point2);
                }
                if (gamepad1.y) {
                    drive.setTarget(point3);
                }
            }else{
                drive.setWeightedPowers(frontPower, strafePower, turnPower);
            }

            telemetry.update();
        }
    }
}
