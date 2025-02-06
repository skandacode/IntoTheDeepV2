package subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@TeleOp
@Config
public class ReadBrushlandDistance extends LinearOpMode {
    public static double k=0.05;
    public void runOpMode() throws InterruptedException {
        LaserRangefinder lrf = new LaserRangefinder(hardwareMap.get(RevColorSensorV3.class, "laser"));
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // set the clock speed on this i2c bus to 400kHz
        lrf.i2c.setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        double filtered=50;
        waitForStart();
        while (opModeIsActive()) {
            double distance = lrf.getDistance(DistanceUnit.MM);
            filtered=filtered*(1-k)+distance*k;
            telemetry.addData("Distance", distance);
            telemetry.addData("Filtered", filtered);

            telemetry.addData("Status", lrf.getStatus());
            telemetry.update();
        }
    }
}
