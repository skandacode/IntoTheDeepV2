package subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import subsystems.pathing.WayPoint;

public class Drivetrain {
    CachedMotorEx leftFront;
    CachedMotorEx leftBack;
    CachedMotorEx rightFront;
    CachedMotorEx rightBack;

    SimpleMotorFeedforward forwardFeedforward=new SimpleMotorFeedforward(0.08, 0.9);
    SimpleMotorFeedforward strafeFeedforward=new SimpleMotorFeedforward(0.2, 1);
    SimpleMotorFeedforward headingFeedforward=new SimpleMotorFeedforward(0.11, 1);

    public Hang hang;

    CachedServo leftpto, rightpto;

    public Drivetrain(HardwareMap hwMap, Telemetry telemetry, FtcDashboard dashboard) {
        leftFront = new CachedMotorEx(hwMap, "frontleft");
        leftBack = new CachedMotorEx(hwMap, "backleft");
        rightFront = new CachedMotorEx(hwMap, "frontright");
        rightBack = new CachedMotorEx(hwMap, "backright");

        leftpto = new CachedServo(hwMap.servo.get("left_pto"));
        rightpto = new CachedServo(hwMap.servo.get("right_pto"));


        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        hang=new Hang(hwMap);
    }
    public void setRawPowers(double frontleft, double frontright, double backleft, double backright){
        double maximum=Math.max(frontleft, frontright);
        maximum=Math.max(maximum, backleft);
        maximum=Math.max(maximum, backright);
        if (maximum>1){
            frontleft=frontleft/maximum;
            frontright=frontright/maximum;
            backleft=backleft/maximum;
            backright=backright/maximum;

        }
        leftFront.set(-frontleft);
        leftBack.set(-backleft);
        rightFront.set(frontright);
        rightBack.set(backright);
    }
    public void setWeightedPowers(double front, double strafe, double heading, double ptoPower){
        if (isPtoEngaged()){
            setRawPowers(ptoPower, ptoPower, ptoPower, ptoPower);
            return;
        }
        double weightedFront=forwardFeedforward.calculate(front);
        double weightedStrafe=strafeFeedforward.calculate(strafe);
        double weightedHeading=headingFeedforward.calculate(heading);

        setRawPowers(
                (weightedFront - weightedStrafe - weightedHeading),
                (weightedFront + weightedStrafe + weightedHeading),
                (weightedFront + weightedStrafe - weightedHeading),
                (weightedFront - weightedStrafe + weightedHeading)
        );
    }
    public void setWeightedPowers(double front, double strafe, double heading){
        setWeightedPowers(front, strafe, heading, 0);
    }
    public void setPtoEngaged(boolean engaged){
        hang.setPtoEngaged(engaged);
    }
    public boolean isPtoEngaged(){
        return hang.getPtoEngaged();
    }

    public void setLatchPos(Hang.LatchPositions pos){
        hang.setLatchPos(pos);
    }
}
