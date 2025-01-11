package subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drivetrain {
    CachedMotorEx leftFront;
    CachedMotorEx leftBack;
    CachedMotorEx rightFront;
    CachedMotorEx rightBack;

    SimpleMotorFeedforward forwardFeedforward=new SimpleMotorFeedforward(0.12, 1);
    SimpleMotorFeedforward strafeFeedforward=new SimpleMotorFeedforward(0.26, 1);
    SimpleMotorFeedforward headingFeedforward=new SimpleMotorFeedforward(0.135, 1);

    private boolean ptoEngaged=false;

    CachedServo leftpto, rightpto;

    public Drivetrain(HardwareMap hwMap) {
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
        setPtoEngaged(false);
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
    public void setPtoEngaged(boolean engaged){
        ptoEngaged=engaged;
        if (ptoEngaged){
            leftpto.setPosition(0.4);
            rightpto.setPosition(0.5);
        }else{
            leftpto.setPosition(0.7);
            rightpto.setPosition(0.1);
        }
    }
    public boolean isPtoEngaged(){
        return ptoEngaged;
    }
}
