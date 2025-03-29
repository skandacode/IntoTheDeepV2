package subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Hang {
    private boolean ptoEngaged=false;
    CachedServo leftpto, rightpto;
    CachedServo leftLatch, rightLatch;

    public enum LatchPositions{
        FULLY_RETRACTED,
        EXTENDED,
        PULLDOWN
    }

    public Hang(HardwareMap hwMap){
        leftpto = new CachedServo(hwMap.servo.get("left_pto"));
        rightpto = new CachedServo(hwMap.servo.get("right_pto"));
        rightLatch = new CachedServo(hwMap.servo.get("latch_right"));
        leftLatch = new CachedServo(hwMap.servo.get("latch_left"));
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
    public boolean getPtoEngaged(){
        return ptoEngaged;
    }
    public void setLatchPos(LatchPositions pos){
        if (pos == LatchPositions.FULLY_RETRACTED){
            leftLatch.setPosition(0.98);
            rightLatch.setPosition(0.02);
            System.out.println("Latch fully retracted");
        }
        if (pos == LatchPositions.EXTENDED){
            leftLatch.setPosition(0.16);
            rightLatch.setPosition(0.84);
        }
        if (pos == LatchPositions.PULLDOWN){
            leftLatch.setPosition(0.7);
            rightLatch.setPosition(0.26);
        }
    }
}
