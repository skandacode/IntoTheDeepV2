package subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Hang {
    private boolean ptoEngaged=false;
    CachedServo leftpto, rightpto;

    public Hang(HardwareMap hwMap){
        leftpto = new CachedServo(hwMap.servo.get("left_pto"));
        rightpto = new CachedServo(hwMap.servo.get("right_pto"));
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
}
