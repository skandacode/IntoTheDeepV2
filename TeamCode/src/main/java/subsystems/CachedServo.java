package subsystems;

import com.qualcomm.robotcore.hardware.Servo;

public class CachedServo {
    Servo servo;
    double prevPos = 0;

    public CachedServo(Servo servo) {
        this.servo = servo;
    }

    public void setPosition(double pos) {
        if (pos == prevPos) {
            return;
        }
        servo.setPosition(pos);
        prevPos = pos;
    }

    public double getPrevPos() {
        return prevPos;
    }

    public double getPosition() {
        return servo.getPosition();
    }

    public void scaleRange(double min, double max) {
        servo.scaleRange(min, max);
    }

    public void setDirection(Servo.Direction direction) {
        servo.setDirection(direction);
    }

    public Servo.Direction getDirection() {
        return servo.getDirection();
    }
}