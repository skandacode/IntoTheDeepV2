package subsystems.pathing;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class WayPoint {
    Pose2D position;
    Pose2D tolerance;

    public WayPoint(Pose2D pos, Pose2D tolerance){
        this.position=pos;
        this.tolerance=tolerance;
    }
    public Pose2D getPosition(){
        return position;
    }

    public Pose2D getTolerance() {
        return tolerance;
    }
}