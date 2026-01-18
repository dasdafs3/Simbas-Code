package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret {
    private Follower follower;
    private Servo servo1, servo2;

    private double goalY = 144-6.0;
    private double goalX = 6.0;

    private final double MAX_RIGHT_ANGLE = 90.0;
    private final double MAX_LEFT_ANGLE = -90.0;
    private final double MAX_SERVO_ANGLE = 355.0;
    private final double GEAR_RATIO = 26.0/120.0 * 96.0/20.0;

    public boolean isAuto = false;

    public Turret(HardwareMap hW, Follower follower, Alliance alliance) {
        this.follower = follower;
        servo1 = hW.servo.get("raxon");
        servo1.setDirection(Servo.Direction.REVERSE);
        servo2 = hW.servo.get("laxon");

        if(alliance == Alliance.RED) {
            goalX = 144-6.0;
        }
    }

    public void update() {
        if(isAuto) {
            double x = follower.getPose().getX();
            double y = follower.getPose().getY();

            double angleToRot = clamp(Math.atan2(goalY - y, goalX - x) - follower.getHeading(), -MAX_LEFT_ANGLE, MAX_RIGHT_ANGLE);

            servo1.setPosition(getPosition(angleToRot));
            servo2.setPosition(getPosition(angleToRot));
        }
    }

    public void spinLeft() {
        servo1.setPosition(servo1.getPosition() - .02);
        servo2.setPosition(servo2.getPosition() - .02);
    }

    public void spinRight() {
        servo1.setPosition(servo1.getPosition() + .02);
        servo2.setPosition(servo2.getPosition() + .02);
    }

    public double getYaw(double position) {
        double offset = position-0.5;
        return offset*MAX_SERVO_ANGLE*GEAR_RATIO;
    }

    public double getPosition(double yaw) {
        double offset = yaw/MAX_SERVO_ANGLE*GEAR_RATIO;
        return offset+0.5;
    }

    public double clamp(double value, double min, double max) {
        if(value < min) {
            return min;
        } else if(value > max) {
            return max;
        } else {
            return value;
        }
    }
}
