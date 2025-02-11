package frc.robot.other;

public class MotorTransform {
    private double mul;
    private double offset;

    public MotorTransform(double mul,double offset){
        this.mul=mul;
        this.offset=offset;
    }

    public double transformPos(double input){
        return (input*mul)+offset;
    }

    public double transformVel(double input){
        return input*mul;
    }

    public double transformPosInv(double input){
        return (input-offset)/mul;
    }

    public double transformVelInv(double input){
        return input/mul;
    }
}
