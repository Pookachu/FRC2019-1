package frc.robot;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;

public class liftPID extends PIDSubsystem {
    private PWMVictorSPX LiftA = new PWMVictorSPX(4);
    private PWMVictorSPX LiftB = new PWMVictorSPX(5);
    Encoder HeightyLiftyThingy = new Encoder(1, 0, true, Encoder.EncodingType.k1X);

    public liftPID() {
        super("liftPID", 0.002, 0, 0);
        setAbsoluteTolerance(10);
        getPIDController().setContinuous(false);
        getPIDController().setOutputRange(-1, 1);
        getPIDController().setInputRange(0, 6000);
    }

    public void initDefaultCommand() {

    }

    protected double returnPIDInput() {
        return HeightyLiftyThingy.get();
    }

    protected void enablePID() {
        getPIDController().enable();
    }

    protected void setPoint(double setPoint) {
        getPIDController().setSetpoint(setPoint);
        System.out.println("Set Point:");
        System.out.println(setPoint);
    }

    protected void usePIDOutput(double output) {
        System.out.println("Encoder Output:");
        //System.out.println(output);
        System.out.println(HeightyLiftyThingy.get());
        if (output >= 1) {
            output = 1;
        }
        LiftA.set(-0.6*output);
        LiftB.set(-0.6*output);
    }
}