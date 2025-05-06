package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Drivetrain extends SubsystemBase {

    public final MecanumDrive mecanumDrive;

    private SparkMax maxone;
    private SparkMax maxtwo;
    private SparkMax maxthree;
    private SparkMax maxfour;

    public Drivetrain() {
        // Initialize MecanumDrive
        SparkMaxConfig normal = new SparkMaxConfig();
            normal
                .inverted(false)
                .idleMode(IdleMode.kBrake);
            normal.encoder
                .positionConversionFactor(1000)
                .velocityConversionFactor(1000);
            normal.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(1.0, 0.0, 0.0);
        
        maxone = new SparkMax(0, MotorType.kBrushed);
        maxone.configure(normal, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        maxtwo = new SparkMax(1, MotorType.kBrushed);
        maxtwo.configure(normal, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        maxthree = new SparkMax(2, MotorType.kBrushed);
        maxthree.configure(normal, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        maxfour = new SparkMax(3, MotorType.kBrushed);
        maxfour.configure(normal, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);



        mecanumDrive = new MecanumDrive(maxone, maxtwo, maxthree, maxfour);
        mecanumDrive.setSafetyEnabled(false);
        mecanumDrive.setExpiration(0.1);
        mecanumDrive.setMaxOutput(1.0);

        
    }


    public SparkMax getFrontLeftMotor() {
        return maxone;
    }
    public SparkMax getRearLeftMotor() {
        return maxtwo;
    }
    public SparkMax getFrontRightMotor() {
        return maxthree;
    }
    public SparkMax getRearRightMotor() {
        return maxfour;
    }

    @Override
    public void periodic() {
        // Called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // Called once per scheduler run during simulation
    }

    /**
     * Drives the robot using mecanum drive in Cartesian coordinates.
     *
     * @param ySpeed    Forward/backward speed
     * @param xSpeed    Left/right strafe speed
     * @param zRotation Rotation speed
     */
    
     public void driveCartesian(double ySpeed, double xSpeed, double zRotation) {
        mecanumDrive.driveCartesian(ySpeed, xSpeed, zRotation);
    }

    

}
