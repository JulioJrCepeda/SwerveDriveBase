package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule extends SubsystemBase {	
    private final CANSparkMax m_driveMotor;
    private final CANSparkMax m_turningMotor
    ;
    private final RelativeEncoder m_driveEncoder;
    private final RelativeEncoder m_turningEncoder;
    private final CANcoder absoluteEncoder;

    private final SparkPIDController m_drivePIDController;
    private final PIDController m_turningPIDController;
    private int id;
    private boolean OnCommand;

    private SwerveModuleState m_desiredState; // class for desired angle and velocity
    private double offset;

    public SwerveModule(int driveMotorID, int turningMotorID, int CANCoderID, double offset) {
        m_driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        m_turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);

        m_driveEncoder = m_driveMotor.getEncoder();
        m_turningEncoder = m_turningMotor.getEncoder();
        absoluteEncoder = new CANcoder(CANCoderID);
        m_turningEncoder.setPosition(0);

        this.offset = offset;
        this.id = driveMotorID;
        this.OnCommand = false;

        m_turningEncoder.setPositionConversionFactor(1.0/SwerveConstants.GEAR_RATIO);
        m_driveEncoder.setVelocityConversionFactor(1.0/SwerveConstants.DRIVE_GEAR_RATIO);
        m_driveEncoder.setPositionConversionFactor(1.0/SwerveConstants.DRIVE_GEAR_RATIO);
       
        m_drivePIDController = m_driveMotor.getPIDController();
		m_drivePIDController.setP(0.01); // Need to adjust PID values

        m_turningPIDController = new PIDController(0.1, 0, 0);
		m_turningPIDController.enableContinuousInput(0, 2 * Math.PI);

        m_desiredState = new SwerveModuleState();
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        //TODO Optimize the desired state to avoid unnecessary rotation
        //m_desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(getEncoderAngle()));
        m_desiredState = desiredState;
    }

    public void SetCommandState(boolean onCommand){
        m_turningPIDController.reset();
        OnCommand = onCommand;
    }
    
    public void resetEncoder(){
        m_turningEncoder.setPosition(0);
    }

    public void periodic() {
        // Set the drive motor speed
        m_drivePIDController.setReference(getRPMs(), ControlType.kVelocity);

        // Set the turning motor position
		double angle = getEncoderAngle();
        double desiredAngle = getDesiredAngle();
        double turnOutput = m_turningPIDController.calculate(angle, desiredAngle);	
        m_turningMotor.set(turnOutput);

        if(id == 3) {
            SmartDashboard.putNumber("CurrentAngle", angle);
            SmartDashboard.putNumber("RawAngle", m_turningEncoder.getPosition());
            SmartDashboard.putNumber("DesiredAngle", desiredAngle);
            SmartDashboard.putNumber("angleOutput", turnOutput);
            SmartDashboard.putNumber("DriveSpeed", getRPMs());
        }
    }

    double getRPMs(){
        return (m_desiredState.speedMetersPerSecond * 60) / SwerveConstants.WHEEL_CIRCUMFERENCE;
    }

    double getEncoderAngle(){
        if(OnCommand){
            return absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
        } else {
            return (m_turningEncoder.getPosition() % 1.0) * 2 * Math.PI;
        }
    }

    double getDesiredAngle(){
        if(OnCommand){
            return Math.toRadians(offset);
        } else {
            return m_desiredState.angle.getRadians();
        }
    }

    //functions currently NOT in use!

    public SwerveModulePosition getPosition() {
        double position = m_driveEncoder.getPosition() * SwerveConstants.WHEEL_CIRCUMFERENCE;
        Rotation2d angle = Rotation2d.fromRadians(getEncoderAngle());
        return new SwerveModulePosition(position, angle);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            m_driveEncoder.getVelocity(),
            Rotation2d.fromRadians(getEncoderAngle())
        );
    }

}
