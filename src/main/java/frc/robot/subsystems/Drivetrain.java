package frc.robot.subsystems;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import java.util.function.DoubleSupplier;
// import com.ctre.phoenix6.hardware.Pigeon2;
// import com.pathplanner.lib.auto.AutoBuilder;

public class Drivetrain extends SubsystemBase {	
    private final SwerveDriveKinematics m_kinematics;
    //private final SwerveDriveOdometry m_odometry;
	
    //private final Pigeon2  m_pigeon;
 
    private final SwerveModule m_frontLeft;
    private final SwerveModule m_frontRight; 
    private final SwerveModule m_backLeft; 
    private final SwerveModule m_backRight;


    public Drivetrain() {
        //m_pigeon = new Pigeon2(SwerveConstants.PIGEON_ID);	
        
        // Initialize kinematics with the module locations in meters I think
		m_kinematics = new SwerveDriveKinematics( 
		new Translation2d(0.368, 0.368), //frontLeft
		new Translation2d(0.368, -0.368), //frontRight
		new Translation2d(-0.368, 0.368), //backLeft
		new Translation2d(-0.368, -0.368) //backRight
		);	
        		
        // Initialize Modules
	    m_frontLeft = new SwerveModule(SwerveConstants.FRONT_LEFT_DRIVE, 
        SwerveConstants.FRONT_LEFT_STEER,
        SwerveConstants.FRONT_LEFT_CANCODER,
        SwerveConstants.FRONT_LEFT_OFFSET
        );
		m_frontRight = new SwerveModule(SwerveConstants.FRONT_RIGHT_DRIVE, 
        SwerveConstants.FRONT_RIGHT_STEER,
        SwerveConstants.FRONT_RIGHT_CANCODER,
        SwerveConstants.FRONT_RIGHT_OFFSET
        );
		m_backLeft = new SwerveModule(SwerveConstants.BACK_LEFT_DRIVE, 
        SwerveConstants.BACK_LEFT_STEER,
        SwerveConstants.BACK_LEFT_CANCODER,
        SwerveConstants.BACK_LEFT_OFFSET
        );
		m_backRight = new SwerveModule(SwerveConstants.BACK_RIGHT_DRIVE, 
        SwerveConstants.BACK_RIGHT_STEER,
        SwerveConstants.BACK_RIGHT_CANCODER,
        SwerveConstants.BACK_RIGHT_OFFSET
        );

        // Initalize obometry with kinematics and the current module positions
        // m_odometry = new SwerveDriveOdometry(m_kinematics, m_pigeon.getRotation2d(), new SwerveModulePosition[] {
        //     m_frontLeft.getPosition(),
        //     m_frontRight.getPosition(),
        //     m_backLeft.getPosition(),
        //     m_backRight.getPosition(),
        // });

		//Set up AutoBuilder for Autonomus
		// AutoBuilder.configureHolonomic(
        //     this::getPose, // Robot pose supplier
        //     this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        //     this::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        //     this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        //     SwerveConstants.AUTO_CONFIG,
        //     () -> {
        //       // Boolean supplier that controls when the path will be mirrored for the red alliance
        //       // This will flip the path being followed to the red side of the field.
        //       // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        //       var alliance = DriverStation.getAlliance();
        //       if (alliance.isPresent()) {
        //         return alliance.get() == DriverStation.Alliance.Red;
        //       }
        //       return false;
        //     },
        //     this // Reference to this subsystem to set requirements
		// );
    }

    public void drive(DoubleSupplier xSpeedSupplier, DoubleSupplier ySpeedSupplier, DoubleSupplier wSupplier) {
        // Create ChassisSpeeds from joystick inputs
        ChassisSpeeds speeds = new ChassisSpeeds(
            xSpeedSupplier.getAsDouble(),
            ySpeedSupplier.getAsDouble(),
            wSupplier.getAsDouble()
        );
        driveRobotRelative(speeds);
    }

	public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.MAX_SPEED);

        // Set the desired state for each swerve module
        m_frontLeft.setDesiredState(moduleStates[0]);
        m_frontRight.setDesiredState(moduleStates[1]);
        m_backLeft.setDesiredState(moduleStates[2]);
        m_backRight.setDesiredState(moduleStates[3]);
    }
	
    public void resetEncoders(){
        m_frontLeft.resetEncoder();
        m_frontRight.resetEncoder();
        m_backLeft.resetEncoder();
        m_backRight.resetEncoder();
    }
    public void OnCommand(boolean OnComamnd){
        m_frontLeft.SetCommandState(OnComamnd);
        m_frontRight.SetCommandState(OnComamnd);
        m_backLeft.SetCommandState(OnComamnd);
        m_backRight.SetCommandState(OnComamnd);
    }


    @Override
    public void periodic() {
        // Update the odometry with the current heading and module states
        // m_odometry.update(
        //     m_pigeon.getRotation2d(),
        //     new SwerveModulePosition[] {
        //         m_frontLeft.getPosition(),
        //         m_frontRight.getPosition(),
        //         m_backLeft.getPosition(),
        //         m_backRight.getPosition()
        //     }
        // );
    
        // // Debuging Only
        // Pose2d pose = getPose();
        // SmartDashboard.putNumber("Robot X", pose.getX());
        // SmartDashboard.putNumber("Robot Y", pose.getY());
        // SmartDashboard.putNumber("Robot Heading", pose.getRotation().getDegrees());

    }

    //functions currently NOT in use!

    // ChassisSpeeds getFieldRelativeSpeeds(ChassisSpeeds speeds) {
    //     Rotation2d robotHeading = m_pigeon.getRotation2d();
    //     return ChassisSpeeds.fromRobotRelativeSpeeds(speeds, robotHeading);
    // }
	

    // void driveFieldRelative(ChassisSpeeds speeds) {
	// 	   ChassisSpeeds gloabalSpeeds = getFieldRelativeSpeeds(speeds);
    
    //     SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(gloabalSpeeds);
    
    //     SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.MAX_SPEED);
    
    //     // Set the desired state for each swerve module
    //     m_frontLeft.setDesiredState(moduleStates[0]);
    //     m_frontRight.setDesiredState(moduleStates[1]);
    //     m_backLeft.setDesiredState(moduleStates[2]);
    //     m_backRight.setDesiredState(moduleStates[3]);
    // }


    // public ChassisSpeeds getCurrentSpeeds() {
    //     //Get the current states of the swerve modules and return a ChassisSpeeds
    //     SwerveModuleState[] moduleStates = new SwerveModuleState[] {
    //         m_frontLeft.getState(),
    //         m_frontRight.getState(),
    //         m_backLeft.getState(),
    //         m_backRight.getState()
    //     };
    //     return m_kinematics.toChassisSpeeds(moduleStates);
    // }

    // Resets the robot's odometry to the given pose
    // public void resetPose(Pose2d pose) {
    //     m_odometry.resetPosition(m_pigeon.getRotation2d(), new SwerveModulePosition[] {
    //             m_frontLeft.getPosition(),
    //             m_frontRight.getPosition(),
    //             m_backLeft.getPosition(),
    //             m_backRight.getPosition()
    //         }, 
    //         pose);
    // }

    // public Pose2d getPose() {
    //      return m_odometry.getPoseMeters();
    // }
}
