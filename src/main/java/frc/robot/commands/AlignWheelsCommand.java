package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Timer;

public class AlignWheelsCommand extends Command {

    private final Timer timer;
    private final Drivetrain drivetrain;

    public AlignWheelsCommand(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.timer = new Timer();
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        drivetrain.setEncoders();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return timer.get() > 2.0;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }
}
