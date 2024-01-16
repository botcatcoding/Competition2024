package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feed;

public class FeedCommand extends CommandBase
{
    double speed = 0;
    Feed feed;
    public FeedCommand(double s, Feed fd)
    {
        addRequirements(fd);
        speed = s;
        feed = fd;
    }
    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        feed.setFeed(speed);
    }
    @Override
    public void end(boolean interrupted) {

    }
    @Override
    public boolean isFinished() {
        return false;
    }
}

