package frc.robot.logging;

import frc.robot.subsystems.climber.ClimberIO;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ClimberIOInputsAutoLogged extends ClimberIO.ClimberIOInputs implements LoggableInputs {

    @Override
    public void toLog(LogTable table) {
        table.put("Climber/PositionRad", positionRad);
        table.put("Climber/VelocityRadPerSec", velocityRadPerSec);
        table.put("Climber/AppliedVolts", appliedVolts);
        table.put("Climber/CurrentAmps", currentAmps);
    }

    @Override
    public void fromLog(LogTable table) {
        table.get("Climber/PositionRad", positionRad);
        table.get("Climber/VelocityRadPerSec", velocityRadPerSec);
        table.get("Climber/AppliedVolts", appliedVolts);
        table.get("Climber/CurrentAmps", currentAmps);
    }

    public ClimberIOInputsAutoLogged clone() {
        ClimberIOInputsAutoLogged copy = new ClimberIOInputsAutoLogged();
        copy.positionRad = this.positionRad;
        copy.velocityRadPerSec = this.velocityRadPerSec;
        copy.appliedVolts = this.appliedVolts;
        copy.currentAmps = this.currentAmps;
        return copy;
      }
    
}
