package frc.robot.logging;

import frc.robot.subsystems.elevator.ElevatorIO;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ElevatorIOInputsAutoLogged extends ElevatorIO.ElevatorIOInputs implements LoggableInputs {

    @Override
    public void toLog(LogTable table) {
        table.put("Elevator/PositionRad", positionRad);
        table.put("Elevator/VelocityRadPerSec", velocityRadPerSec);
        table.put("Elevator/AppliedVolts", appliedVolts);
        table.put("Elevator/CurrentAmps", currentAmps);
    }

    @Override
    public void fromLog(LogTable table) {
        table.get("Elevator/PositionRad", positionRad);
        table.get("Elevator/VelocityRadPerSec", velocityRadPerSec);
        table.get("Elevator/AppliedVolts", appliedVolts);
        table.get("Elevator/CurrentAmps", currentAmps);
    }

    public ElevatorIOInputsAutoLogged clone() {
        ElevatorIOInputsAutoLogged copy = new ElevatorIOInputsAutoLogged();
        copy.positionRad = this.positionRad;
        copy.velocityRadPerSec = this.velocityRadPerSec;
        copy.appliedVolts = this.appliedVolts;
        copy.currentAmps = this.currentAmps;
        return copy;
      }
    
}
