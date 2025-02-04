package frc.robot.logging;

import frc.robot.subsystems.mechanism.MechanismIO;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class MechanismIOInputsAutoLogged extends MechanismIO.MechanismIOInputs implements LoggableInputs {

    @Override
    public void toLog(LogTable table) {
        table.put("Mechanism/CoralPositionRad", coralPositionRad);
        table.put("Mechanism/AlgaePositionRad", algaePositionRad);
        table.put("Mechanism/PivotPositionRad", pivotPositionRad);
    
        table.put("Mechanism/CoralVelocityRadPerSec", coralVelocityRadPerSec);
        table.put("Mechanism/AlgaeVelocityRadPerSec", algaeVelocityRadPerSec);
        table.put("Mechanism/PivotVelocityRadPerSec", pivotVelocityRadPerSec);
    
        table.put("Mechanism/CoralAppliedVolts", coralAppliedVolts);
        table.put("Mechanism/AlgaeAppliedVolts", algaeAppliedVolts);
        table.put("Mechanism/PivotAppliedVolts", pivotAppliedVolts);
    
        table.put("Mechanism/CoralCurrentAmps", coralCurrentAmps);
        table.put("Mechanism/AlgaeCurrentAmps", algaeCurrentAmps);
        table.put("Mechanism/PivotCurrentAmps", pivotCurrentAmps);
    }

    @Override
    public void fromLog(LogTable table) {
        table.get("Mechanism/CoralPositionRad", coralPositionRad);
        table.get("Mechanism/AlgaePositionRad", algaePositionRad);
        table.get("Mechanism/PivotPositionRad", pivotPositionRad);
    
        table.get("Mechanism/CoralVelocityRadPerSec", coralVelocityRadPerSec);
        table.get("Mechanism/AlgaeVelocityRadPerSec", algaeVelocityRadPerSec);
        table.get("Mechanism/PivotVelocityRadPerSec", pivotVelocityRadPerSec);
    
        table.get("Mechanism/CoralAppliedVolts", coralAppliedVolts);
        table.get("Mechanism/AlgaeAppliedVolts", algaeAppliedVolts);
        table.get("Mechanism/PivotAppliedVolts", pivotAppliedVolts);
    
        table.get("Mechanism/CoralCurrentAmps", coralCurrentAmps);
        table.get("Mechanism/AlgaeCurrentAmps", algaeCurrentAmps);
        table.get("Mechanism/PivotCurrentAmps", pivotCurrentAmps);
    }

    public MechanismIOInputsAutoLogged clone() {
        MechanismIOInputsAutoLogged copy = new MechanismIOInputsAutoLogged();

        copy.coralPositionRad = this.coralPositionRad;
        copy.algaePositionRad = this.algaePositionRad;
        copy.pivotPositionRad = this.pivotPositionRad;
    
        copy.coralVelocityRadPerSec = this.coralVelocityRadPerSec;
        copy.algaeVelocityRadPerSec = this.algaeVelocityRadPerSec;
        copy.pivotVelocityRadPerSec = this.pivotVelocityRadPerSec;
    
        copy. coralAppliedVolts = this.coralAppliedVolts;
        copy. algaeAppliedVolts = this.algaeAppliedVolts;
        copy. pivotAppliedVolts = this.pivotAppliedVolts;
    
        copy.coralCurrentAmps = this.coralCurrentAmps;
        copy.algaeCurrentAmps = this.algaeCurrentAmps;
        copy.pivotCurrentAmps = this.pivotCurrentAmps;
        
        return copy;
      }
    
}
