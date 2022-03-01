//OI = Operator Interface
package frc.team1918.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * This class reads and writes values to/from the SmartDashboard
 */
public class Dashboard {
    public static final class Climber {
        public static final void setClimberDirection(String direction) { SmartDashboard.putString("ClimberDirection", direction); }
        public static final void setClimberPosition(double position) { SmartDashboard.putNumber("ClimberPosition", position); }
        public static final void setWhirlyPosition(String position) { SmartDashboard.putString("WhirlygigPosition", position); }
    }
    public static final class Shooter {
        public static final void setCurrentSpeed(double speed) { SmartDashboard.putNumber("ShootSpeed",speed); }
        public static final void setHoodPosition(boolean up) { SmartDashboard.putString("HoodPosition",(up)?"up":"down"); }
        public static final void setTargetSpeed(double speed) { SmartDashboard.putNumber("Shooter Target Speed",speed); }
        public static final double getTargetSpeed(double default_val) { return (double) SmartDashboard.getNumber("Shooter Target Speed",default_val); }
    }
    public static final class Gyro {
        public static final void setGyroAngle(double angle) { SmartDashboard.putNumber("GyroAngle",angle); }
    }
    public static final class DriveTrain {
        public static final void setTurnPosition(String module, double value) { SmartDashboard.putNumber(module+" Position", value); }
        public static final void setTurnSetpoint(String module, double value) { SmartDashboard.putNumber(module+" Setpoint", value); }
        public static final void setTurnPositionError(String module, double value) { SmartDashboard.putNumber(module+" Position Error", value); }
        public static final void setTurnVelocity(String module, double value) { SmartDashboard.putNumber(module+" Velocity", value); }
        public static final void setTurnPositionErrorChange(String module, double value) { SmartDashboard.putNumber(module+" Position Error Change", value); }
        public static final void setDriveVelocity(String module, double value) { SmartDashboard.putNumber(module+" Drive Velocity", value); }
    }
}