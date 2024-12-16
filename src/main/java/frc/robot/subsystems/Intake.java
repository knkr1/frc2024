package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;

public class Intake extends SubsystemBase {
    private CANSparkMax intakeMotor;

    public Intake() {
        intakeMotor = new CANSparkMax(12, MotorType.kBrushless);
    }

    public void conf() {
        intakeMotor.restoreFactoryDefaults();
        // intakeMotor.enableVoltageCompensation(12);
        intakeMotor.setSmartCurrentLimit(43);
        intakeMotor.setInverted(false);
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setClosedLoopRampRate(0.1);
        intakeMotor.setOpenLoopRampRate(0.1);
        CANSparkMaxUtil.setCANSparkMaxBusUsage(intakeMotor, Usage.kMinimal, false);
        Timer.delay(0.2);
        intakeMotor.burnFlash();
        Timer.delay(0.2);
    }

    public Command RunMotor(){
        return run(()->{
            intakeMotor.set(0.5);
        });
    }
    public Command StopMotor(){
        return run(()->{
            intakeMotor.set(0.0);
        });
    }
}
