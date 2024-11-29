package frc.robot.subsystems;

import org.ejml.equation.Function;
import org.photonvision.PhotonCamera;

import com.fasterxml.jackson.databind.deser.BuilderBasedDeserializer;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MuratCont;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.Arrays;
import java.util.Optional;
import edu.wpi.first.wpilibj.AnalogInput;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Indexer extends SubsystemBase {
    private AnalogInput noteLaser;
    public Indexer(){
        noteLaser = new AnalogInput(3);
    }
    public boolean isIndex(){
        return noteLaser.getVoltage() > 4.0;
    }
}