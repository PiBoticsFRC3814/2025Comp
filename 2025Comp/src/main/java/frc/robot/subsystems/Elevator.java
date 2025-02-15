// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */


  public TalonSRX elevator1;
  public TalonSRX elevator2;
  public Boolean coral1 = false;
  public Boolean coral2 = false;
  public Boolean coral3 = false;
  public Boolean coral4 = false;
  public Boolean processor = false;
  public Boolean net = false;
  public Boolean stowed = true;
  public DigitalInput coral1Position;
  public DigitalInput coral2Position;
  public DigitalInput coral3Position;
  public DigitalInput coral4Position;
  public DigitalInput processorPosition;
  public DigitalInput netPosition;

  public Elevator() {

    elevator1 = new TalonSRX(Constants.ELEVATOR_IDS[0]);
    elevator2 = new TalonSRX(Constants.ELEVATOR_IDS[1]);

    elevator1.configPeakCurrentLimit(Constants.ELAVATOR_CURRENT_LIMIT);
    elevator2.configPeakCurrentLimit(Constants.ELAVATOR_CURRENT_LIMIT);
    elevator2.setInverted(true);

    elevator2.follow(elevator1);

    coral1Position = new DigitalInput(Constants.ELEVATOR_POSITION[0]);
    coral2Position = new DigitalInput(Constants.ELEVATOR_POSITION[1]);
    coral3Position = new DigitalInput(Constants.ELEVATOR_POSITION[2]);
    coral4Position = new DigitalInput(Constants.ELEVATOR_POSITION[3]);
    processorPosition = new DigitalInput(Constants.ELEVATOR_POSITION[4]);
    netPosition = new DigitalInput(Constants.ELEVATOR_POSITION[4]);
    //highPosition = new DigitalInput(Constants.ELEVATOR_POSITION[0]);
    //middlePosition = new DigitalInput(Constants.ELEVATOR_POSITION[1]);
    //lowPosition = new DigitalInput(Constants.ELEVATOR_POSITION[2]);

  }

  public void manualGoUp() {
    elevator1.set(TalonSRXControlMode.PercentOutput, Constants.ELEVATOR_MAX_UP_SPEED);
    if (coral1Position.get()) {
      coral1 = true;
      coral2 = false;
      coral3 = false;
      coral4 = false;
      processor = false;
      net = false;
      stowed = false;
    }
    if (coral2Position.get()) {
    coral1 = false;
    coral2 = true;
    coral3 = false;
    coral4 = false;
    processor = false;
    net = false;
    stowed = false;
    }
    if (coral3Position.get()) {
      coral1 = false;
      coral2 = false;
      coral3 = true;
      coral4 = false;
      processor = false;
      processor = false;
      net = false;
      stowed = false;
    }
    if (coral4Position.get()) {
      coral1 = false;
      coral2 = false;
      coral3 = false;
      coral4 = true;
      processor = false;
      net = false;
      stowed = false;
    }
    if (processorPosition.get()) {
      coral1 = false;
      coral2 = false;
      coral3 = false;
      coral4 = false;
      processor = true;
      net = false;
      stowed = false;
    }
    if (netPosition.get()) {
      coral1 = false;
      coral2 = false;
      coral3 = false;
      coral4 = false;
      processor = false;
      net = true;
      stowed = false;
    }
  
  }

  public void manualGoDown() {
    elevator1.set(TalonSRXControlMode.PercentOutput, Constants.ELEVATOR_MAX_DOWN_SPEED);
    if (coral1Position.get()) {
      coral1 = true;
      coral2 = false;
      coral3 = false;
      coral4 = false;
      processor = false;
      net = false;
      stowed = false;
    }
    if (coral2Position.get()) {
    coral1 = false;
    coral2 = true;
    coral3 = false;
    coral4 = false;
    processor = false;
    net = false;
    stowed = false;
    }
    if (coral3Position.get()) {
      coral1 = false;
      coral2 = false;
      coral3 = true;
      coral4 = false;
      processor = false;
      processor = false;
      net = false;
      stowed = false;
    }
    if (coral4Position.get()) {
      coral1 = false;
      coral2 = false;
      coral3 = false;
      coral4 = true;
      processor = false;
      net = false;
      stowed = false;
    }
    if (processorPosition.get()) {
      coral1 = false;
      coral2 = false;
      coral3 = false;
      coral4 = false;
      processor = true;
      net = false;
      stowed = false;
    }
    if (netPosition.get()) {
      coral1 = false;
      coral2 = false;
      coral3 = false;
      coral4 = false;
      processor = false;
      net = true;
      stowed = false;
    }
  }

  public void stopElevator(){
    elevator1.set(TalonSRXControlMode.PercentOutput, 0.0);
  } 

  public int getLastKnownPosistion(){
    if (coral1) {
      return 2;
    }
    else if (coral2) {
      return 3;
    }
    else if (coral3) {
      return 4;
    }
    else if (coral4) {
      return 5;
    }
    else if (processor){
      return 1;
    }
    else if (net){
      return 6;
    }
    else if (stowed){
      return 0;
    }
    else return 0;
  
  }

  public boolean goToCoral1 (boolean direction) {
    if (direction) {
      if (!coral1Position.get()){
        elevator1.set(TalonSRXControlMode.PercentOutput, Constants.ELEVATOR_MAX_UP_SPEED);
        return coral1;
      }
      else {
        elevator1.set(TalonSRXControlMode.PercentOutput,0.0);
        coral1 = true;
        coral2 = false;
        coral3 = false;
        coral4 = false;
        processor = false;
        net = false;
        stowed = false;
        return coral1;
      }
    }
    else {
      if (!coral1Position.get()){
        elevator1.set(TalonSRXControlMode.PercentOutput, Constants.ELEVATOR_MAX_DOWN_SPEED);
        return coral1;
      }
      else {
        elevator1.set(TalonSRXControlMode.PercentOutput,0.0);
        coral1 = true;
        coral2 = false;
        coral3 = false;
        coral4 = false;
        processor = false;
        net = false;
        stowed = false;
        return coral1;
      }
    }
  }

  public boolean goToCoral2 (boolean direction) {
    if (direction) {
      if (!coral2Position.get()){
        elevator1.set(TalonSRXControlMode.PercentOutput, Constants.ELEVATOR_MAX_UP_SPEED);
        return coral1;
      }
      else {
        elevator1.set(TalonSRXControlMode.PercentOutput,0.0);
        coral1 = false;
        coral2 = true;
        coral3 = false;
        coral4 = false;
        processor = false;
        net = false;
        stowed = false;
        return coral2;
      }
    }
    else {
      if (!coral2Position.get()){
        elevator1.set(TalonSRXControlMode.PercentOutput, Constants.ELEVATOR_MAX_DOWN_SPEED);
        return coral2;
      }
      else {
        elevator1.set(TalonSRXControlMode.PercentOutput,0.0);
        coral1 = false;
        coral2 = true;
        coral3 = false;
        coral4 = false;
        processor = false;
        net = false;
        stowed = false;
        return coral2;
      }
    }
  }

  public boolean goToCoral3 (boolean direction) {
    if (direction) {
      if (!coral3Position.get()){
        elevator1.set(TalonSRXControlMode.PercentOutput, Constants.ELEVATOR_MAX_UP_SPEED);
        return coral3;
      }
      else {
        elevator1.set(TalonSRXControlMode.PercentOutput,0.0);
        coral1 = false;
        coral2 = false;
        coral3 = true;
        coral4 = false;
        processor = false;
        net = false;
        stowed = false;
        return coral3;
      }
    }
    else {
      if (!coral3Position.get()){
        elevator1.set(TalonSRXControlMode.PercentOutput, Constants.ELEVATOR_MAX_DOWN_SPEED);
        return coral3;
      }
      else {
        elevator1.set(TalonSRXControlMode.PercentOutput,0.0);
        coral1 = false;
        coral2 = false;
        coral3 = true;
        coral4 = false;
        processor = false;
        net = false;
        stowed = false;
        return coral3;
      }
    }
  }

  public boolean goToCoral4 (boolean direction) {
    if (direction) {
      if (!coral4Position.get()){
        elevator1.set(TalonSRXControlMode.PercentOutput, Constants.ELEVATOR_MAX_UP_SPEED);
        return coral4;
      }
      else {
        elevator1.set(TalonSRXControlMode.PercentOutput,0.0);
        coral1 = false;
        coral2 = false;
        coral3 = false;
        coral4 = true;
        processor = false;
        net = false;
        stowed = false;
        return coral4;
      }
    }
    else {
      if (!coral4Position.get()){
        elevator1.set(TalonSRXControlMode.PercentOutput, Constants.ELEVATOR_MAX_DOWN_SPEED);
        return coral4;
      }
      else {
        elevator1.set(TalonSRXControlMode.PercentOutput,0.0);
        coral1 = false;
        coral2 = false;
        coral3 = false;
        coral4 = true;
        processor = false;
        net = false;
        stowed = false;
        return coral4;
      }
    }
  }
  
  public boolean goToProccesor (boolean direction) {
    if (direction) {
      if (!processorPosition.get()){
        elevator1.set(TalonSRXControlMode.PercentOutput, Constants.ELEVATOR_MAX_UP_SPEED);
        return processor;
      }
      else {
        elevator1.set(TalonSRXControlMode.PercentOutput,0.0);
        coral1 = false;
        coral2 = false;
        coral3 = false;
        coral4 = false;
        processor = true;
        net = false;
        stowed = false;
        return processor;
      }
    }
    else {
      if (!processorPosition.get()){
        elevator1.set(TalonSRXControlMode.PercentOutput, Constants.ELEVATOR_MAX_DOWN_SPEED);
        return processor;
      }
      else {
        elevator1.set(TalonSRXControlMode.PercentOutput,0.0);
        coral1 = false;
        coral2 = false;
        coral3 = false;
        coral4 = false;
        processor = true;
        net = false;
        stowed = false;
        return processor;
      }
    }
  }

  public boolean goToNet (boolean direction) {
    if (direction) {
      if (!netPosition.get()){
        elevator1.set(TalonSRXControlMode.PercentOutput, Constants.ELEVATOR_MAX_UP_SPEED);
        return net;
      }
      else {
        elevator1.set(TalonSRXControlMode.PercentOutput,0.0);
        coral1 = false;
        coral2 = false;
        coral3 = false;
        coral4 = false;
        processor = false;
        net = true;
        stowed = false;
        return coral1;
      }
    }
    else {
      if (!netPosition.get()){
        elevator1.set(TalonSRXControlMode.PercentOutput, Constants.ELEVATOR_MAX_DOWN_SPEED);
        return net;
      }
      else {
        elevator1.set(TalonSRXControlMode.PercentOutput,0.0);
        coral1 = false;
        coral2 = false;
        coral3 = false;
        coral4 = false;
        processor = false;
        net = true;
        stowed = false;
        return net;
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
