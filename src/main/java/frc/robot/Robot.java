/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
//import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.RelativeEncoder;

import javax.naming.InitialContext;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Robot extends TimedRobot {
  /*
   * Configuration
   */

  // Encoders
  private static final double driveEncoderConversionRatio = (6 * Math.PI) / 10.75;

  // DIO Ports
  //private static final int intakeSensorPort = 0;

  // PWM Ports
  private static final int intakeMotorPort = 0;
  private static final int BackLeftConMotorPort = 1;
 private static final int FrontLeftConMotorPort = 2;
 private static final int BackRightConMotorPort = 3;
 private static final int FrontRightConMotorPort = 4;

  // CAN IDs
  //private static final int conveyorID = 30; 
  private static final int LeftClimberMotor1ID = 53;
  private static final int RightClimberMotor1ID = 55;

  //private static final int rightShooterID = 40;
  //private static final int leftShooterID = 41;

  private static final int leftRearID = 52;
  private static final int rightRearID = 31;
  private static final int leftFrontID = 54;
  private static final int rightFrontID = 51;

  /*
   * Components
   */
  // Joysticks
  private Joystick mainJoystick;
  private Joystick secondaryJoystick;

  // DIO
  //private DigitalInput intakeSensor;

  // Motor Controllers
  private Talon intakeMotor;
  //private Talon climberLockMotor;

  private Talon BackLeftConMotor;
  private Talon BackRightConMotor;
  private Talon FrontLeftConMotor;
  private Talon FrontRightConMotor;

  private CANSparkMax LeftClimberMotor1;
  private CANSparkMax RightClimberMotor1;


 // private CANSparkMax rightShooterMotor;
  //private CANSparkMax leftShooterMotor;

  private CANSparkMax leftRearMotor;
  private CANSparkMax rightRearMotor;
  private CANSparkMax leftFrontMotor;
  private CANSparkMax rightFrontMotor;

  // Encoders
  private RelativeEncoder driveEncoder;

  // Shuffleboard
  private ShuffleboardTab sbTab;
  private NetworkTableEntry sbDriveTypeEntry;

  // Limelight
  //private NetworkTable limelightTable;

  // Misc
  int ticksToConveyorStop = 0;
  int currentStoredBalls = 0;
  boolean updatedBallCount = false;

  @Override
  public void robotInit() {
    // Joysticks
    mainJoystick = new Joystick(0);
    secondaryJoystick = new Joystick(1);

    // DIO
   // intakeSensor = new DigitalInput(intakeSensorPort);

    // Motor Controllers
  intakeMotor = new Talon(intakeMotorPort);
  BackLeftConMotor = new Talon(BackLeftConMotorPort);  
  FrontLeftConMotor = new Talon(FrontLeftConMotorPort);
  BackRightConMotor = new Talon(BackRightConMotorPort);
  FrontRightConMotor = new Talon(FrontRightConMotorPort);


  //intakeMotor = new Talon(intakeMotorPort);
  
   // climberMotor = new CANSparkMax(climberID, MotorType.kBrushless);

    //rightShooterMotor = new CANSparkMax(rightShooterID, MotorType.kBrushless);
    //leftShooterMotor = new CANSparkMax(leftShooterID, MotorType.kBrushless);

    leftRearMotor = new CANSparkMax(leftRearID, MotorType.kBrushless);
    rightRearMotor = new CANSparkMax(rightRearID, MotorType.kBrushless);
    leftFrontMotor = new CANSparkMax(leftFrontID, MotorType.kBrushless);
    rightFrontMotor = new CANSparkMax(rightFrontID, MotorType.kBrushless);
    LeftClimberMotor1 = new CANSparkMax(LeftClimberMotor1ID, MotorType.kBrushless);
    RightClimberMotor1 = new CANSparkMax(RightClimberMotor1ID, MotorType.kBrushless);

    // Encoders
    driveEncoder = leftRearMotor.getEncoder();
    driveEncoder.setPositionConversionFactor(driveEncoderConversionRatio);

    // Shuffleboard
    sbTab = Shuffleboard.getTab("Drive");
    sbDriveTypeEntry = sbTab.add("Current Balls", 0).getEntry();

    // Limelight
    //limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  }

  @Override
  public void autonomousInit() {
    driveEncoder.setPosition(0);
  }

  @Override
  public void autonomousPeriodic(){ 
    if(driveEncoder.getPosition() < 12){
      leftFrontMotor.set(0.05);
      leftRearMotor.set(0.05);
  
      rightFrontMotor.set(-0.05);
      rightRearMotor.set(-0.05);
    } else {
      leftFrontMotor.set(0);
      leftRearMotor.set(0);
  
      rightFrontMotor.set(0);
      rightRearMotor.set(0);
    }
  }
  


  @Override
  public void teleopPeriodic() {
    /*
     * Shuffleboard
     *
    sbDriveTypeEntry.setNumber(currentStoredBalls);

    /*
     *
     *  Robot Drive Code
     
    //controller input

float targetXAxis;
float targetYAxis;
//The max speed the robot can change in a second. Set this higher if the robot is to slugish, lower if to quick;
double xAxisMaxSpeedPerSecond= 0.1;
double yAxisMaxSpeedPerSecond= 0.1;
//The current speed input to set the motors too
float xAxisSpeed = 0;
float yAxisSpeed = 0;

public Void Update();
{


targetXAxis = Controller.axis.x;//Get controller inputs x axis

float differenceX = targetXAxis-xAxisSpeed;// Calculate the difference between the current speed and target speed

float speedX = xAxisMaxSpeedPerSecond * Time.DeltaTime; // Calculate the max change in speed that can occur this frame. 
//Using DeltaTime over just a constant speed makes it work fine even if the program runs slower;

xAxisSpeed += Float.Clamp(differenceX,-speedX,speedX); //Clamp the difference with the max speed
xAxisSpeed = mainJoystick.getRawAxis(1)/2;
yAxisSpeed = mainJoystick.getRawAxis(4)/3;


//The same thing for y axis
targetYAxis = Controller.axis.y;

var differenceY = targetYAxis-yAxisSpeed;

var speedY = yAxisMaxSpeedPerSecond * Time.DeltaTime;

yAxisSpeed += Float.Clamp(differenceY,-speedY,speedY);
    leftFrontMotor.set(-mainJoystick.getRawAxis(1) / 2 + mainJoystick.getRawAxis(4) / 3);
    leftRearMotor.set(-mainJoystick.getRawAxis(1) / 2 + mainJoystick.getRawAxis(4) / 3);

    rightFrontMotor.set(mainJoystick.getRawAxis(1) / 2 + mainJoystick.getRawAxis(4) / 3);
    rightRearMotor.set(mainJoystick.getRawAxis(1) / 2 + mainJoystick.getRawAxis(4) / 3);


///put code that sets the robot speed to xAxisSpeed and yAxisSpeed here.///

}
   /* leftFrontMotor.set(-mainJoystick.getRawAxis(1) / 2 + mainJoystick.getRawAxis(4) / 3);
    leftRearMotor.set(-mainJoystick.getRawAxis(1) / 2 + mainJoystick.getRawAxis(4) / 3);

    rightFrontMotor.set(mainJoystick.getRawAxis(1) / 2 + mainJoystick.getRawAxis(4) / 3);
    rightRearMotor.set(mainJoystick.getRawAxis(1) / 2 + mainJoystick.getRawAxis(4) / 3);
*/
    // convayer
  if (secondaryJoystick.getRawButton(4)) {
      BackRightConMotor.set(-.4);
      FrontRightConMotor.set(-.4);
      BackLeftConMotor.set(.4);
      FrontLeftConMotor.set(.4);

    } else {
      BackRightConMotor.set(0);
      FrontRightConMotor.set(0);
      BackLeftConMotor.set(0);
      FrontLeftConMotor.set(0);
    
    }if (secondaryJoystick.getRawButton(1)) {
      BackRightConMotor.set(.4);
      FrontRightConMotor.set(.4);
      BackLeftConMotor.set(-.4);
      FrontLeftConMotor.set(-.4);

    } else {
      BackRightConMotor.set(0);
      FrontRightConMotor.set(0);
      BackLeftConMotor.set(0);
      FrontLeftConMotor.set(0);
    }
     
    

    // Intake
    if (secondaryJoystick.getRawButton(5)) {
      intakeMotor.set(.5);
        }
    else if (secondaryJoystick.getRawButton(6))
      intakeMotor.set(-.5);
     else {
      intakeMotor.set(0);
    }

    

    // Climber
    if (secondaryJoystick.getRawAxis(3) > .9) {
      LeftClimberMotor1.set(.5);
      RightClimberMotor1.set(-.5);
    }
    else if (secondaryJoystick.getRawAxis(2) > .9) {
      LeftClimberMotor1.set(-.5);
      RightClimberMotor1.set(.5);
    }
    else {
      LeftClimberMotor1.set(0);
      RightClimberMotor1.set(0);
    }



    /*// Limelight
    if (secondaryJoystick.getRawButton(4)) {
      limelightTable.getEntry("ledMode").setNumber(3);
      Number targetOffsetAngle_Horizontal = limelightTable.getEntry("tx").getNumber(0);

      leftFrontMotor.set(0.01 * targetOffsetAngle_Horizontal.doubleValue());
      leftRearMotor.set(0.01 * targetOffsetAngle_Horizontal.doubleValue());
      rightFrontMotor.set(0.01 * targetOffsetAngle_Horizontal.doubleValue());
      rightRearMotor.set(0.01 * targetOffsetAngle_Horizontal.doubleValue());   
    } else {
      limelightTable.getEntry("ledMode").setNumber(1);
    }*/
  
    }
  }

