// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/** Imported Libraries  */
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import java.util.Map;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.controller.PIDController;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private static final String d_default = "cubed";
  private static final String d_none = "none";
  private int m_drive_number =3;
  private String m_autoSelected;
  private String m_driveSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final SendableChooser<String> m_drive_value = new SendableChooser<>();
  private MecanumDrive m_robotDrive;
  private Joystick m_stick;
  private XboxController c_stick;
  private SlewRateLimiter filterx;
  private SlewRateLimiter filtery;
  private SlewRateLimiter filterz;
  private int autoState;
  /** These are the CAN channels for the pickup, elevator and shooter motors. */
  private int kmotor1Channel = 4;
  private int kmotor2Channel = 5;
  private int kmotor3Channel = 6;
  private WPI_TalonSRX motor1;
  private WPI_VictorSPX motor2;
  private WPI_TalonSRX motor3;
  private DoubleSolenoid climbDoublePCM;

  private Timer auto_timer;
  private DigitalInput Frontsensor;
  private DigitalInput Backsensor; 
  private int elvstate;
  private Solenoid pickPCM;

  private boolean enter;
  private double backwardsTime;
  
      // NavX sensor for robot orientation
      private AHRS navx = new AHRS(SPI.Port.kMXP);

 


    // Rotation target angle
    private double targetAngle = 0;
  private double newAngle = 45;

 

    // Rotation True
    private boolean RotateActive;

    // Timeout for the rotation
    private Timer rotationTimer = new Timer();
    private static final double TIMEOUT_SECONDS = 1.7;


  /** Code to create a Motor Speed tab on the shuffleboard and add adjustment 
   * sliders to adjust the motor speed for the shooter and pickup motors
   * This allows us to make motor speed adjustments without altering the code.
   * Refer to the following information.
   * https://docs.wpilib.org/en/stable/docs/software/dashboards/shuffleboard/getting-started/shuffleboard-tour.html
   */
  private ShuffleboardTab tab = Shuffleboard.getTab("Demo Controls");
  private GenericEntry shooterSpeed = tab.add("Shooter Speed", -.8) 
  .withWidget(BuiltInWidgets.kNumberSlider)
  .getEntry();

  private GenericEntry eleSpeed = tab.add("Elevator Shoot Speed", -.5) 
  .withWidget(BuiltInWidgets.kNumberSlider) .withProperties(Map.of("min", -1, "max", -.2))
  .getEntry();
 
  private GenericEntry twistratio = tab.add("Twist", .7) 
  .withWidget(BuiltInWidgets.kNumberSlider) .withProperties(Map.of("min", .5, "max", 1))
  .getEntry();

  private GenericEntry slewY = tab.add("Slew Rate Y", .8) 
  .withWidget(BuiltInWidgets.kNumberSlider) .withProperties(Map.of("min", .3, "max", 5))
  .getEntry();

  private GenericEntry shooterRpm = tab.add("Shooter RPM", -3200) 
  .withWidget(BuiltInWidgets.kNumberSlider) .withProperties(Map.of("min", -4500, "max", -1000))
  .getEntry();
 
  private GenericEntry slewX = tab.add("Slew Rate X", 1.5) 
  .withWidget(BuiltInWidgets.kNumberSlider) .withProperties(Map.of("min", .3, "max", 5))
  .getEntry();

  private GenericEntry slewZ = tab.add("Slew Rate Z", 1.5) 
  .withWidget(BuiltInWidgets.kNumberSlider) .withProperties(Map.of("min", .3, "max", 5))
  .getEntry();

  private GenericEntry setAngle = tab.add("Set Angle", 45) 
  .withWidget(BuiltInWidgets.kNumberSlider) .withProperties(Map.of("min", -90, "max", 90))
  .getEntry();

    private GenericEntry set_rotatekP = tab.add("Set Rotate P", .02) 
  .withWidget(BuiltInWidgets.kNumberSlider) .withProperties(Map.of("min", .01, "max", .09))
  .getEntry();

private GenericEntry set_rotatekI = tab.add("Set Rotate I", .00) 
  .withWidget(BuiltInWidgets.kNumberSlider) .withProperties(Map.of("min", .0, "max", .5))
  .getEntry();
 
  private GenericEntry set_rotatekD = tab.add("Set Rotate D", 1.5) 
  .withWidget(BuiltInWidgets.kNumberSlider) .withProperties(Map.of("min", .01, "max", 4))
  .getEntry();

    private GenericEntry set_MaxRotateSpeed = tab.add("Set Max Rotate Speed", .3) 
  .withWidget(BuiltInWidgets.kNumberSlider) .withProperties(Map.of("min", .1, "max", 1))
  .getEntry();

       // PID controller for rotation
      private PIDController pidController;
      // PID constants for rotation (tuned experimentally)
    private  double rotate_kP ; // Proportional gain
    private double rotate_kI  ;  // Integral gain
    private double rotate_kD ; // Derivative gain

       // Rotation speed limit
    private double MAX_ROTATION_SPEED = set_MaxRotateSpeed.getDouble(.3);
  
 

  /** This is a method that is used to modify the joystick value. 
   * We are cubeing the values.  What this does is to make the joystick less senstive 
   * at slower speeds making the robot easier to control but allowing the robot to go at full speed.
   * x is the joystick value
   * y is the power
   * The result is the multiplation of the joystick value by itself three times.
  Example: For a joystick value of .5 or half speed (.5 × .5 × .5 = .125) .125 or 1/8 speed is sent to the drive software.
  For a joystick value of 1 (full speed)  (1 x 1 x 1 = 1) 1 (full speed) is sent to the drive software.
   */
  private double signedPow (double x, double y) {
    double a = Math.pow(x, y);
      return a;
  }
/**This sets the velocity of the shooter motor 
 * For every revolution of the encoder 4096 edges are detected.
 * The sample period for sensing is .1 seconds.
 * In each second there are 10 sample periods.
 * In each minute there are 600 sample periods. (10 X 60)
 * If we want the motor to turn at 3000 Revolutions per minute we need to know 
 * how many revolutions that we should see in the .1 second sample period or 
 * Revolutions per .1 second (RPM / 600).
 * Finally we need to convert to the number of edges the sensor should see in a .1 second sample
 * if the motor were runing at the correct speed.
 * In this case that would be Revolutions per .1 sec * edges per revolution.
*/
  private double shooter_rpm ;
  private double v_shooter ;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() { 
    CameraServer.startAutomaticCapture();

            // Initialize navX and reset the gyro
            navx.reset();

            // Initialize the PID controller for rotation
            pidController = new PIDController(rotate_kP, rotate_kI, rotate_kD);
            pidController.setTolerance(1.5); // Set tolerance to 3 degree
    
            RotateActive = false;

    auto_timer = new Timer();

    /** The slew rate limiter is used to control how fast the robot can accerate.
     * In short it limits the rate of change of the output.
     * We use this to keep the robot from tipping as we change speeds.
     * Note: The higher the value the slower the acceleration rate.
     */

     double slew_Y = slewY.getDouble(1.0);
     double slew_X = slewX.getDouble(1.0);
     double slew_Z = slewZ.getDouble(1.0);
    filterx = new SlewRateLimiter(slew_X);
    filtery = new SlewRateLimiter(slew_Y);
    filterz = new SlewRateLimiter(slew_Z);
    
  /** This sets up the Digital inputs on the RoboRio to read the ball sensors on the robot. */
    Frontsensor  = new DigitalInput(0);                        
    Backsensor = new DigitalInput(1);
     /** This sets the initial elevator state to 0. */
    elvstate = 0;

    /** These are the CAN channels for the Falon 500 motors used for the drive system. */
    int kFrontLeftChannel = 3;
    int kFrontRightChannel = 7;
    int kRearRightChannel = 2;
    int kRearLeftChannel = 1;
    /** This sets up the Joy stick channels */
    int kJoystickControlChannel = 1;
    int kJoystickMovementChannel = 0;
    /** This sets the outputs on the pneumatic control module that are used to control
     * the climber solenoid
     */
    int kclimbFowardChannel = 1;
    int kclimbBackwardChannel=2;
    climbDoublePCM = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, kclimbFowardChannel, kclimbBackwardChannel);

    /** This sets the output on the pneumatic control module used to control
     * the pickup in and out solenoid.
     */
    pickPCM = new Solenoid(PneumaticsModuleType.CTREPCM, 3);

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

  
    
    TalonFX frontLeft = new TalonFX (kFrontLeftChannel);
    TalonFX rearLeft = new TalonFX (kRearLeftChannel);
    TalonFX frontRight = new TalonFX (kFrontRightChannel);
    TalonFX rearRight = new  TalonFX (kRearRightChannel);
       // Invert the right side motors.
    // You may need to change or remove this to match your robot.
   
    rearRight.setInverted(true);
    frontRight.setInverted(true);
   
    motor1 = new  WPI_TalonSRX (kmotor1Channel);
    //Shooter Motor
    motor2 = new  WPI_VictorSPX (kmotor2Channel);
    // Elevator Motor
    motor3 = new  WPI_TalonSRX (kmotor3Channel);
    //Pickup Motor
  
  
//Configures the feedback sensor for the shooter motor
    motor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    motor1.setSensorPhase(false);

		/* Config the Velocity closed loop gains in slot0 */
    motor1.config_kP(0, .08, 0);
    motor1.config_kI(0, 0, 0);
    motor1.config_kD(0, 4, 0);


    /** This sets up the drive system. */
    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

    m_stick = new Joystick(kJoystickMovementChannel);
    c_stick = new XboxController(kJoystickControlChannel);
  }
  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
   
    /** This sends information to the Smart Dashboatd to indicate system states*/
      SmartDashboard.putNumber("Elevator State  ", elvstate);
      SmartDashboard.putData("Pickup Arm Extended  ", pickPCM);
      SmartDashboard.putData("Climber State  ", climbDoublePCM);
      SmartDashboard.putData("Front Sensor  ", Frontsensor);
      SmartDashboard.putData("Back Sensor  ", Backsensor);
      SmartDashboard.putNumber("Drive Input  ", m_drive_number);
      SmartDashboard.putData("Drive Selected  ", m_drive_value);
      
      SmartDashboard.putNumber("Shooter RPM  ", shooter_rpm);
      m_drive_value.setDefaultOption("Cubed Input", d_default);
      m_drive_value.addOption("No Input Modification", d_none);
      SmartDashboard.putData("Drive Control Options", m_drive_value);

      shooter_rpm = shooterRpm.getDouble(1.0);
      v_shooter = (shooter_rpm / 600) * 4096;

      newAngle = setAngle.getDouble(45);
      
      rotate_kP = SmartDashboard.getNumber("kP", set_rotatekP.getDouble(kDefaultPeriod));
      rotate_kI = SmartDashboard.getNumber("kI", set_rotatekI.getDouble(kDefaultPeriod));
      rotate_kD = SmartDashboard.getNumber("kD", set_rotatekD.getDouble(kDefaultPeriod));
      pidController.setP(rotate_kP);
      pidController.setI(rotate_kI);
      pidController.setD(rotate_kD);

  
    }
   

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    autoState = 1;
    enter = true;
    backwardsTime = 0;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
    switch (autoState) {
      case 1:
      if (enter == true) 
      {
        auto_timer.reset();
        auto_timer.start();
        enter = false;
      }
        // shoot 
        double fSpeed = shooterSpeed.getDouble  (-.6);
         motor1.set(Preferences.getDouble("Motor1ForwardSpeed", fSpeed));
         motor2.set(Preferences.getDouble("Motor2ForwardSpeed", -.3));
         motor3.set(0);
         m_robotDrive.driveCartesian(0, 0,0);

        // If 3 seconds
        if (auto_timer.get() >1)
        { 
          autoState = 2;
          enter = true;
        }
        break;
      case 2:
      if (enter == true) 
      {
        auto_timer.reset();
        auto_timer.start();
        enter = false;
      }
       // setup pickup 
       pickPCM.set(true);
       
      motor3.set (-0.8);
      motor1.set(0);
      motor2.set(0);
      m_robotDrive.driveCartesian(0, 0,0);
       // if robot has stopped moving then goto state 3
       if (auto_timer.get() >1)
        { 
          autoState = 3;
          enter = true;
        }
        break;
      case 3:
       if (enter == true) 
      {
        auto_timer.reset();
        auto_timer.start();
        enter = false;
      }
      //move backwards 
      pickPCM.set(true);
      m_robotDrive.driveCartesian(.2, 0,0);
      motor3.set (-0.8);
      motor1.set(0);
      motor2.set(0);
      if (auto_timer.get() >5)
        { 
          autoState = 7;
          enter = true;
        }
      if (Frontsensor.get()==false)
      {
         backwardsTime=auto_timer.get();     
         autoState = 4;
         enter = true;
      }
        
         break;
      case 4:
      if (enter == true) 
      {
        auto_timer.reset();
        auto_timer.start();
        enter = false;
      }
      // pickup ball 
      pickPCM.set(true);
      motor2.set(-0.5);
      // If it has been 1 second then goto state 5
      motor1.set(0);
      motor3.set (-0.8);
      m_robotDrive.driveCartesian(0, 0,0);
      if (Backsensor.get()==false)
        { 
          autoState = 5;
          enter = true;
        }
        break;
      case 5:
      if (enter == true) 
      {
        auto_timer.reset();
        auto_timer.start();
        enter = false;
      }
      // move forward
      pickPCM.set(true);
      motor1.set(0);
      motor2.set(0);
      motor3.set(0);
      m_robotDrive.driveCartesian(-.2, 0,0);
      if (auto_timer.get()> backwardsTime -0.5)
        { 
          autoState = 6;
          enter = true;
        }
        break;
      case 6:
     if (enter == true) 
      {
        auto_timer.reset();
        auto_timer.start();
        enter = false;
      }
      //shoot
      motor1.set(-0.6);
      motor2.set(-0.3);
      motor3.set(0);
      m_robotDrive.driveCartesian(0, 0,0);
      if (auto_timer.get() > 3)
        { 
          autoState = 7;
          enter = true;
        }
        break;
      case 7:
      if (enter == true) 
      {
        auto_timer.reset();
        auto_timer.start();
        enter = false;
      }
        // stop 
       motor1.set(0);
       motor2.set(0);
       motor3.set(0);
       m_robotDrive.driveCartesian(0, 0,0);
      
        break;
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
 elvstate = 0;

 double slew_Y = slewY.getDouble(1.0);
 double slew_X = slewX.getDouble(1.0);
 double slew_Z = slewZ.getDouble(1.0);
filterx = new SlewRateLimiter(slew_X);
filtery = new SlewRateLimiter(slew_Y);
filterz = new SlewRateLimiter(slew_Z);
 
m_driveSelected = m_drive_value.getSelected();
// m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
System.out.println("Input selected: " + m_driveSelected);

if (m_driveSelected == "cubed")
m_drive_number = 3;
else if (m_driveSelected == "none")
m_drive_number = 1;
else 
m_drive_number = 0;

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
    /** This code takes the joystick values to control the speed and direction of the drive motors
     * signedPow(joystick value, Power Value) This takes the value of the joystick and cubes it.
     * 
     * Note: The power value must be 3 to make this work correctly.
     * 
     * filter.calculate (calculated joystick value)  This limits the rate of changed
     * or how fast the robot can accelerate or decellerate.
     
     */
double twist = twistratio.getDouble(1.0);


    m_robotDrive.driveCartesian
     (filtery.calculate (0 - signedPow (m_stick.getY(), m_drive_number)) ,
      filterx.calculate( signedPow(m_stick.getX(), m_drive_number)), 
      filterz.calculate( signedPow((m_stick.getZ()* twist), m_drive_number)));

     /** This controls the moving the pickup arm in and out
      * The solenoid we are using is spring loaded.
      When the power to the solenoid is off (false) the arm
      will retract.  This also happens when the robot is diabled or the 
      power is turned off.
      */
      
    /**Note this function has been disabled.  The bumper buttons will both extend the pickup and either pickup the ball
     * or push them away.
     * 
     * if (c_stick.getYButton())
     {pickPCM.set(true);
    
     }
     else if (c_stick.getXButton())
     {pickPCM.set(false);
    
     }

     /** This controls the speed and direction of the pickup motor
      * The arm will be extended and the motor will run.
      We get the speed value from the Shuffleboard (pSpeedf and pSpeedr)
      .8 is the default speed.
      */
     if (c_stick.getRightBumper())
     {
      pickPCM.set(true);
      motor3.set (.8);
     }
     else if (c_stick.getLeftBumper())
     {
      pickPCM.set(true);
      motor3.set (-.8);
     }
     else 
     {
      pickPCM.set(false);
       motor3.set(0);
     } 
     
     /** This code controls the climber cylinder
     
      This solenoid does not have a spring return and will stay
      in the state it is in when the robot is disabled or powered off.
      */
    if (c_stick.getRightStickButton() )
    {
      climbDoublePCM.set(kForward);
    } 
    else if (c_stick.getLeftStickButton() )
    {
      climbDoublePCM.set(kReverse);
    }
    else
    {
      climbDoublePCM.set(kOff);
    }

    /** This controls the automatic operation of the elevator to load a picked up ball.
     * This is one way of doing it.  It has been disabled.  See If statements below.
     * 
  
    switch(elvstate)
    {
      case 1:
      motor1.set(0); 
      motor2.set(0);
      if(Frontsensor.get()==false)
      {
        elvstate=2;
      }
      break;
      case 2:
      motor1.set(0); 
      motor2.set(-0.3);
      if (Backsensor.get()==false)
      {
        elvstate=3;
      }
      break;
      case 3:
      motor1.set(0); 
      motor2.set(0);
      if(c_stick.getAButton())
      {
        elvstate=4;
      }
      break;
      case 4:
       motor1.set(-0.8);
       motor2.set(-0.5);
      if(c_stick.getAButton()==false)
      {
        elvstate=1;
      }
      break;
    } 
   */
/** The Initial elevator state is set to 0.
 * State 1 means the first ball has been picked up
 * The elevator will run to index the ball.
 */
     if (Frontsensor.get()==false && elvstate==0) 
    {
      elvstate=1;
    }
    /** The elevator will run to index the ball until the ball is seen by the rear sensor.
 */
    if (elvstate==1 && Backsensor.get() ==false)  {
      elvstate=2;
    }
        /** The elevator index the ball until the ball is seen by the rear sensor.
     * This will turn off the elevator motor.
 */
    if (elvstate==2 && Frontsensor.get()==false)  {
      elvstate=3;
    }
     /** The second ball has been picked up 
      * The elevator will run to index the ball until the first ball moves past the rear sensor.
 */   
   if (elvstate==3 && Backsensor.get()==true)  {
        elvstate=4;
      }
           /** The second ball has been picked up 
      * The elevator will run to index the ball until the first ball moves past the rear sensor.
      The first ball will be above the rear sensor and the second ball will be below the sensor.
     * This will turn off the elevator motor.
 */ 
      if ((c_stick.getAButton()==true) || (c_stick.getBButton()==true)) {
        double elevSetSpeed = eleSpeed.getDouble(1.0);
        motor2.set(elevSetSpeed);
        elvstate=0;
}    
 /**The A or B buttons override the automatic indexing and reset the elevator state to 0 
  * 
   */ 
    if ((c_stick.getAButton()==false) && (c_stick.getBButton()==false) && elvstate==1 || elvstate==3) {
      motor2.set(-0.3);
    } 
 
   if ((elvstate==0 || elvstate==2 || elvstate==4) && ( (c_stick.getAButton()==false) && (c_stick.getBButton()==false)) ){
    motor2.set(0);
    } 
     
    double shooterPercentage = shooterSpeed.getDouble(1.0);

/** This controls the shooter motor
 * The A button is used to show how the shooter performs without any feedback controls.
 * The B button demonstrates how the accuracy of the shooter can be improved with a closed loop system.
 */
    if (c_stick.getAButton())
    {
       motor1.set(ControlMode.PercentOutput, shooterPercentage);
     }
  else if (c_stick.getBButton()==true)  {

    motor1.set(ControlMode.Velocity, v_shooter);

  }
  
     else
     {
       motor1.set(ControlMode.PercentOutput, 0);
     }

           // Check if X button is pressed on the Xbox controller for rotation
           if (c_stick.getXButtonPressed()) {
          
            // Set target angle for 45-degree rotation clockwise
            targetAngle = navx.getAngle() + newAngle;
            rotationTimer.reset();
            rotationTimer.start();
            RotateActive = true;
        
      }
        while (RotateActive == true) {
          
        
        // Run PID loop to rotate the robot
        double pidOutput = pidController.calculate(navx.getAngle(), targetAngle);

        // Limit the maximum rotation speed
        pidOutput = Math.max(Math.min(pidOutput, MAX_ROTATION_SPEED), -MAX_ROTATION_SPEED);

        // Control the robot's rotation using the PID output
        m_robotDrive.driveCartesian(0, 0, pidOutput);

        // Stop the rotation when the  timeout occurs
        if ( rotationTimer.hasElapsed(TIMEOUT_SECONDS)) {
            rotationTimer.stop();
            RotateActive = false;
        }

      }
  }
   

  /** This function is called 
   * once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
