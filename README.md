# Competition_Code_2020

    Constants.java - contains the inputs and values for everything
    
    Main.java - FRC file that defines the robot (NEVER TO TOUCH)
    
    Robot.java - uses certain commands ONLY during certain times of the match
    
    RobotContainer.java - contains the button definitions, the stick (controller) definitions, and the trajectory code

/* Commands */

Autos
          
    Auto1.java - commandGroup used for complex autonomous (currently twitchy)
    
    TurretAuto.java - command to center the turret during Autonomous

Colors
                
    Colored.java - command to turn control panel wheel to correct color

Drives
                
    AutoDrive.java - command to drive robot during Autonomous

    Shift.java - command to shift between high and low gear

    StickDrive.java - command to drive robot manually with controller

HarvestandFeed

    Agitator.java - command to kick ball into the tunnel for the shooter

    Harvest.java - command to use the harvester wheels

    Kick.java - command to kick the balls up towards the shooter

Sensors

    sensDown.java - command to lower the sensitivity of the robot's turn

    SensorReset.java - command to reset the sensors at the beginning of Autonomous

    sensUp.java - command to raise the sensitivity of the robot's turn

TurretandShooter

    hoodSolenoid.java - command to raise and lower the hood
    
    ShooterMotorHigh.java - command to shoot ball with 100% power and activate the topKick motor
    
    ShooterMotorLow.java - command to shoot ball with 85% power and activate the topKick motor
    
    TurretVision.java - command to center the turret with a button

/* Subsystems */

    Colorings.java - subsystem to control parts of the robot dealing with the control panel
    
    DriveTrain.java - subsystem to control parts of the robot dealing with the chassis, sensitivity, encoders, and gyro
    
    Harvester.java - subsystem to control the harvester and feed
    
    Turret.java - subsystem to control the turret and shooter
    
    Vision.java - subsystem to control the vision tracking
