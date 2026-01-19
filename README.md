# SwerveBase
Base code for FRC Team 8726 Robots. Code for specific bots will be forked from this repository.

## Using StatefulSubsystem and the new state system

- Hello to the control peoples. This is a guide to using stateful subsystem

Stateful subsystem is an extension of subsystem base, something that WPILIB gives us so we can have access to basic commands in our subsystems, such as periodic(), which runs periodically, meaning once every 0.02 seconds. Stateful subsystem adds a couple new commands to the subsystem base which are useful. 

What StatefulSubsystem Adds

supplier
A pending command that we want to run next.

activeCommand
The command that is currently running for this subsystem.

executionTrigger
A Trigger that automatically schedules the next command when one is supplied.

We don't mess with these directly but instead interact with 
```java
public void runNextCommand(Command toRun, boolean runsWhenDisabled)
```
This command takes 2 parameters. The first one is the command you want to run and the second one is a boolean which sets if it runs when disabled. (You usually want this for state commands). First it cancels the currently running command (if one exists) so that no more than one command can be running on the subsystem which is very important. It then sets the next command to start running. 

We use it to both schedule commands in subsystems and set new states. Example of setting a new state with this system:
```java
robot.runNextCommand(new ManualShoot(robot), false);
```
Example of a subsystem command with this system: 
NOTE: For writing subsystems, to prevent threading errors, never return commands. Only return void methods as shown below
```java
public void startExampleCommand() {
  runNextCommand(new InstantCommand(() -> {
      controlEffort = pidController.calculate(0.0);
  }), false);
}
//Instead of what we would have done last season:
//DO NOT DO THIS PLEASE
public Command startExampleComand() {
  return new InstantCommand(() -> {
      controlEffort = pidController.calculate(0.0);
  });
}
```
We also have getCurrentCommandName which returns the name of the command that is currently running, initinitSendable which adds the command name to a sendable for debugging and driver dash stuff, and putOnDashboard which puts the subystem name on the dashboard. 

The Robot Container

Why RobotContainer Is a StatefulSubsystem
RobotContainer is our new state machine/state manager. You will notice that all the states take take the robot container as a parameter and this is so that whenever we change states from within another state, we do it on the robots command sequence so we can only have 1 state running at the same time. 
Example of changing states within a state:
```java
if (controller.getYButtonPressed()) {
    robot.runNextCommand(new ManualIntake(robot), false); //robot is the robot container
}
```

The State System

Currently, all of our state logic is in 1 file called State.java which has multiple classes. The first class we will have is StateVariables which allows an easy way to transmit global data, or any data which needs to be accessed and changed in multipl states. (Ex: Lunite Count from BBots SY 25-26) This is a simple public class where we can put all the variables that we will need to access throughout states. 

After that, each of the following classes is their own state which has multiple methods. All of the states extend StateBase which is an extension of command. This means that our states are basically commands in which have 3 properties that we care about. Initialize, Execute, and End. The initialize part runs once at the beginning and we call our own state method, onStateEnter(), in it. This means that when we make our own states, we can put anything we want to run in the beginning one time in the onStateEnter() and it will be called automatically when we enter the state. Next, we have execute which runs periodically. In it, we call our command periodic(), means until the command, or in this case state, ends, we will keep running this method so anything like that can go in periodic(). Similarly, we call onStateExit() in End which means when we exit the state we will call this part 1 time. 

Writing States

Writing states isn't that hard of a task. To start, you need to make your constructer in which you will set the name of the state and instantiate the robot container. After that, you will create 3 methods: onStateEnter, periodic, and onStateExit. (read above for more info about these) In onStateEnter, one of the things you will need to do for every state will be setting the default command using the robot container's instance of the swerve subsystem. After that, you can set any initial commands, by calling the void methods from the robot container's instance of the subsystem, or variables you want or need. Then, in periodic, you will have to put controller bindings, using if statements, and any other periodic logic that you want it to constantly check. After that, if there is anything you want to reset then you can do it in onStateExit. Now you have a state :). 

Example of state extending StateBase:
(Just look at State.java)

## Publishing values to NetworkTables
***It is not recommended to ever call the SmartDashboard.put methods unless you to for code that is for some reason outside of a subsystem.***  
Instead, all subsystems extending from StatefulSubsystem (new robot architecture class) will be automatically put into NetworkTables under the path Subsystems/{subsystem name}. 
To put values you can see for debugging, override the initSendable method on the subsystem, and use the SubsystemBuilder to add properties as needed - these will be automatically
updated by calling the provided getter and setter functions. The setter function can be null. 
You can also do things like publish constants - see the WPILib docs on this class for more info (linked at the end of the section).

When overriding initSendable, *PLEASE* remember to call `super.initSendable()`.

If you want to publish values to NetworkTables that can be easily changed at runtime through dashboards, all you have to do is add a setter function when adding properties in initSendable.
This will automatically call the setter function if the value changes.
**You should prefix these values with `MutableValues/`. for example, `MutableValues/kP`. This is the prefix our dashboard will check for debug settable values.**
**You should still be able to set other values in glass if needed through - but putting them under a certain prefix will keep things cleaner.**

For any subsystems that need to put data in NetworkTables, call putOnDashboard() at some point to automatically publish them in the correct spot. This is easiest to do in the constructor of your subsystem.  

Additionally, see the documentation of the FieldPointDisplay class for information on how to register setpoints that
can be set from the driver dashboard or publish points that will appear on the dashboard.
Example:
```java
public MySubsystem() {
  super("MySubsystem");
  /* regular constructor code ... */
  putOnDashboard();
}
```
Example code overriding initSendable:
```java
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    // Constant values
    builder.addBooleanProperty("isAtSetpoint", pidController::atSetpoint, null);

    // Mutable values
    builder.addDoubleProperty("MutableValues/kP", pidController::getP, pidController::setP);
    builder.addDoubleProperty("MutableValues/kI", pidController::getI, pidController::setI);
    builder.addDoubleProperty("MutableValues/kD", pidController::getD, pidController::setD);
  }
```

Relevant WPILib docs pages:  
Sendable - https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/util/sendable/Sendable.html  
SendableBuilder - https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/util/sendable/SendableBuilder.html
