# SwerveBase
Base code for FRC Team 8726 Robots. Code for specific bots will be forked from this repository.

## Using StatefulSubsystem and the new state system

- Hello to the control peoples. This is a guide to using the 8726 StatefulSubsystem framework 

StatefulSubsystem is an extension of SubsystemBase. In general, the programming is similar, with the periodic() function and related methods still being the same. The difference shows in how commands are executed.

Before:
- Subsystems expose public functions which return commands that are used by Trigger objects
```java
new Trigger(controller.a().onTrue(johnSubsystem.doSomethingCommand())); //where doSomethingCommand returns a Command
```
After:
- Subsystems can themselves call runNextCommand or have it called on them by other objects, upon which the new command will become that subsystem's "state"
```java
public void doSomethingCommand() { //This would be within JohnSubsystem.java
  runNextCommand(new InstantCommand(() -> {
      controlEffort = pidController.calculate(0.0);
  }), false);
}
```
runNextCommand command takes 2 parameters. The first one is the Command you want to run and the second one is a boolean which sets if it runs when disabled. When called, the function cancels the currently running command (if one exists) so that no more than one command can be running on the subsystem which is very important. It then sets the next command to start running. If runWhenDisabled is set to false, then runNextCommand will do nothing unless the robot is enabled.

runNextCommand also schedules robot-wide states by calling runNextCommand on an instance of RobotContainer (note that there should be only one)
```java
robot.runNextCommand(new ManualShoot(robot), false);
```

Other Functions:
- getCurrentCommandName returns the name of the command that is currently running as a String for debugging purposes.
- initSendable adds the command name to a sendable for use with Aluminum
- putOnDashboard puts the subystem name on the dashboard. 
- Always call super(String subsystemName); in the constructor of StatefulSubsystems, otherwise they won't work
- After super, ALWAYS call SmartDashboard.putData("Subsystems/" + subsystemName, this); to ensure that Aluminum functions properly

The Robot Container

Why RobotContainer Is a StatefulSubsystem
RobotContainer is our new state machine/state manager. You will notice that all the states take the robotContainer as a parameter and this is so that whenever we change states from within another state, we do it on the robots command sequence so we can only have 1 state running at the same time. Note that the same RobotContainer instance is passed between every state to ensure state consistency

Example of changing states within a state:
```java
if (controller.getYButtonPressed()) {
    robot.runNextCommand(new ManualIntake(robot), false); //robot is the robot container
}
```

The State System

Currently, all of our state logic is in 1 file called State.java which has multiple classes. The first class we will have is StateVariables which allows an easy way to transmit global data, or any data which needs to be accessed and changed in multiple states. (Ex: Lunite Count from Bunnybots SY 25-26) This is a simple public class where we can put all the variables that we will need to access throughout states. 

After that, each of the following classes is their own state which has multiple methods. All of the states extend StateBase which is an extension of command. This means that our states are basically commands in which have 3 properties that we care about. Initialize, Execute, and End. The initialize part runs once at the beginning and we call our own state method, onStateEnter(), in it. This means that when we make our own states, we can put anything we want to run in the beginning one time in the onStateEnter() and it will be called automatically when we enter the state. periodic() handles real-time operations and controller bindings for the state. onStateExit() is called when the state ends. 

Writing States

Write any and all new states in State.java as classes which extend StateBase. Create a constructor which calls setName(String stateName), which defines the name of the robot state. After that, you will create 3 methods: onStateEnter(), periodic(), and onStateExit(). (read above for more info about these) In onStateEnter, one of the things you will need to do for every state will be setting the default command using the robot container's instance of SwerveSubsystem. Some states will prefer slowmode or autoalign, while others would prefer standard driving functionality.

For examples, look to State.java

## Publishing values to NetworkTables
***It is not recommended to ever call the SmartDashboard.put methods unless you to for code that is for some reason outside of a subsystem.***  
Instead, all subsystems extending from StatefulSubsystem (new robot architecture class) will be automatically put into NetworkTables under the path Subsystems/{subsystem name}. 
To put values you can see for debugging, override the initSendable method on the subsystem, and use the SubsystemBuilder to add properties as needed - these will be automatically
updated by calling the provided getter and setter functions. The setter function can be null. 
You can also do things like publish constants - see the WPILib docs on this class for more info (linked at the end of the section).

When overriding initSendable, *PLEASE* remember to call `super.initSendable(builder)`.

If you want to publish values to NetworkTables that can be easily changed at runtime through dashboards, all you have to do is add a setter function when adding properties in initSendable.
This will automatically call the setter function if the value changes.
**You should prefix these values with `MutableValues/`. for example, `MutableValues/kP`. This is the prefix our dashboard will check for debug settable values.**
**You should still be able to set other values in Glass if needed, but putting them under a certain prefix will keep things cleaner.**

Additionally, see the documentation of the FieldPointDisplay class for information on how to register setpoints that
can be set from the driver dashboard or publish points that will appear on the dashboard.
Example:
```java
public MySubsystem() {
  super("MySubsystem");
  /* regular constructor code ... */
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
