# SwerveBase
Base code for FRC Team 8726 Robots. Code for specific bots will be forked from this repository.

## Using StatefulSubsystem
- TODO: Someone give nice docs on this

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
