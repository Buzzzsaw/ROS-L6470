# Nodes to control and test the inter-pupillary rail stepper

Nodes :
  * stepper_controller : Control the stepper motor by issuing commands to this node. The commands are listed below.

## Stepper Controller node

Start node:
```
rosrun stepper_controller stepper_controller_node.py
```

This node listens to publishers on:
 * /motor/inter_eye/command : issue commands to the stepper motor. The motor will also publish useful state information such as when it hits a limit switch.

The launch file will simply start the node:
```
roslaunch stepper_controller stepper_controller.launch
```

### Messages format
The messages are encoded in JSON. Each message represents a command to run on the stepper motor.

#### run command
```
{
    "command" : "run",
    "parameters" : {
        "direction" : "clockwise",
        "speed" : 800.0
    }
}
```

##### direction (string)
Used to indicate the direction of the rotation:
 * clockwise: the stepper will run in a clockwise direction
 * counter-clockwise: the stepper will run in a counter-clockwise direction

Using a different direction name will result in the command being ignored.

##### speed (float)
Used to indicate at which speed the stepper will run.
This value should be between the configured MIN_SPEED (default 0 step/s) and MAX_SPEED (default 991.8 step/s). Otherwise, the stepper's speed is clamped to one of these values.

#### move command
```
{
    "command" : "move",
    "parameters" : {
        "direction" : "clockwise",
        "steps" : 450
    }
}
```

##### direction (string)
Used to indicate the direction of the rotation:
 * clockwise: the stepper will run in a clockwise direction
 * counter-clockwise: the stepper will run in a counter-clockwise direction

Using a different direction name will result in the command being ignored.

##### steps (int)
Used to indicate the number of microsteps by which the stepper will move.
The steps value is in agreement with the selected step mode; the parameter
value unit is equal to the selected step mode (full, half, quarter, etc.).

#### goTo command
```
{
    "command" : "goTo",
    "parameters" : {
        "position" : 680
    }
}
```

##### position (int)
Used to indicate at which position the stepper will go to.
Note that the position value is always in agreement with the selected step mode; the
parameter value unit is equal to the selected step mode (full, half, quarter, etc.).

#### stop command
```
{
    "command" : "stop",
    "parameters" : {
            "type" : "soft"
    }
}
```

##### type (string)
Used to indicate the type of stop to apply to the stepper:
 * hard: the stepper will stop immediately
 * soft: the stepper will stop after a deceleration as configured by the DEC register

Using a different stop type name will result in the command being ignored.

## SPI Notes :
The SPI device consisting of the stepper can be either `/dev/spidev0.0` or `/dev/spidev0.1`. This has to be modified accordingly in:

```
stepper_controller/src/stepper_controller_node.py
```
