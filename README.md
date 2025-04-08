#LIBPID - CONTROLLER

This library implements a simple PID (Proportional-Integral-Derivative) controller in C.

## PID Controller

A PID controller is a widely used control loop mechanism that calculates an error value as the difference between a desired setpoint and a measured process variable. It applies a correction based on proportional, integral, and derivative terms.

This type of controller is useful for maintaining stable and accurate control in various systems, such as robotics, automation, and embedded applications.

The main advantage of a PID controller is its ability to dynamically adjust its output to minimize the error over time, while its main drawback is the need for careful tuning of the `kp`, `ki`, and `kd` parameters.

> [!TIP]
> PID controllers are commonly used in embedded systems for precise control of motors, temperature regulation, and signal stabilization.

## Dependencies

This library uses [ArenaAllocator](https://github.com/eagletrt/libarena-allocator-sw/) for memory management. Make sure to initialize ArenaAllocatorHandler_t to use prev_errors array.

## Usage

To use this library, include the `pid-controller-api.h` header file in your program, declare a PID controller handler, and initialize it with the appropriate function before processing inputs.

```c

int main(void) {
    ArenaAllocatorHandler_t harena;
    arena_allocator_api_init(&harena);

    PidController_t pid;
    if (pid_controller_api_init(&pid, 1.0, 0.1, 0.01, 0.1, 10.0, &harena, 3) != PID_OK) {
        // Error handling
    }

    while (1) {
        float status = read_sensor_value();
        pid_controller_api_update(&pid, status);
        float output = pid_controller_api_compute(&pid);
        apply_output(output);
    }

    arena_allocator_api_free(&harena);
    return 0;
}
```

> [!IMPORTANT]
> Remember to free the arena memory at the end of the program, even if it never ends.

For more information, check the [examples](examples) folder.
