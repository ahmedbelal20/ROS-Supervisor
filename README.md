# ROS Module Supervisor

A Python package designed to orchestrate and manage modular ROS systems based on system states. This package provides an abstraction layer for launching, shutting down, monitoring, and dynamically switching between predefined sets of ROS modules. It simplifies state-dependent node control using JSON-based configurations and heartbeat monitoring for robust execution.


# 📦 Features

- Load and manage sets of ROS modules from JSON configuration.
- Launch and shutdown ROS launch files programmatically.
- Monitor module responsiveness via heartbeat topics.
- Automatically restart unresponsive or offline modules.
- Strict status checks with structured exception handling.
- Built-in ROS logging support for lifecycle transparency.


# 📁 Structure

```
your_repo/
├── module_handler.py # Core module manager
├── module.py # Individual module lifecycle and monitoring
├── supervisor_exceptions.py # Custom exception definitions
└── config/
  └── modules.json # JSON definitions for module sets
```


# 🧠 Concepts

## Modules
Each "module" is a wrapper around a specific launch file within a ROS package. Modules may optionally publish heartbeat messages to confirm liveliness.

## States
System states are represented as keys in a JSON file, each containing a list of module definitions required to be active in that state.


# 🔧 JSON Configuration

Example structure:
```json
{
  "state_1": [
    {
      "name": "module_a",
      "pkg": "your_package",
      "launch_file": "module_a.launch",
      "heartbeats_topic": "/module_a/status"
    }
  ],
  "state_2": [
    {
      "name": "module_b",
      "pkg": "your_package",
      "launch_file": "module_b.launch",
      "heartbeats_topic": "/module_b/status"
    }
  ]
}
```


# 🚀 Usage

1. Initialize the Handler
```python
from your_package.module_handler import ModuleHandler
handler = ModuleHandler()
```

2. Load Module Set.
```python
handler.parse_json("/path/to/modules.json")
handler.clear_and_load("state_1")
```

3. Launch All Modules
```python
handler.launch_all(delay=2)
```

4. Monitor and Maintain Online Status
```python
handler.keep_online()
```

5. Retrieve Module States
```python
states = handler.get_states()
for s in states:
    print(f"{s.name}: {s.status}")
```

6. Shutdown All Modules
```python
handler.shutdown_all()
```


#⚠️ Exceptions

- ```HandlerInitError```
- ```HandlerStatusError```
- ```HandlerLaunchError```
- ```HandlerShutdownError```
- ```HandlerLoadError```
- ```ModuleStatusError```
- ```ModuleLaunchError```
- ```ModuleShutdownError```

All exceptions provide meaningful messages for debugging and status validation.

# 📚 Dependencies
- Python 3.6+
- ROS (tested with ROS Noetic)
- ```rospy```, ```roslaunch```
- Custom message type: ```roar_msgs/ModuleStatus``` - Can be edited in source code.
