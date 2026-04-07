# Chapter 2: ROS2 Basics → Dora Dataflow Paradigm

## Concept Mapping

| ROS2 Concept | Dora Equivalent | Notes |
|-------------|----------------|-------|
| Node | Operator (Python script) | Each operator has a `main()` with event loop |
| Topic (pub/sub) | Dataflow connections | Defined in YAML, typed via PyArrow |
| Service (req/resp) | Send output → receive input | No built-in service pattern; emulated via paired outputs |
| Action (long-running) | Trajectory execution loop | Executor sends status updates, caller polls |
| Parameter server | Environment variables | `ROBOT_CONFIG_MODULE` env var per operator |
| TF2 (transform tree) | Config-based FK | ❌ No dynamic TF tree |
| Launch file | Dataflow YAML | Declarative node graph |
| Message types | PyArrow arrays + JSON | `pa.array()` for numerics, JSON bytes for structs |

## Key Differences

1. **No middleware**: Dora uses shared memory + zero-copy, not DDS
2. **Single process per operator**: Each operator is a Python process
3. **No dynamic discovery**: All connections declared in YAML at startup
4. **No TF tree**: Forward kinematics computed locally in each operator from config

## Dora Operator Pattern

```python
from dora import Node
import pyarrow as pa

def main():
    node = Node()
    for event in node:
        if event["type"] == "INPUT":
            if event["id"] == "my_input":
                data = event["value"].to_numpy()
                result = process(data)
                node.send_output("my_output", pa.array(result))
        elif event["type"] == "STOP":
            break
```

## Gap: No Dynamic TF Tree

ROS2's TF2 provides a dynamic tree of coordinate frames. Dora-MoveIt2 uses static config-based transforms (`LINK_TRANSFORMS` in robot config). For dual-arm, each arm's base frame is defined in `ARM_BASE_TRANSFORMS`.
