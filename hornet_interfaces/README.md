# Shared interface package , no runtime node

`hornet_interfaces` uses ament_cmake/rosidl to generate three message types: `StereoObservation`, `ObjectPosition`, and `SourceSelection`. The calibration/depth packages otherwise use Python. The messages define typed contracts so members can work independently.

Build with `colcon` before using the generated Python imports. See [docs/interfaces.md](../docs/interfaces.md) for exact field meaning, topic names, frames, QoS, timestamps, missing-data behavior and source handover. Review changes with every affected producer and consumer owner before merging. Do not change an input/output type in just one node.
