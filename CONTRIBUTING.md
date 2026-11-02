# Working as the P&L subteam

1. Pick a node-role folder and open an issue with the Node work template. Assign a real member through GitHub after agreeing ownership; no assignments are created by this scaffold.
2. Read the root README, your folder README, `docs/interfaces.md`, and `docs/frames.md` before implementing.
3. Develop on `feature/<node>-<change>` or another agreed branch. Preserve one logical node role per top-level folder. Put its scripts, config, package files and launch inside its folder. Shared message definitions live only in `hornet_interfaces`.
4. State inputs, outputs, frames, timestamps, QoS and invalid-data behavior in the node README. Interface changes require both producer and consumer review.
5. Include a small reproducible test or recorded-data validation with expected behavior. Do not commit large bags/weights/datasets; document controlled storage and versions.
6. Open a PR describing behavior, interface impact, validation and remaining hardware work. Use measured evidence for accuracy/performance claims. Keep synthetic results clearly labelled.

Initial integration order: camera + measured calibration → synchronized detector observations → depth geometry → localization/extrinsics → map-coordinate validation → handover tuning → bottom flow/plane localization. Mock inputs can unblock development, but they must not be confused with a working real-robot system.

Private-repository access and member invitations remain with the subteam lead. No external messages or invitations are sent by this setup.
