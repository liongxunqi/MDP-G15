# Android delivery and synchronization

Only Android is changed. The authoritative RPi reference is
`origin/protocol-v3-fu`, fetched and checked at `90478b6` (robot-coordinate update).
It is read directly from Git, not merged, modified or deployed by this work.

## Obstacle map

The integrated app sends one ordered batch after each completed map edit and each
connection/reconnection: `CLEAR`, then `OBSTACLE,id,x,y,FACING` for every current
obstacle. A deleted obstacle is absent from that replacement list. Reset sends
only `CLEAR`. Placement, dragging, faces, undo and redo use the same path.
Coordinates are multiplied by ten because the existing RPi divides them by ten;
directions use NORTH/EAST/SOUTH/WEST, or SKIP for an unset face. IDs are preserved.
The reusable arena module's original delta codec remains available to other hosts.

Offline edits are retained in saved Android arena state. Reconnecting publishes
the latest full map, rather than replaying old edits. Received TARGET/ROBOT/status
messages do not echo back as edits. Recognition IDs and physical robot pose remain
incoming telemetry; the RPi has no API to set these from the tablet. Dragging the
robot marker is a local placement edit, not a physical movement command.

## All outgoing commands

All application messages are newline-delimited, matching the existing RPi stream
reader. A map batch is written as one queue item so a D-pad or Begin command cannot
interleave its lines. D-pad/Stop/Begin payload tokens remain unchanged. The queue
accepts commands only during a live connection and drops pending messages when it
ends; failed writes and queued movement are never automatically replayed. Dropped,
rejected and failed messages are logged alongside QUEUED, TX and RX events.

## What Android alone cannot guarantee

- TX confirms a local socket write, not RPi receipt, application, or STM execution.
  The UI explicitly leaves map delivery unconfirmed; no synthetic ACK is used.
- CLEAR plus obstacle lines is not an atomic transaction on the unchanged RPi.
  Loss mid-batch can leave a partial map until reconnection republishes it. Its
  debounce/path-planning races and already-running mission behavior are unchanged.
  Do not infer a safely replanned route from an Android edit during execution.
- The protocol-v3-fu task1.py handles BEGIN but does not handle the tablet's D-pad
  strings. Android retains them; matching deployed RPi/STM support must be verified
  by their owner. No alternate movement mapping is guessed.
- There are no command IDs, acknowledgements, map readback, or execution IDs in
  the current RPi interface. Guaranteed convergence and exactly-once execution
  require those receiver capabilities and cannot be provided by Android alone.
- Protocol v3 sends `ROBOT,x,y,dir` after STM OK only if the completed segment has
  a planner `dirs` entry. Android accepts integer cell coordinates and N/E/S/W (or
  full direction names). This reports expected segment-end pose, not measured
  continuous movement. The branch's PC stub currently returns `dirs: []`, so it
  produces no robot-position updates until a planner supplies those entries.

## Verification

`ObstacleSyncTest`, `ArenaViewModelTest`, `SessionOutboxTest`, and
`ChecklistProtocolTest` cover all map edits, reconnect/state restoration, incoming
telemetry without echoes, queue expiry for every command, and framing/order.
The integrated emulator test covers offline edits and live reconnect publication.

Fetch `origin` first, then run `python3 protocol-tests/check_rpi_compatibility.py`
from this directory. It prints the tested branch/commit and reads the actual
protocol-v3 receive methods with `git show`; it does not fall back to main.
Four tests cover map replacement/deletion/units, Begin, the missing D-pad/Stop
handler, and robot telemetry with/without planner directions, using fake I/O.
These are compatibility checks, not Bluetooth or physical robot tests.
