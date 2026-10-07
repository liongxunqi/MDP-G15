# Arena UI module

`arena-ui` is an embeddable Compose feature for the Android controller. It owns arena
rendering and editing, robot/target visualization, and the arena status panel. It does
not own Bluetooth, connection/reconnection, message framing or queues, device selection,
or manual movement commands.

## Arena model

- The arena defaults to 20 columns by 20 rows.
- Coordinates use a bottom-left origin: `x` increases left-to-right and `y` increases
  bottom-to-top.
- An obstacle occupies one cell and keeps a stable positive ID for the arena session.
- The robot has a 2.0 × 2.1-cell rotated footprint with decimal bottom-left
  reference coordinates and a north-zero clockwise heading in degrees. Operator
  dragging snaps the reference to integer coordinates and rejects collisions.
  Received poses are shown even when off-map/overlapping, with a warning. Later
  reports animate to their received endpoint; only D-pad commands create an
  explicitly estimated preview. Animation does not acknowledge a command.

## Messages

`ArenaViewModel.accept(message)` expects one complete application message and recognizes:

```text
STATUS,<text>
MSG,<text>
TARGET,<obstacleId>,<targetId>
TARGET,<obstacleId>,<targetId>,<N|E|S|W>
ROBOT,<decimal-x>,<decimal-y>,<degrees|N|E|S|W>
STATUS,START,<decimal-x>,<decimal-y>,<degrees|N|E|S|W>
```

Target messages also accept the briefing's `B`-prefixed obstacle labels (e.g. `B2`).

Unrelated messages are ignored. Malformed relevant messages leave arena state unchanged
and appear only as local arena feedback. Unknown target obstacle IDs do not create phantom
obstacles.

By default, completed local edits are submitted through `ArenaOutboundSink`:

```text
OBSTACLE,UPSERT,<id>,<x>,<y>,<N|E|S|W>
OBSTACLE,REMOVE,<id>
```

Creation, movement, and face changes use a complete upsert. Drag previews never send;
the final valid edit is submitted after release.

The integrated controller enables `ArenaViewModel.Factory(..., useRpiMapSync = true)`
and calls `connectionChanged(isConnected)`. This adapter replaces the full RPi map
with a `CLEAR` + obstacle batch after local edits and setup reconnects, using the existing
RPi units and full direction names. Active/uncertain runs and completed-result viewing
do not resend the map on reconnect. It does not claim acknowledged delivery.
See [Task 1 assessment](../TASK1_ASSESSMENT.md) for planning, starting, result retention
and the Android/PC evidence handoff.
See [Android/RPi delivery](../ANDROID_RPI_DELIVERY.md) for the protocol and limits.

## Host integration

Create one lifecycle-owned `ArenaViewModel` with `ArenaViewModel.Factory`, collect the
host's complete-message stream into `viewModel.accept(message)`, and provide a sink that
forwards outbound strings to the host's existing send queue. Render it with:

```kotlin
ArenaScreen(viewModel = arenaViewModel)
```

The host remains responsible for Bluetooth permissions, connection/reconnection,
packet framing, queueing, retries, and manual controls.
