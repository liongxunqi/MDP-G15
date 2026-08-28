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
- The robot is rendered as a directional pose at one received coordinate; this module
  does not simulate movement or enforce a robot collision footprint.

## Messages

`ArenaViewModel.accept(message)` expects one complete application message and recognizes:

```text
STATUS,<text>
MSG,<text>
TARGET,<obstacleId>,<targetId>
TARGET,<obstacleId>,<targetId>,<N|E|S|W>
ROBOT,<x>,<y>,<N|E|S|W>
```

Unrelated messages are ignored. Malformed relevant messages leave arena state unchanged
and appear only as local arena feedback. Unknown target obstacle IDs do not create phantom
obstacles.

Completed local edits are submitted through `ArenaOutboundSink`:

```text
OBSTACLE,UPSERT,<id>,<x>,<y>,<N|E|S|W>
OBSTACLE,REMOVE,<id>
```

Creation, movement, and face changes use a complete upsert. Drag previews never send;
the final valid edit is submitted after release.

## Host integration

Create one lifecycle-owned `ArenaViewModel` with `ArenaViewModel.Factory`, collect the
host's complete-message stream into `viewModel.accept(message)`, and provide a sink that
forwards outbound strings to the host's existing send queue. Render it with:

```kotlin
ArenaScreen(viewModel = arenaViewModel)
```

The host remains responsible for Bluetooth permissions, connection/reconnection,
packet framing, queueing, retries, and manual controls.
