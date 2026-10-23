# robotop

A robot-debugging TUI. SSH-friendly, no GUI required. A single menu-driven Textual app that gathers the information you most often need on-site into one place, **no rviz / Foxglove required — usable in a pure terminal**.

## Features

- **🩺 System status dashboard (`dash`)** — a live five-card grid showing the key subsystems at a glance:
  - **MC Action** current motion behavior (STAND_DEFAULT / LOCOMOTION_DEFAULT, etc.) + running/transition status
  - **System** SystemState business state (IN_READY / IN_MOVE, etc.)
  - **Faults / HDS** active diagnostic codes / alert codes / post-proc signal counts + the most recent code
  - **PMU / Battery** PMU temperature, Orin voltage/current, battery level, error bits
  - **Orin** Jetson Orin CPU/GPU/MEM/temperature (published by a separate `orin-monitor` process)
- **📜 Log aggregation (`logs`)** — unifies multiple log sources into one screen:
  - `~/.ros/log/<latest_session>/` (ROS 2 node logs; the default source, skip with `--no-ros`)
  - other files via `--add-path FILE` (e.g. the AIMA log `~/.aima/log/aima.log`, AIMRT app logs)
  - optional subscription to the `/rosout` topic
  - multiple timestamp formats parsed automatically, time zones unified
  - **live mode** + **historical query** (`--since 10m`, `"yesterday 18:00"`, absolute times)
  - filter DSL: `level>=WARN node~=slam connect`
- **🔌 Communication view (`comm`)** — lists the topics / services in the current ROS graph and measures the rate of a selected topic **on demand** (zero-load startup); QoS auto-matches the publisher. Hidden topics are filtered by default, matching `ros2 topic list`.
- **⚠️ Faults / HDS (`faults`)** — a dedicated Health Diagnostics System screen:
  - **Live**: three tabs (Diag / Alert / PostProc) as live tables
  - **History**: calls the `GetHdsEventHistory` service, filters by time window / state / diagnostic code, with pagination
- **📊 Orin monitor publisher (`orin-monitor`)** — a standalone ROS 2 node that wraps the Jetson Orin's CPU/GPU/MEM/temperature into `diagnostic_msgs/DiagnosticArray` and publishes to `/orin/stats` (via psutil + tegrastats); usable by dash, recordable into a bag, and subscribable by other nodes.

**Unified entry point:** `ros2 run robotop robotop` with no arguments opens the menu — arrow keys to pick a screen, Enter to open, `q` to return to the menu from a sub-screen, `q` to quit from the menu, `Ctrl+C` to force-quit anywhere. You can also use the classic direct subcommands: `robotop dash` / `robotop logs --since 1h`, etc.

> ⚠ **Before first use**: for the dash Orin card to have data, an `orin_monitor_node` process must be running in the background (it is not embedded in dash — it is a standalone publisher that any node can subscribe to). Using systemd auto-start on boot is recommended; see the [`orin-monitor` section](#orin-monitor--orin-resource-publisher). Quick check:
> ```bash
> ros2 run robotop orin_monitor_node     # run in a separate terminal, keep it up
> ```

---

## Installation

The workspace already provides `aimdk_msgs`, `rclpy`, and `diagnostic_msgs`. Install the runtime Python packages (typer / textual / dateparser) in your user directory:

```bash
pip3 install --user typer textual dateparser
```

Build:

```bash
cd ~/aimdk
colcon build --packages-select robotop
```

> **Note: do not use `--symlink-install`.** typer's entry point goes through `importlib.metadata`, but symlink mode only generates an egg-link, which the standard library does not resolve, giving `PackageNotFoundError: No package metadata was found for robotop`.

Every new terminal needs to source:

```bash
source /opt/ros/humble/setup.bash
source ~/aimdk/install/setup.bash
```

---

## Command overview

Running `ros2 run robotop robotop` with no subcommand opens the **menu** (MenuScreen) by default, where arrow keys + Enter pick Dashboard / Logs / Comm / Faults. Press `q` in a sub-screen to return to the menu, `q` on the menu to quit, and `Ctrl+C` to force-quit anywhere.

| Command | Function |
|---|---|
| `robotop` or `robotop menu` | top-level menu, unified entry point |
| `robotop dash` | system status grid (MC / System / Faults / PMU / Orin) |
| `robotop logs` | aggregate ROS / app logs, live or historical |
| `robotop comm` | topic/service list + on-demand rate |
| `robotop faults` | HDS diagnostics/alerts live table; `--history` uses GetHdsEventHistory |
| `robotop orin-monitor` | publishes `/orin/stats` (`diagnostic_msgs/DiagnosticArray`) |

All invoked as:

```bash
ros2 run robotop robotop                # open the menu (recommended)
ros2 run robotop robotop <subcmd> [options]  # go straight to a screen
ros2 run robotop orin_monitor_node           # standalone node, good for systemd
```

---

## dash — system status

> ⚠ **Prerequisite: the Orin card needs `orin_monitor_node` running in the background** — this publisher is not part of dash itself; start a separate process to publish `/orin/stats`. See the [`orin-monitor` section](#orin-monitor--orin-resource-publisher); the cleanest way is systemd auto-start on boot. **For a quick check, use a separate terminal:**
> ```bash
> ros2 run robotop orin_monitor_node      # foreground, Ctrl-C to exit
> ```
> Not started → the Orin card shows `[stale - run \`ros2 run robotop orin_monitor_node\`]`.
> The other four cards need no extra process.

```bash
ros2 run robotop robotop dash
```

Five cards (3x2 grid, with the Orin card spanning two columns):

- **MC Action** — `/aimdk_5Fmsgs/srv/GetMcAction`, shows the current MC mode (e.g. `STAND_DEFAULT`) + status (`RUNNING` / `TRANSITION` / `IDLE`)
- **System** — `/aimdk_5Fmsgs/srv/GetSystemState`, shows the business state name + IN_READY/IN_MOVE, etc.
- **Faults / HDS** — subscribes `/aima/hds/{diag_code_list,alert_code_list,post_proc/signal}`, shows active diag/alert counts + the most recent code (if the HDS types aren't in the aimdk_msgs install, shows a hint)
- **PMU / Battery** — subscribes `/aima/hal/pmu/state` (QoS: Best Effort + Transient Local)
- **Orin** — subscribes `/orin/stats`, **requires `orin_monitor_node` running first (see prerequisite)**

**Keys:** `q` back to menu (quits in standalone mode), `r` refresh now, `Ctrl+C` force-quit. A card showing `[stale]` means its data source isn't connected.

---

## logs — log aggregation

### Live (default)

```bash
ros2 run robotop robotop logs --follow
ros2 run robotop robotop logs -f --rosout                          # add /rosout subscription
ros2 run robotop robotop logs -f --add-path ~/.aima/log/aima.log   # add the AIMA log
ros2 run robotop robotop logs -f --no-ros                          # skip the ROS session source
```

Default source: the latest `.log` of the latest ROS session (`~/.ros/log/<latest_session>/`). AIMA, AIMRT, and other logs are added explicitly with `--add-path`.

### Historical query

```bash
ros2 run robotop robotop logs --since 1h
ros2 run robotop robotop logs --since 7d --level WARN
ros2 run robotop robotop logs --since "2026-06-04 14:00" --until "2026-06-04 16:00"
ros2 run robotop robotop logs --since 30m --grep "fall"
```

`--since` / `--until` accept:

- relative: `30s` / `10m` / `1h` / `2d`
- `now`
- natural language: `"yesterday 18:00"`, `"2026-06-04"`
- absolute: `"2026-06-04 14:00"`

### TUI keys

| Key | Action |
|---|---|
| `/` | enter the top filter input box |
| `Esc` | exit the filter box back to the log area |
| `Enter` | apply the current filter (inside the filter box) |
| `Ctrl+L` | clear the screen |
| `q` | quit |

### Filter DSL

Top input box, space-separated; a line shows only if **all terms match**:

| Expression | Meaning |
|---|---|
| `error connect` | message contains both "error" and "connect" (bare text = message substring, case-insensitive) |
| `level=ERROR` | level exactly equals ERROR |
| `level>=WARN` | WARN and above (order: DEBUG < INFO < WARN < ERROR < FATAL) |
| `node~=slam` | source (node/file) contains "slam" |
| `source~=aima` | same as above (alias for `node~=`) |

The bottom status bar shows `total=X  shown=Y` so you can watch the filter hit rate.

### Parsed formats

| Source | Sample |
|---|---|
| ROS node log | `[INFO] [1780226934.705] [node]: msg` |
| ROS launch log | `1779783007.674 [INFO] [launch]: msg` |
| AIMA | `2026-06-02 15:50:36,921  DEBUG  msg` |
| AIMRT app log | `[2026-05-20 13:39:27.156][Info][2145][file.cc:47] msg` |

Unmatched lines: live follow mode passes them through as-is (marked `unknown`); historical scan mode skips them.

---

## faults — HDS faults

Live/historical view of the Health Diagnostics System (HDS).

### Live (default)

```bash
ros2 run robotop robotop faults
```

Three tabs, each mapping to a topic:

- **Diag** — `/aima/hds/diag_code_list` (`DiagnosticInfoArray`): columns first / last time, type (NORMAL/TRIGGER+/TRIGGER-), module_id, code, diag_code, info
- **Alert** — `/aima/hds/alert_code_list` (`AlertCodeArray`): first / last / code string
- **PostProc** — `/aima/hds/post_proc/signal` (`PostProcSigArray`): ts, diag_code, various signal bits

The bottom status bar shows the current count per tab. Press `q` to quit.

### History

```bash
ros2 run robotop robotop faults --history --since 1h
ros2 run robotop robotop faults --history --since 1d --state active
ros2 run robotop robotop faults --history --diag-code 0x6300000001
ros2 run robotop robotop faults --history --alert-code battery_low --asc
```

Calls `/aimdk_5Fmsgs/srv/GetHdsEventHistory`, filter parameters:

| Parameter | Meaning |
|---|---|
| `--since` / `--until` | time window, parsed like `logs --since` (`10m` / `1h` / `"yesterday 18:00"`) |
| `--state all/active/recovered` | state filter |
| `--diag-code N` | filter by diagnostic code (repeatable) |
| `--alert-code STR` | filter by alert string (repeatable) |
| `--limit N` | max records to return (default 200) |
| `--asc` | ascending time order (default descending) |

Returned fields: id / first / last / duration / recovered / diag_code / alert_code / location.

> ⚠ Requires the `aimdk_msgs` install to contain `DiagnosticInfoArray`, `AlertCodeArray`, `PostProcSigArray`, and `GetHdsEventHistory`. The current prebuilt aimdk_msgs in the workspace **does not** include these — add the corresponding HDS `.msg`/`.srv` files to the aimdk_msgs interface tree and rebuild. When the types aren't installed, the `robotop dash` Faults card shows hint text and `robotop faults` exits with an error.

---

## comm — communication view

```bash
ros2 run robotop robotop comm                    # defaults to match `ros2 topic list`
ros2 run robotop robotop comm --include-hidden   # show all, including `_*` hidden paths
```

Two tabs:

- **Topics** — columns: `M` / `topic` / `type` / `pub` (publisher count) / `sub` (subscriber count) / `hz` / `count`
- **Services** — columns: `service` / `type`

Data sources: `get_topic_names_and_types()` / `get_service_names_and_types()`, querying the ROS graph just like `ros2 topic list` / `ros2 service list`. Auto-refreshes every second (the table isn't rebuilt while the list is stable, so the cursor and scroll position aren't lost), **zero-load startup — it never actively subscribes to any topic**.

### M column meaning

| Marker | Meaning |
|---|---|
| (empty) | not measuring |
| `●` | subscribing and measuring; `hz` / `count` columns refresh live |
| [red]`✗`[/red] | **pressing `m` to start measuring failed**; when the row is focused, the bottom status bar **keeps** the full error text |

The most common failure is **the type not being in the aimdk_msgs install** (some internal sensor/diagnostic types are not shipped in the prebuilt package): the service is running and the topic is in the graph, but no Python class was generated in the install, so `get_message()` throws AttributeError. The fix is the same as for HDS — add the corresponding `.msg` files to the aimdk_msgs interface tree and rebuild aimdk_msgs.

### Keys

| Key | Action |
|---|---|
| `m` | toggle measuring on the selected row: first press starts subscribe + count, second press stops and destroys the subscription. Failure shows ✗ |
| `M` | stop measuring on all topics at once |
| `/` | enter the top filter bar (substring match on name or type) |
| `Esc` | exit the filter back to the table |
| `r` | re-enumerate immediately |
| `q` | quit |

### Rate-measurement details

- **QoS adaptation**: uses `get_publishers_info_by_topic(topic)` to read the publisher's actual QoS and creates the subscription with matching Reliability + Durability. `Best Effort + Volatile`, `Reliable + Transient Local`, etc. all keep up. With no publisher, falls back to `Best Effort + Volatile + KeepLast(5)`.
- **hz computation**: a rolling window of the most recent 50 messages, `(N-1) / (t_last - t_first)`
- measuring large messages (PointCloud2 / Image) **really consumes CPU and bandwidth** — only measure the ones you care about
- on TUI exit it automatically runs `stop_all`, leaving no residual subscriptions

### Hidden-topic filtering

Default behavior matches `ros2 topic list`: **any path segment starting with `_`** counts as hidden and is filtered out. For example `/foo/_internal/bar` and other underscore-prefixed segments are hidden by default. `/aimdk_2Eprotocol/...` (the `_2E` embedded in the middle, not at the start of a segment) is not hidden and shows normally.

Add `--include-hidden` to show everything.

### Note: does not actively call services

The service table only lists name/type; it never actively calls any service. Some services are control hubs (MigrateSystemState / SetMcAction), and calling them by mistake could trigger actions. By MVP design, active calls are forbidden.

### Count difference vs `ros2 topic list`

May be slightly lower. `ros2 topic list` uses the background resident daemon (which discovered all peers long ago), while `robotop comm` starts a new rclpy node each time and spends the first 1-3 seconds on DDS discovery. After a few seconds it climbs to a close number.

---

## orin-monitor — Orin resource publisher

```bash
ros2 run robotop orin_monitor_node                      # 1 Hz, default
ros2 run robotop orin_monitor_node --rate 2             # 2 Hz
```

Data collection: **psutil + tegrastats** (no privileges required):

- `psutil` provides per-core CPU usage / memory used+total / memory percentage
- the `tegrastats --interval 1000` subprocess streams output in real time; regex parses out GPU usage, CPU/GPU/thermal temperatures, and total power (`VDD_IN`)

Publishes to `/orin/stats`, type `diagnostic_msgs/msg/DiagnosticArray`. Keys include: `cpu_pct_avg`, `cpu_cores`, `cpu_pct_core_0..N`, `gpu_pct`, `mem_used_mb`, `mem_total_mb`, `mem_pct`, `temp_cpu`, `temp_gpu`, `temp_thermal`, `power_total_mw`.

### Persistence: tmux (recommended, no sudo)

```bash
tmux new -d -s orin 'source /opt/ros/humble/setup.bash && source ~/aimdk/install/setup.bash && ros2 run robotop orin_monitor_node'
# view output: tmux attach -t orin  (Ctrl-b d to detach without killing the process)
# stop: tmux kill-session -t orin
```

tmux survives an ssh disconnect; it is lost on machine reboot.

### Persistence: systemd auto-start on boot (needs sudo)

```ini
# /etc/systemd/system/orin-monitor.service
[Unit]
Description=Orin stats publisher for robotop
After=network.target

[Service]
Type=simple
User=agi
ExecStart=/bin/bash -lc 'source /opt/ros/humble/setup.bash && source ~/aimdk/install/setup.bash && exec ros2 run robotop orin_monitor_node'
Restart=on-failure

[Install]
WantedBy=default.target
```

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now orin-monitor
systemctl status orin-monitor
journalctl -u orin-monitor -f
```

---

## Architecture

```
robotop/
├── cli.py                    typer entry point; no subcommand -> menu, with subcommand -> the matching Screen
├── main_app.py               RobotopMainApp + MenuScreen, holds the shared RosBridge / Collector / Inspector
├── ros_bridge.py             rclpy singleton + MultiThreadedExecutor background spin
├── orin_monitor_node.py      standalone ROS node (psutil + tegrastats provider)
├── status/
│   ├── collector.py          subscribes PMU / orin/stats / HDS, polls GetSystemState / GetMcAction
│   └── screen.py             Textual grid (MC / System / Faults / PMU / Orin)
├── logs/
│   ├── parsers.py            multi-format regex, LogLine dataclass (timezone-aware UTC)
│   ├── sources.py            LogSource Protocol + RosSession / Aima / FileTail / Rosout
│   └── screen.py             Textual + RichLog, filter DSL, live/historical dual mode
├── comm/
│   ├── inspector.py          rclpy node: enumerate topic/service + on-demand subscribe measuring (QoS-adaptive)
│   └── screen.py             Textual TabbedContent, Topics/Services two tabs
└── faults/
    ├── history.py            GetHdsEventHistory synchronous client
    └── screen.py             LiveFaultsScreen (three tabs) + HistoryFaultsScreen (static table)
```

**Key design:**

- **App / Screen layering** — each feature module is a Textual `Screen`, and the top-level `RobotopMainApp` manages them with a screen stack. Subcommands (`robotop dash`, etc.) use `_SingleScreenApp` to wrap a Screen into a single-screen App, sharing the same Screen classes as the menu's push-pop flow.
- **rclpy background-thread spin** — spinning directly inside Textual's async loop would block the UI, so a separate thread runs the executor and the UI pulls data via shared structures (snapshot / Queue).
- **Lazy-created + shared ROS resources** — `RobotopMainApp` holds `_collector` and `_inspector`; the collector is created only on first entering dash/faults, the inspector only on first entering comm; dash and faults share the same collector without duplicate subscriptions.
- **One producer thread per LogSource** — `follow()` uses a tail-like blocking loop, each pushing into a shared Queue; the Textual main loop pulls once every 100ms.
- **Interface first, types later** — HDS / some hal types aren't in the prebuilt aimdk_msgs; the code guards imports with try/except and the UI shows a "(types not installed)" hint; once the .msg/.srv are added and aimdk_msgs is rebuilt, **robotop connects automatically without a single line changed**.
- **DiagnosticArray as the Orin output format** — a standard type that rviz / Foxglove can display directly, with no custom msg needed.

---

## Typical scenarios

**General entry: the menu**

```bash
ros2 run robotop robotop          # arrow keys pick a screen -> Enter, q back to menu, Ctrl+C force-quit
```

**Robot power-on self-check**

```bash
# Terminal A: resident publisher
ros2 run robotop orin_monitor_node

# Terminal B: open the menu and pick Dashboard, or directly:
ros2 run robotop robotop dash
```

Whichever card is stale tells you which subsystem didn't start.

**Investigating an anomaly**

```bash
ros2 run robotop robotop logs --since 5m --level WARN
# in the TUI, press / then type `node~=<suspect module> error`
```

**Watching a single node**

```bash
ros2 run robotop robotop logs -f --rosout
# / then type `node~=<node name>`
```

**Measuring topic rate**

```bash
ros2 run robotop robotop comm
# / to filter by keyword, put the cursor on a row, press m to start measuring, press m again to stop
```

**Browsing HDS fault history**

```bash
ros2 run robotop robotop faults --history --since 1h --state active
```

---

## Known limitations

- **HDS / some custom hal types are not in the prebuilt aimdk_msgs install**.
  - the `robotop dash` Faults card shows hint text
  - `robotop faults` exits with an error
  - pressing `m` on those topics in `robotop comm` shows a [red]`✗`[/red] plus "type not installed"
  - fix: copy the corresponding `.msg/.srv` into the aimdk_msgs interface tree and rebuild aimdk_msgs. The code already guards with try/except, so it works as soon as the types are installed.
- DomainErrorState is scattered only in the hal joint state, not aggregated separately.
- Logs live follow only tails the single latest `.log` of the latest session; switching sessions requires a restart.
- After changing code you must `colcon build --packages-select robotop` (**not `--symlink-install`**, otherwise the typer entry point throws `PackageNotFoundError`).
