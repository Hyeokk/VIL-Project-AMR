# Point Cloud Merge: Principle and Implementation

## Why Merge

The M300 LiDAR has a vertical FoV of -10 deg to +60 deg. At a mounting height of ~0.7m, the -10 deg lower bound creates a blind zone: the ground within ~3.6m in front of the robot is invisible to the LiDAR.

```
            M300 LiDAR (top)
                 |
                 |  -10 deg lower limit
                 |     \
                 |      \
   ==============|=======\=========== ground
                          ^
                     ~3.6m blind zone
```

The S10 Ultra camera (120 x 80 deg FoV, mounted below the LiDAR) looks downward-forward and covers this gap.

---

## Data Flow

```
M300 LiDAR + IMU
     |
  FAST-LIO2 (odometry + motion compensation)
     |
  /cloud_registered_body          /lx_camera_node/LxCamera_Cloud
  frame: body                     frame: mrdvs_tof
  type: PointXYZI                 type: PointXYZRGB
  rate: 10 Hz                     rate: 10 Hz
  clock: M300 internal (~25,000s) clock: system (~1.77 billion s)
     |                                 |
     +------------ cloud_merger -------+
                        |
                  /merged_cloud
                  frame: body
                  type: PointXYZI
                  rate: 10 Hz (driven by M300)
                        |
                    GroundGrid
```

The M300 cloud is already in the `body` frame (FAST-LIO2 registers it). The S10 cloud is in the `mrdvs_tof` frame and must be transformed to `body` before merging.

---

## Merge Procedure

### Step 1: S10 Callback — Store and Cache TF

When an S10 cloud arrives, the node stores the message and records the wall-clock receive time. On the first message, it looks up the static TF `mrdvs_tof -> body` and caches it as a 4x4 matrix. This matrix encodes the physical mounting position of the camera relative to the robot body (0.35m forward, 0.35m up). Since all links in this TF path are static, one lookup is sufficient for the entire session.

### Step 2: M300 Callback — Merge and Publish

Every M300 message triggers the merge:

1. Copy all M300 points into the output cloud. No transformation needed (already in `body` frame).
2. Check if S10 data is fresh using `steady_clock` (see "Why steady_clock" below).
3. If fresh, transform each S10 point from `mrdvs_tof` to `body` using the cached TF matrix, convert RGB to intensity, and append to the output cloud.
4. Publish with the M300 message header (timestamp + frame_id = `body`).

### Why M300 Drives the Merge

The M300 callback triggers the merge, not a timer or S10 callback. This ensures:
- Output rate matches M300 exactly (10 Hz).
- Output timestamp stays in the M300 clock domain, consistent with the FAST-LIO2 TF tree.
- If S10 is offline, M300 still passes through. If M300 is offline, no output is produced (correct, since FAST-LIO2 odometry is also unavailable).

---

## The Clock Problem

### Two Independent Clocks

The M300 LiDAR has an internal clock that starts counting from zero when powered on. Think of it as a stopwatch. The Ubuntu system has its own clock based on Unix epoch. These two clocks have no relationship:

```
M300 stopwatch:    25,093 seconds (hours since power-on)
System clock: 1,771,930,958 seconds (since January 1, 1970)
```

The S10 Ultra camera originally had a third clock (its own internal clock at ~22,000s), but we changed it to use the system clock since its data does not enter FAST-LIO2.

### Why Not Unify All Clocks to System Clock

A natural question: "Why not change the M300 driver to use system clock (`now()`) so everything matches?"

The answer lies in how M300 data is structured internally. The M300 driver publishes three time-related values from the same hardware clock:

**1. Frame timestamp** (`data->timestamp`): when the frame scan started.

**2. Per-point offset_time** (`p_point_data[i].offset_time`): how many nanoseconds after the frame start each individual point was measured. M300 is a non-repetitive scanner — within a single 100ms frame, each of the ~20,000 points is measured at a slightly different moment.

**3. IMU timestamp** (`data->timestamp` from IMU callback): when each IMU sample was taken.

All three share the same hardware clock. FAST-LIO2 uses this to perform motion compensation:

```
Frame starts at:  data->timestamp = 25,093,000,000,000 ns
Point A measured: 25,093,000,000,000 + 1,000,000 = 25,093.001s
Point B measured: 25,093,000,000,000 + 2,000,000 = 25,093.002s
IMU at:           25,093,000,000,000 ns
IMU at:           25,093,005,000,000 ns

FAST-LIO2 calculates: Point B was measured at 40% between these two IMU samples.
It interpolates the exact robot pose at that moment to correct Point B's position.
```

If we replace `data->timestamp` with `now()`:

```
Frame arrives at: now() = 1,771,930,958,110,000,000 ns (8ms network delay)
Point A offset:   still 1,000,000 ns (from original data->timestamp basis)

Computed time: 1,771,930,958,110,000,000 + 1,000,000 = 1,771,930,958.111s
Actual measurement time was: 1,771,930,958.103s (before network delay)
→ 8ms error
```

The offset_time values are calculated by M300 hardware relative to `data->timestamp`. Changing the base without changing the offsets shifts all computed times by an unpredictable amount (network delay varies per frame).

For IMU, the same problem applies. Network delay is different for each packet:

```
Actual measurement order (M300 hardware clock):
  IMU  → Point → IMU
  gap:   2ms      3ms    (precise, hardware-guaranteed)

With now() (arrival time):
  IMU arrives  → Point arrives → IMU arrives
  gap:   7ms         2ms         (distorted by varying network delay)
```

FAST-LIO2 interprets these gaps as actual motion timing. Incorrect gaps produce incorrect motion compensation, leading to blurred point clouds and odometry drift.

### Why S10 Could Switch to System Clock

The S10 Ultra data does not enter FAST-LIO2. It has no per-point offset_time. It has no IMU synchronization requirement. The only question cloud_merger asks is "did this data arrive recently?" — which does not depend on the timestamp value at all.

### How cloud_merger Handles the Clock Mismatch

Instead of comparing message timestamps (which are in incompatible clock domains), cloud_merger uses `steady_clock` — a monotonic wall clock that is independent of both sensor clocks:

```cpp
// S10 callback: record when data arrived
s10_recv_time_ = std::chrono::steady_clock::now();

// M300 callback: check how long ago S10 data arrived
auto now = std::chrono::steady_clock::now();
double dt = (now - s10_recv_time_).count();  // seconds
if (dt < 0.5) { /* fresh enough, merge */ }
```

This approach works because both sensors run at 10 Hz. If the S10 data arrived less than 0.5 seconds ago (allowing up to 4 missed frames), it is considered fresh enough to merge. The actual timestamp values (25,093s vs 1,771,930,958s) are never compared.

```
Wall clock timeline:
  t=0.00s  M300 data arrives  (stamp: 25093.100, M300 clock)
  t=0.02s  S10 data arrives   (stamp: 1771930958.5, system clock)
  t=0.10s  M300 data arrives  → merge check
           steady_clock: S10 arrived 0.08s ago → below 0.5s → merge
```

---

## Coordinate Frame Handling

The two sensors output points in different conventions:

| Input | Frame | Convention |
|---|---|---|
| M300 (via FAST-LIO2) | `body` | ROS standard (X=fwd, Y=left, Z=up) |
| S10 Ultra | `mrdvs_tof` | Depends on `LX_INT_XYZ_COORDINATE` |

Setting `LX_INT_XYZ_COORDINATE=1` makes the SDK output in ROS convention directly. Without this (default=0), points come in camera optical convention (X=right, Y=down, Z=forward), requiring a 90-degree rotation in the TF. With it set to 1, the static TF only needs position offset, no rotation.

---

## Thread Safety

S10 and M300 callbacks run on separate threads. The shared S10 data is protected by a mutex. The M300 callback locks the mutex only during the merge section:

```
S10 thread:              M300 thread:
  lock(mutex)              copy M300 points (no lock needed)
  store S10 data           lock(mutex)
  record recv_time         read S10 data, transform, merge
  unlock(mutex)            unlock(mutex)
                           publish merged cloud
```

---

## Graceful Degradation

| S10 Status | Behavior |
|---|---|
| Online, fresh data | Full merge (M300 + S10) |
| Online, stale data (>0.5s) | M300 only, warning logged |
| Offline | M300 only, no warning |
| M300 offline | No output (FAST-LIO2 also unavailable) |

The 0.5s threshold allows up to 4 missed S10 frames (at 10 Hz) before considering the data stale.