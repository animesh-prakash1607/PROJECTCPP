# PROJECTCPP
# Autonomous Landslide & Flash-Flood Early Warning Grid (ALF-EWG)

**Course:** Object-Oriented Programming Using C++ (CIC-211)
**Institute:** Maharaja Agrasen Institute of Technology, Department of Computer Science and Engineering
**Project:** Option 1, Disaster Management Software System
**Language:** Standard C++ (compiles under C++11, C++14 and C++17)

## About the Project

Landslides and flash floods in mountainous regions such as the Himalayan belt give very little warning. Authorities deploy solar-powered IoT sensor nodes (soil moisture detectors, seismic geophones, acoustic tiltmeters and river-level sonar gauges) that report to a central processing node.

ALF-EWG is the backend of that central node. It reads sensor data, converts it into a hazard index, classifies the risk level, sounds regional sirens, ranks stations for evacuation, and saves emergency incidents permanently to a binary log file with commit and rollback support.

## Repository Contents

| File | Description |
|---|---|
| `project.cpp` | Complete source code of the project (single file) |
| `project.exe` | Pre-built Windows executable |
| `README.md` | Project documentation |

## How to Build and Run

Any standard C++ compiler works.

```
g++ -std=c++14 -Wall -Wextra -o project project.cpp
./project
```

On Windows, run `project.exe` from Command Prompt or PowerShell after building (or use the included executable).

The program creates two temporary files in the current folder (`stations_snapshot.txt` and `incident_log.bin`) and deletes them before it exits.

## How the System Works

1. **Ingest:** station measurements (rainfall rate, slope angle, soil saturation, vibration frequency) are read from a text stream with the overloaded `>>` operator. Sensor nodes are polled in repeated sweep cycles through base-class pointers.
2. **Evaluate:** the overloaded `computeHazardIndex()` functions produce a hazard value from 0 to 100, which is classified as NORMAL, ADVISORY, WATCH or WARNING.
3. **Aggregate:** stations are combined into regional zones with the overloaded `+` operator.
4. **Alert:** the hottest station of each zone drives its siren, a static global alert level is updated, and stations are sorted by a functor into an evacuation dispatch table.
5. **Persist:** the most serious incidents are written to a binary file inside a transaction that can be committed or rolled back.

### Hazard Index

| Overload | Formula |
|---|---|
| `computeHazardIndex(RainSlope)` | 100 x (0.6 x rain/120 + 0.4 x slope/60) |
| `computeHazardIndex(SoilVibration)` | 100 x (0.7 x soil saturation/100 + 0.3 x vibration/50) |
| `computeHazardIndex(river, rain, soil)` | 100 x (0.5 x river level/12 + 0.3 x rain/120 + 0.2 x soil/100) |

| Hazard value | Alert level |
|---|---|
| below 30 | NORMAL |
| 30 to below 55 | ADVISORY |
| 55 to below 75 | WATCH |
| 75 and above | WARNING |

## Syllabus Coverage

### Unit I: Core Foundations and Call Semantics
- Namespaces `DisasterCore::Sensors` and `DisasterCore::Alerts`
- Inline functions for real-time risk calculations
- Overloaded `computeHazardIndex()`
- Call by reference and return by reference for coordinate transformation and telemetry accumulation
- Friend class `TelemetryAuditor` reading private sensor metrics without getters

### Unit II: Encapsulation, Dynamic Memory and Operator Overloading
- `SensorStation`, `EarlyWarningSiren` and `TelemetryBuffer` with data abstraction and const correctness
- Static members tracking live stations, sounding sirens and the global alert level
- `TelemetryBuffer` with parameterized and dynamic constructors, deep copy constructor, copy assignment and destructor
- Operators `+` (zone aggregation), `>` (hazard priority), `<<` and `>>` (console and file streams)

### Unit III: Inheritance, Containership and Polymorphism
- Abstract base class `BaseSensorNode` with pure virtual `readTelemetry()` and `evaluateThreshold()`
- Diamond problem solved with virtual base classes: `DisasterTelemetryDevice`, `SolarPoweredNode`, `WirelessLoRaTransceiver`, `AutonomousHybridProbe`
- Containership: `PowerSubsystem` object inside `BaseSensorNode`
- `std::vector<BaseSensorNode*>` with runtime dynamic dispatch and virtual destructors

### Unit IV: Streams, Persistence, Exception Safety and STL
- Custom exceptions derived from `std::exception`: `SensorFailureException`, `TelemetryChecksumMismatch`, `ThresholdOverflowException` (plus `LogTransactionException`)
- Binary `std::fstream` incident log with transactional rollback using `seekp()` and `seekg()`
- STL: `std::vector`, `std::map`, `std::sort`, `std::for_each`
- Functors for priority-based evacuation routing and custom stream manipulators for tabular alert output

## Class Hierarchy

```
              DisasterTelemetryDevice
         ^                                ^
         | virtual                        | virtual
  SolarPoweredNode             WirelessLoRaTransceiver
         ^                                ^
         +----------------+---------------+
                          |
                AutonomousHybridProbe
```

`BaseSensorNode` (abstract) sits between the root and `SolarPoweredNode`. The concrete sensors `SoilMoistureProbe`, `SeismicGeophone`, `AcousticTiltmeter` and `RiverSonarGauge` derive from `SolarPoweredNode`.

## Sample Output (excerpt)

```
+------+--------------+----------+-----------+----------------------------------+
| Rank | Station      | Hazard   | Level     | Dispatch action                  |
+------+--------------+----------+-----------+----------------------------------+
| 1    | JSM-01       | 90.03    | WARNING   | EVACUATE via ridge road R-1      |
| 2    | KDR-01       | 79.85    | WARNING   | EVACUATE via ridge road R-2      |
| 3    | JSM-03       | 60.07    | WATCH     | PREPARE evacuation, route R-3    |
| 4    | KDR-02       | 39.23    | ADVISORY  | STANDBY rescue crews             |
| 5    | JSM-02       | 21.70    | NORMAL    | ROUTINE monitoring               |
+------+--------------+----------+-----------+----------------------------------+
```

The demonstration run also shows a `ThresholdOverflowException`, a `TelemetryChecksumMismatch` and a `SensorFailureException` being caught during sensor sweeps, and a binary log transaction that is rolled back after an invalid record.

## Notes

- Sensor readings come from a small deterministic generator, so the output is identical on every run.
- Two test hooks, `injectSpike()` and `corruptNextPacket()`, create the fault scenarios shown in the demonstration.
- The program was checked with AddressSanitizer and UndefinedBehaviorSanitizer and reported no memory errors or leaks.

## Author

- Name: ______________________
- Enrollment No.: ______________________
- Team member (optional): ______________________
