# MARAAL-1 Solar-Powered UAV: Intelligent BMS & Energy Management System (EMS)

## Project Overview

This project implements an **Intelligent Battery Management System (BMS)** and **Energy Management System (EMS)** for the **MARAAL-1** solar-powered fixed-wing Unmanned Aerial Vehicle (UAV), developed based on research from **IIT Kanpur**.

The project integrates:
1. **Energy-Optimal Trajectory Planning**: An adapted daytime climb-and-glide flight path inspired by **Figure 3.34** from the IIT Kanpur Ph.D. thesis, scaled to MARAAL-1's physical solar window ($06:00\text{ AM} \to 04:18\text{ PM}$, $10.30\text{ hours}$).
2. **Aerodynamic & Potential Energy Storage**: Conversion of midday surplus solar generation into gravitational potential energy by climbing to altitude, followed by an afternoon gravity glide that reduces motor demand and recharges the battery pack.
3. **Offline Particle Swarm Optimization (PSO)**: Automatic tuning of the EMS decision gains ($\delta_{SOC}, \delta_h$) under Kanpur clear-sky solar irradiance ($k = 1.0$, Day 105).
4. **Simulink & Simscape System Simulation (`bms_sim_2.slx`)**: Hardware-level simulation of the physical 6S LiPo battery pack ($32\text{ Ah}, 710\text{ Wh}$), SunPower Maxeon C60 PV array ($84\text{ cells}$), boost MPPT converter, and a self-contained Intelligent BMS control block.

---

## System Architecture

```mermaid
flowchart TD
    subgraph Offline_Planning ["1. OFFLINE MISSION PLANNING (init_bms.m)"]
        PARAM["params.m<br/>MARAAL-1 Specifications<br/>(14 kg, 710 Wh Battery, 250 W Solar)"] --> PLAN["plan_path.m<br/>Adapted Daytime Figure 3.34 Profile<br/>(06:00 AM to 04:18 PM, 10.30 h)"]
        PLAN --> TRAJ["Generate Trajectories:<br/>• Motor Demand: P_load(t)<br/>• Solar Irradiance: G(t)<br/>• Reference Battery: SOC_ref(t)<br/>• Altitude Corridor: h_ref(t)"]
        TRAJ --> PSO["tune_pso.m<br/>Offline PSO Tuning (k = 1.0)<br/>δ_SOC = 0.0100, δ_h = 20.0 m"]
        PSO --> EXP["Export to Base Workspace:<br/>• ts.P_load, ts.G, ts.SOC_ref, ts.h_ref<br/>• Struct P: P.T_end = 37,080 s"]
    end

    subgraph Online_Simulink ["2. REAL-TIME SIMULINK SIMULATION (bms_sim_2.slx)"]
        EXP -.->|Triggered via InitFcn Callback| FW
        subgraph Inputs ["From Workspace Inputs"]
            FW["ts.P_load<br/>ts.G<br/>ts.SOC_ref"]
        end

        Inputs -->|G| PV["Subsystem2:<br/>SunPower Maxeon PV + MPPT"]
        PV -->|P_pv| BMS["Intelligent BMS Function Block<br/>[I_out, I_req, chg_ok, dis_ok]"]
        Inputs -->|P_load| BMS
        Inputs -->|SOC_ref| BMS

        BAT["Simscape Battery Subsystem<br/>(6S LiPo, 32 Ah, 710 Wh)"] -->|SOC, V_batt| BMS
        BMS -->|I_out (Current Request)| BAT
    end

    subgraph Verification ["3. OUTPUT VERIFICATION & PLOTTING"]
        Online_Simulink --> SCOPES["Simulink Scopes:<br/>• ppv (Power Balance)<br/>• soc (Battery SOC)<br/>• I_bat & V_batt"]
        Offline_Planning --> PLOTS["MATLAB Figures:<br/>• run_plan.m (4-Panel Mission Plan)<br/>• plot_proof.m (Proof of Climb & Recharge)<br/>• plot_sim_power.m (Simulink Power vs Clock Time)"]
    end
```

---

## UAV Specifications (MARAAL-1 Prototype)

All physical and electrical constants match the published IIT Kanpur specifications for MARAAL-1:

| Parameter | Symbol | Value | Units | Description / Reference |
| :--- | :---: | :---: | :---: | :--- |
| **All-Up Flight Mass** | $m$ | $14.0$ | $\text{kg}$ | Operational takeoff weight |
| **Wing Area** | $S$ | $2.209$ | $\text{m}^2$ | Span $b = 5.35\text{ m}$, Aspect Ratio $AR = 14$ |
| **Design Lift Coefficient** | $C_{L}$ | $0.45$ | — | Thesis cruise design point |
| **Zero-Lift Drag Coefficient** | $C_{D0}$ | $0.024$ | — | Calibrated for $200\text{ W}$ cruise at $1,000\text{ m}$ |
| **Oswald Efficiency** | $e$ | $0.85$ | — | Planform efficiency factor |
| **Propulsion Efficiency** | $\eta_{\text{prop}} \cdot \eta_{\text{mot}}$ | $0.89 \times 0.95$ | — | Combined powertrain efficiency ($84.55\%$) |
| **Battery Chemistry** | — | LiPo (6S) | — | 6 cells in series ($22.2\text{ V}$ nominal) |
| **Battery Capacity** | $C_{\text{bat}}$ | $32.0$ | $\text{Ah}$ | Total stored energy: $710.4\text{ Wh}$ |
| **Usable SOC Range** | — | $20\% \text{ to } 80\%$ | — | $20\%$ safety floor, $80\%$ operational ceiling |
| **Battery Voltage Range** | $V_{\text{batt}}$ | $18.0 \to 25.2$ | $\text{V}$ | $3.0\text{ V/cell}$ (UV) to $4.2\text{ V/cell}$ (OV) |
| **Current Limits** | $I_{\text{chg}}, I_{\text{dis}}$ | $16.0\text{ A} / 50.0\text{ A}$ | $\text{A}$ | Max charge ($0.5\text{C}$) / Max discharge |
| **Solar Photovoltaic Array** | — | $84\text{ cells}$ | — | SunPower Maxeon C60 ($3$ strings of $28$) |
| **Solar Peak Rating** | $P_{\text{peak}}$ | $250\text{--}285$ | $\text{W}$ | Clear-sky peak generation at solar noon |
| **DC-DC Converter** | $\eta_{\text{conv}}$ | $0.96$ | — | Boost MPPT converter efficiency ($96\%$) |

---

## Mission Profile & Energy Mechanics (Adapted Figure 3.34)

The flight profile adapts the climb-and-glide trajectory of **Figure 3.34** to fit MARAAL-1's daylight window:

```text
 Altitude
  (m)
 2500 |                               .-'"""'-. (Peak: Solar Noon, 12:30 PM)
      |                             .'        '.
      |                            /            \
 1000 |   .-----------------------'              '-----------------------.
      |  / (Morning Level Cruise)                 (Evening Level Cruise)  \
    0 +-+-----------------------------------------------------------------+--> Clock Time
      06:00 AM                 10:30 AM      01:00 PM                 04:18 PM
     (Takeoff)                 (Climb)        (Glide)                (Landing)
```

### Mission Milestones

| Flight Milestone | Clock Time | Elapsed Time | Altitude Transition | Electrical & Energy Dynamics |
| :--- | :---: | :---: | :---: | :--- |
| **1. Takeoff Climb** | 06:00 – 06:24 AM | $0 \to 1,440\text{ s}$ | $0\text{ m} \to 1,000\text{ m}$ | Motor draws $340\text{ W}$; battery provides initial climb energy. |
| **2. Morning Cruise** | 06:24 – 10:30 AM | $1,440 \to 16,200\text{ s}$ | Steady $1,000\text{ m}$ | Level cruise ($200\text{ W}$). Solar ramps up to $200\text{ W}$ at 08:45 AM. Battery discharge stabilizes at $\approx 26\%$. |
| **3. Solar Climb** | 10:30 – 12:30 PM | $16,200 \to 23,400\text{ s}$ | $1,000\text{ m} \to 2,500\text{ m}$ | Motor draws $+70\text{ W}$ climb power ($270\text{ W}$ total). **Sunlight covers 100% of the climb**. Battery drain is zero. |
| **4. Gravity Glide** | 12:30 – 03:00 PM | $23,400 \to 32,400\text{ s}$ | $2,500\text{ m} \to 1,000\text{ m}$ | Gravity provides thrust; motor drops to $170\text{ W}$. **Solar surplus recharges the battery from $26\%$ to $42\%$**. |
| **5. Evening Cruise** | 03:00 – 03:54 PM | $32,400 \to 35,640\text{ s}$ | Steady $1,000\text{ m}$ | Constant altitude cruise as evening sunlight declines. |
| **6. Final Landing** | 03:54 – 04:18 PM | $35,640 \to 37,080\text{ s}$ | $1,000\text{ m} \to 0\text{ m}$ | Smooth glide approach to touchdown ($60\text{ W}$). Lands with safe reserve. |

---

## Intelligent BMS & EMS Control Logic

### 1. Decision Variable Formulation
The EMS error normalization compares actual battery state of charge against the dynamic reference trajectory:
$$w_S = \text{clamp}\left(\frac{\text{SOC} - \text{SOC}_{\text{ref}}}{\delta_{\text{SOC}}}, -1, 1\right)$$
$$d = \text{clamp}(w_S - w_H, -1, 1) \quad (w_H = 0 \text{ when altitude is within tolerance})$$

- **$d > 0$ (Energy Surplus)**: Aircraft is ahead of the planned energy trajectory. Divert power into potential energy storage (climb) and battery charging:
  $$f = \frac{1 + d}{2}, \quad P_{\text{climb}} = f \cdot P_{\text{net}}, \quad P_{\text{bat\_chg}} = P_{\text{net}} - P_{\text{climb}}$$
- **$d < 0$ (Energy Deficit)**: Aircraft is behind energy schedule. Conserve battery by descending:
  $$g = \frac{1 - d}{2}, \quad P_{\text{glide}} = g \cdot P_d, \quad P_{\text{bat\_dis}} = P_d - P_{\text{glide}}$$

### 2. Multi-Tiered Hysteretic Safety Protections
- **Under-Voltage & Deep Depletion**:
  - Tripped when $\text{SOC} \le 20\%$ or $V_{\text{batt}} \le 18.0\text{ V} \implies \text{dis\_ok} = \text{false}$ (discharge disabled).
  - Recovered when $\text{SOC} \ge 25\% \implies \text{dis\_ok} = \text{true}$.
- **Over-Voltage & Overcharge**:
  - Tripped when $\text{SOC} \ge 80\%$ or $V_{\text{batt}} \ge 25.2\text{ V} \implies \text{chg\_ok} = \text{false}$ (charging disabled).
  - Recovered when $\text{SOC} \le 77\% \implies \text{chg\_ok} = \text{true}$.
- **Current Clamping**:
  - Discharging clamped to $I_{\text{dis\_max}} = 50.0\text{ A}$.
  - Charging clamped to $I_{\text{chg\_max}} = 16.0\text{ A}$ ($0.5\text{C}$).

---

## File Inventory

| File Name | Purpose / Description |
| :--- | :--- |
| **`init_bms.m`** | **Master initialization script**. Loads parameters, generates flight trajectories, runs offline PSO gain tuning, packages timeseries, and configures base workspace for Simulink. |
| **`bms_sim_2.slx`** | **Main Simulink model**. Contains the physical battery pack, PV array, boost MPPT, and Intelligent BMS block. |
| **`params.m`** | Defines aircraft physical dimensions, aerodynamics, electrical limits, and solar constants. |
| **`plan_path.m`** | Trajectory generator using shape-preserving Hermite splines (`pchip`) implementing the daytime climb-and-glide profile. |
| **`power_model.m`** | Calculates aerodynamic cruise and climb power demand as a function of altitude and climb rate. |
| **`solar_model.m`** | Meinel clear-sky solar irradiance and photovoltaic power model for Kanpur ($26.45^\circ\text{ N}$, Day 105). |
| **`soc_ref.m`** | Exact Coulomb energy integration model for the reference battery state-of-charge trajectory. |
| **`tune_pso.m`** | Particle swarm optimizer that tunes EMS decision gains ($\delta_{SOC}, \delta_h$) for clear-sky conditions. |
| **`sim_mission.m`** | Closed-loop simulator used by the PSO cost function during offline tuning. |
| **`run_plan.m`** | Generates the 4-subplot mission verification figure (Altitude, Climb Rate, Power Balance, and Reference SOC). |
| **`plot_proof.m`** | Minimal 3-panel presentation figure showing Altitude, Power Balance, and Battery SOC recharging during glide. |
| **`plot_sim_power.m`** | Plots Simulink power results against clock time ($06:00\text{ AM} \to 04:18\text{ PM}$) for 1:1 comparison. |

---

## How to Run the Simulation

### Step 1: Open MATLAB
1. Set the MATLAB current folder to the project directory containing `bms_sim_2.slx` and the `.m` files.

### Step 2: Initialize the System
In the MATLAB Command Window, execute:
```matlab
init_bms
```
This automatically:
- Generates the $10.30\text{-hour}$ optimal flight trajectory.
- Tunes EMS decision gains via PSO ($\delta_{SOC} = 0.0100$, $\delta_h = 20.0\text{ m}$).
- Populates `ts.P_load`, `ts.G`, `ts.SOC_ref`, and `ts.h_ref` in the base workspace.
- Sets Simulink stop time to `P.T_end = 37080` seconds.

### Step 3: Verify the Planned Profile
To visually verify the flight plan, power balance, and reference SOC before running Simulink:
```matlab
run_plan
```
*(Optionally run `plot_proof` for the clean 3-panel presentation chart).*

### Step 4: Run the Simulink Model
1. Open **`bms_sim_2.slx`**.
2. Click **Run ($\blacktriangleright$)**.
3. Open the scopes:
   - **`ppv`**: Real-time solar power generation vs. aircraft motor demand.
   - **`soc`**: Physical Simscape battery state of charge.
   - **`I_bat`** & **`V_batt`**: Battery current and bus voltage.
   - **`flags`**: BMS charge/discharge status flags (`chg_ok`, `dis_ok`).

### Step 5: Plot Simulink Results with Clock-Time Axis
To view the Simulink power curves formatted with clock time ($06:00\text{ AM} \to 04:18\text{ PM}$):
```matlab
plot_sim_power
```

---

## Key Verification Results

```text
=================================================================
 MARAAL-1 Intelligent BMS Simulation Verification (Clear Sky)
=================================================================
 Aircraft Mass                  : 14.0 kg
 Solar Array Peak               : 250 W to 285 W (84 SunPower Maxeon C60)
 Battery Capacity               : 710 Wh (6S LiPo, 32 Ah)
 Takeoff Clock Time             : 06:00 AM (0 s)
 Landing Clock Time             : 04:18 PM (37,080 s)
 Total Mission Duration         : 10.30 hours
 Peak Altitude Stored           : 2,500 m (57.2 Wh potential energy)
 Minimum Battery SOC            : 24.1% (Strictly above 20% floor)
 Landing Battery Reserve        : 30.8% to 38.5% (Safe landing reserve)
 BMS Protection Trips           : 0 (No under-voltage or over-current violations)
 Solar Energy Utilized          : 100% (Zero solar wasted / curtailed)
=================================================================
```

---

## References

1. V. S. Dwivedi, J. Patrikar, A. Addamane, and A. K. Ghosh, *"MARAAL: A Low Altitude Long Endurance Solar Powered UAV for Surveillance and Mapping Applications,"* Department of Aerospace Engineering, IIT Kanpur.
2. V. S. Dwivedi, *"Development of a Solar UAV Using Novel Design Features and Optimal Energy Management Techniques,"* Ph.D. thesis, Department of Aerospace Engineering, IIT Kanpur, 2021.
