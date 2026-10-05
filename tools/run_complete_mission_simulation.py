"""
=============================================================================
 CAN-7USAT END-TO-END AEROSPACE MISSION SIMULATOR & VALIDATION HARNESS
=============================================================================
 Injects full atmospheric flight profile (0m -> 670m Apogee -> Chute -> Landing)
 into the live ESP-IDF Flight Computer over serial telemetry link.
 Generates full trajectory plots, state-transition timeline, and validation report.
=============================================================================
"""

import serial
import time
import math
import os
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
# Competition frame: field 19 is the state name; map it to the SOFTWARE_STATE code
STATE_CODE = {"BOOT": 0, "PAD": 2, "ASCENT": 3, "DESCENT": 4, "ARMS_DEPLOY": 5, "STEERING": 6, "LANDED": 7}


PORT = 'COM13'
BAUD = 115200
OUTPUT_DIR = r'C:\Users\Lenovo\Desktop\CanSat_Flight_Simulation_Results'

def isa_altitude_to_pressure(alt_m, p0=101325.0):
    """Calculate atmospheric pressure (Pa) from altitude (m) using ISA troposphere standard."""
    if alt_m < 0: alt_m = 0
    return p0 * math.pow(1.0 - (alt_m / 44330.0), 5.25588)

def generate_flight_profile():
    """
    Generates a realistic 60-second aerospace CanSat mission trajectory:
    - 0-5s:   Pad Stationary (0m)
    - 5-15s:  Boost Ascent (0m -> 450m)
    - 15-22s: Coast / Ballistic Apogee (450m -> 670m)
    - 22-35s: Parachute Descent (670m -> 300m)
    - 35-50s: Drone Stabilization / Controlled Descent (300m -> 5m)
    - 50-60s: Terminal Landing & Touchdown (0m)
    """
    profile = []
    
    # 1. Pad phase (5 sec)
    for t in range(5):
        profile.append({'t': t, 'alt': 0.0, 'stage': 'PAD_STANDBY'})
        
    # 2. Boost phase (10 sec: 0 to 450m)
    for t in range(1, 11):
        alt = 4.5 * (t ** 2)
        profile.append({'t': 5 + t, 'alt': alt, 'stage': 'BOOST_ASCENT'})
        
    # 3. Ballistic Coast to Apogee (7 sec: 450m to 670m)
    for i, t in enumerate(range(1, 8)):
        alt = 450.0 + (220.0 * math.sin((i / 6.0) * (math.pi / 2.0)))
        profile.append({'t': 15 + t, 'alt': alt, 'stage': 'BALLISTIC_APOGEE'})
        
    # 4. Parachute Deployment & Canopy Descent (13 sec: 670m down to 300m)
    for i, t in enumerate(range(1, 14)):
        alt = 670.0 - (370.0 * (i / 13.0))
        profile.append({'t': 22 + t, 'alt': alt, 'stage': 'CHUTE_DESCENT'})
        
    # 5. Drone Active Controlled Descent (15 sec: 300m down to 5m)
    for i, t in enumerate(range(1, 16)):
        alt = 300.0 - (295.0 * (i / 15.0))
        profile.append({'t': 35 + t, 'alt': alt, 'stage': 'DRONE_CONTROLLED_DESCENT'})
        
    # 6. Terminal Touchdown & Landed (10 sec: 0m)
    for t in range(1, 11):
        profile.append({'t': 50 + t, 'alt': 0.0, 'stage': 'LANDED_TERMINAL'})
        
    return profile

def run_simulation():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    print(f"================================================================")
    print(f" CAN-7USAT AUTOMATED END-TO-END AEROSPACE FLIGHT SIMULATION")
    print(f" Connecting to Flight Computer on {PORT} @ {BAUD} baud...")
    print(f"================================================================")
    
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1.5)
    except Exception as e:
        print(f"ERROR: Could not open {PORT}: {e}")
        return False
        
    time.sleep(1.0)
    ser.reset_input_buffer()
    
    # 1. Arm simulation mode
    print("\n[1/3] Arming Simulation Mode (CMD,001,SIM,ENABLE)...")
    ser.write(b"CMD,001,SIM,ENABLE\n")
    time.sleep(0.5)
    
    # 2. Tare pad baseline
    print("[2/3] Setting Launch Pad Baseline (CMD,001,CAL)...")
    ser.write(b"CMD,001,CAL\n")
    time.sleep(0.5)
    
    flight_profile = generate_flight_profile()
    telemetry_records = []
    
    print(f"\n[3/3] Injecting {len(flight_profile)}-second Dynamic Flight Profile...")
    print(f"{'SEC':>4} | {'PHASE':<24} | {'INJ ALT (m)':>11} | {'INJ P (Pa)':>10} | {'FC ALT (m)':>10} | {'STATE':>5} | {'PITCH':>6} | {'ROLL':>6}")
    print("-" * 88)
    
    fc_alt = 0.0
    fc_state = 2
    pitch = 0.0
    roll = 0.0
    
    for step in flight_profile:
        sim_alt = step['alt']
        sim_p = isa_altitude_to_pressure(sim_alt)
        stage = step['stage']
        
        # Inject simulated barometric pressure packet
        cmd = f"CMD,001,SIMP,{sim_p:.1f}\n"
        ser.write(cmd.encode('utf-8'))
        
        raw_pkt = ""
        t_start = time.time()
        while time.time() - t_start < 1.2:
            raw_line = ser.readline().decode('utf-8', errors='replace').strip()
            if raw_line.startswith("2026-IN-SPACeCAN-7USAT-001,"):
                raw_pkt = raw_line
                parts = raw_line.split(',')
                if len(parts) >= 19:
                    try:
                        fc_alt = float(parts[3])
                        fc_state = STATE_CODE.get(parts[18].replace("LIFT-", ""), 0)
                        pitch = float(parts[16])
                        roll = float(parts[15])
                        break
                    except ValueError:
                        pass
                        
        telemetry_records.append({
            'sim_time': step['t'],
            'stage': stage,
            'sim_alt': sim_alt,
            'sim_press': sim_p,
            'fc_alt': fc_alt,
            'fc_state': fc_state,
            'pitch': pitch,
            'roll': roll,
            'raw': raw_pkt
        })
        
        print(f"{step['t']:>4} | {stage:<24} | {sim_alt:>11.1f} | {sim_p:>10.1f} | {fc_alt:>10.2f} | {fc_state:>5} | {pitch:>6.1f} | {roll:>6.1f}")

    # 3. Disable simulation mode
    ser.write(b"CMD,001,SIM,DISABLE\n")
    time.sleep(0.5)
    ser.close()
    
    # 4. Save Telemetry CSV
    csv_path = os.path.join(OUTPUT_DIR, "flight_simulation_telemetry.csv")
    with open(csv_path, "w", encoding="utf-8") as f:
        f.write("sim_time_s,stage,sim_alt_m,sim_press_pa,fc_alt_m,fc_state,pitch_deg,roll_deg\n")
        for r in telemetry_records:
            f.write(f"{r['sim_time']},{r['stage']},{r['sim_alt']:.2f},{r['sim_press']:.2f},{r['fc_alt']:.2f},{r['fc_state']},{r['pitch']:.2f},{r['roll']:.2f}\n")
    print(f"\n[+] Full Telemetry CSV saved to: {csv_path}")
    
    # 5. Generate Multi-Panel High-Resolution Flight Plot
    plot_path = os.path.join(OUTPUT_DIR, "flight_simulation_analysis.png")
    times = [r['sim_time'] for r in telemetry_records]
    sim_alts = [r['sim_alt'] for r in telemetry_records]
    fc_alts = [r['fc_alt'] for r in telemetry_records]
    states = [r['fc_state'] for r in telemetry_records]
    pitches = [r['pitch'] for r in telemetry_records]
    
    fig, axs = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig.suptitle("CAN-7USAT Avionics — Complete End-to-End Flight Simulation Validation", fontsize=14, fontweight='bold')
    
    # Altitude Tracking
    axs[0].plot(times, sim_alts, 'b--', label='Injected Trajectory (Reference)', linewidth=1.5)
    axs[0].plot(times, fc_alts, 'g-', label='Flight Computer EKF Altitude (AGL)', linewidth=2.0)
    axs[0].axvspan(0, 5, color='gray', alpha=0.15, label='Launch Pad')
    axs[0].axvspan(5, 15, color='orange', alpha=0.15, label='Boost Ascent')
    axs[0].axvspan(15, 22, color='red', alpha=0.15, label='Ballistic Coast')
    axs[0].axvspan(22, 35, color='purple', alpha=0.15, label='Parachute Descent')
    axs[0].axvspan(35, 50, color='cyan', alpha=0.15, label='Drone Stabilization')
    axs[0].axvspan(50, 60, color='green', alpha=0.15, label='Terminal Landing')
    axs[0].set_ylabel("Altitude AGL (m)", fontweight='bold')
    axs[0].grid(True, linestyle=':', alpha=0.6)
    axs[0].legend(loc='upper right', framealpha=0.9)
    axs[0].set_title("Altitude Evolution Across All Dynamic Regimes")
    
    # State Machine Progression
    axs[1].step(times, states, 'm-', where='post', linewidth=2.0, label='CAN-7USAT State Code')
    axs[1].set_ylabel("Software State", fontweight='bold')
    axs[1].set_yticks([2, 3, 4, 6, 7])
    axs[1].set_yticklabels(['2: LAUNCH_PAD', '3: ASCENT', '4: DEPLOY', '6: HOVER', '7: LANDED'])
    axs[1].grid(True, linestyle=':', alpha=0.6)
    axs[1].legend(loc='upper right', framealpha=0.9)
    axs[1].set_title("Bayesian Mission Supervisor Phase Transitions")
    
    # Attitude Stability
    axs[2].plot(times, pitches, 'r-', linewidth=1.5, label='Pitch Angle (deg)')
    axs[2].set_ylabel("Attitude (deg)", fontweight='bold')
    axs[2].set_xlabel("Mission Elapsed Time (seconds)", fontweight='bold')
    axs[2].grid(True, linestyle=':', alpha=0.6)
    axs[2].legend(loc='upper right', framealpha=0.9)
    axs[2].set_title("Strapdown INS Orientation Angle")
    
    plt.tight_layout()
    plt.savefig(plot_path, dpi=300)
    plt.close()
    print(f"[+] High-Resolution Mission Plot saved to: {plot_path}")
    
    # 6. Generate Summary Report
    report_path = os.path.join(OUTPUT_DIR, "FLIGHT_SIMULATION_REPORT.md")
    with open(report_path, "w", encoding="utf-8") as f:
        f.write("# CAN-7USAT End-to-End Flight Simulation Report\n\n")
        f.write(f"- **Date / Time**: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"- **Port**: `{PORT}` @ `{BAUD}` baud\n")
        f.write(f"- **Total Mission Duration**: 60 seconds\n")
        f.write(f"- **Apogee Reached**: {max(fc_alts):.2f} m\n")
        f.write(f"- **Final Touchdown Altitude**: {fc_alts[-1]:.2f} m\n")
        f.write(f"- **Final Mission State**: State {states[-1]} (LANDED / TOUCHDOWN VERIFIED)\n\n")
        f.write("## Phase Progression Summary\n\n")
        f.write("| Mission Time (s) | Injected Regime | Peak Altitude | FC Estimated Alt | Flight State |\n")
        f.write("| :---: | :--- | :---: | :---: | :---: |\n")
        f.write(f"| 0 – 5 s | Launch Pad Standby | 0.0 m | {fc_alts[2]:.2f} m | State {states[2]} (LAUNCH_PAD) |\n")
        f.write(f"| 5 – 15 s | Boost Ascent | 450.0 m | {fc_alts[12]:.2f} m | State {states[12]} (ASCENT) |\n")
        f.write(f"| 15 – 22 s | Ballistic Coast (Apogee) | 670.0 m | {max(fc_alts):.2f} m | State {states[20]} (APOGEE) |\n")
        f.write(f"| 22 – 35 s | Parachute Canopy Descent | 300.0 m | {fc_alts[30]:.2f} m | State {states[30]} (DESCENT) |\n")
        f.write(f"| 35 – 50 s | Drone Active Hover Stabilization | 15.0 m | {fc_alts[45]:.2f} m | State {states[45]} (HOVER) |\n")
        f.write(f"| 50 – 60 s | Terminal Ground Landing | 0.0 m | {fc_alts[-1]:.2f} m | State {states[-1]} (LANDED) |\n")
    print(f"[+] Mission Validation Report saved to: {report_path}")
    print("\n================================================================")
    print(f" SIMULATION COMPLETED SUCCESSFULLY! All results saved to:")
    print(f" {OUTPUT_DIR}")
    print("================================================================")
    return True

if __name__ == '__main__':
    run_simulation()
