import serial, time, sys
sys.stdout.reconfigure(encoding='utf-8')

ser = serial.Serial('COM13', 115200, timeout=1)
time.sleep(3.5)
ser.reset_input_buffer()

def send(c):
    ser.write((c + chr(10)).encode('utf-8'))
    ser.flush()

p0 = 101325.0

print('=' * 75)
print(' RUNNING STEP-BY-STEP ELEVATOR CLIMB WITH FIRMWARE DIAGNOSTICS')
print('=' * 75)

send('CMD,001,SIM,ENABLE')
send('CMD,001,CAL')
time.sleep(0.5)

# Step climb
for alt in [0.0, 5.0, 15.0, 30.0]:
    p = p0 * ((1.0 - 2.25577e-5 * alt) ** 5.25588)
    print(f'\n--> INJECTING ALT: {alt}m')
    send(f'CMD,001,SIMP,{p:.1f}')
    t_end = time.time() + 1.2
    while time.time() < t_end:
        while ser.in_waiting:
            l = ser.readline().decode('utf-8', errors='replace').strip()
            if l:
                print('   [MCU] ' + l)

print('\n--> DROPPING FROM BALCONY (30m -> 0m)')
for alt in [25.0, 18.0, 10.0, 1.0, 0.0]:
    p = p0 * ((1.0 - 2.25577e-5 * alt) ** 5.25588)
    print(f'\n--> INJECTING ALT: {alt}m')
    send(f'CMD,001,SIMP,{p:.1f}')
    t_end = time.time() + 1.2
    while time.time() < t_end:
        while ser.in_waiting:
            l = ser.readline().decode('utf-8', errors='replace').strip()
            if l:
                print('   [MCU] ' + l)

send('CMD,001,SIM,DISABLE')
ser.close()
