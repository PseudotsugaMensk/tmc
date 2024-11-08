import nidaqmx
from nidaqmx.constants import AcquisitionType, ThermocoupleType
import time
import threading
from simple_pid import PID
import keyboard  # To detect key presses

from datetime import datetime

# choose device name for simulation or real device
sumulation_device_name = "cDAQsim1"
real_device_name = "cDAQ1"
device_name = sumulation_device_name

simulate_temperature_rise = False

# Define the maximum allowable heating rate in degrees per second
max_heating_rate_h = 5      # ideally 5 - 20 K/h
max_heating_rate = 5 / 3600 # translate K/h in K/s
max_heating_rate = 100.0    # manually set
max_temp_difference = 10.0  # max difference between setpoint and actual temperature, ideally max 5 K
target_temperature = 60.0   # Desired temperature (setpoint) for all sensors

filename = "./log/" + dt_string + ".csv" 
start_time = time.time()
print("start time ", start_time)
running = True

# Parameters for PWM signal (initial values for each of the 6 channels)
pwm_channels = [
    device_name + "Mod3/port0/line0", device_name + "Mod3/port0/line1", device_name + "Mod3/port0/line2", 
    device_name + "Mod3/port0/line3", device_name + "Mod3/port0/line4", device_name + "Mod3/port0/line5"
]
frequency = 0.5  # Frequency in Hz
duty_cycles = [0.0] * len(pwm_channels)  # Initial duty cycles for 6 channels (0.0 to 1.0)
set_points = [target_temperature] * len(pwm_channels)

# Parameters for temperature reading
thermocouple_channels = [
    device_name + "Mod1/ai0", device_name + "Mod1/ai1", device_name + "Mod1/ai2", device_name + "Mod1/ai3",
    device_name + "Mod1/ai4", device_name + "Mod1/ai5"
]  # List of 6 thermocouple channels
sample_rate = 1  # Number of temperature samples per second

# Shared variables between threads (global)
temperatures = [[25.0]] * len(thermocouple_channels)  # Store the most recent temperature readings for 6 thermocouples
duty_cycle_locks = [threading.Lock() for _ in range(len(pwm_channels))]  # Lock for each channel to update duty cycle

# PID controller setup
pidParams = {
    "T1" : {"Kp" : 0.064823199, "Ki" : 0.001087731, "Kd" : 2.422442948}, 
    "T2" : {"Kp" : 0.062793234, "Ki" : 0.001055583, "Kd" : 2.346583167},
    "T3" : {"Kp" : 0.071449094, "Ki" : 0.00120115, "Kd" : 2.670052627},
    "B1" : {"Kp" : 0.139964202, "Ki" : 0.002342305, "Kd" : 5.230462219},
    "B2" : {"Kp" : 0.106006268, "Ki" : 0.001777297, "Kd" : 3.961454223},
    "B3" : {"Kp" : 0.104808663, "Ki" : 0.001758581, "Kd" : 3.91669975},
}

pidParams = {
    "T1" : {"Kp" : 0.02, "Ki" : 0.0, "Kd" : 5.0}, 
    "T2" : {"Kp" : 0.025, "Ki" : 0.0, "Kd" : 5.0},
    "T3" : {"Kp" : 0.025, "Ki" : 0.0, "Kd" : 5.0},
    "B1" : {"Kp" : 0.08, "Ki" : 0.0, "Kd" : 5.0},
    "B2" : {"Kp" : 0.06, "Ki" : 0.0, "Kd" : 5.0},
    "B3" : {"Kp" : 0.06, "Ki" : 0.0, "Kd" : 5.0},
}

# Kpt = 0.02
# Kit = 0.000
# Kdt = 5.0
# Kpb = 0.03
# Kib = 0.000
# Kdb = 5.0
# pidParams = {
#     "T1" : {"Kp" : Kpt, "Ki" : Kit, "Kd" : Kdt}, 
#     "T2" : {"Kp" : Kpt, "Ki" : Kit, "Kd" : Kdt}, 
#     "T3" : {"Kp" : Kpt, "Ki" : Kit, "Kd" : Kdt}, 
#     "B1" : {"Kp" : Kpb, "Ki" : Kib, "Kd" : Kdb}, 
#     "B2" : {"Kp" : Kpb, "Ki" : Kib, "Kd" : Kdb}, 
#     "B3" : {"Kp" : Kpb, "Ki" : Kib, "Kd" : Kdb}, 
# }


pids = [PID(pidParams["T1"]["Kp"], pidParams["T1"]["Ki"], pidParams["T1"]["Kd"], setpoint=target_temperature),
        PID(pidParams["T2"]["Kp"], pidParams["T2"]["Ki"], pidParams["T2"]["Kd"], setpoint=target_temperature),
        PID(pidParams["T3"]["Kp"], pidParams["T3"]["Ki"], pidParams["T3"]["Kd"], setpoint=target_temperature),
        PID(pidParams["B1"]["Kp"], pidParams["B1"]["Ki"], pidParams["B1"]["Kd"], setpoint=target_temperature),
        PID(pidParams["B2"]["Kp"], pidParams["B2"]["Ki"], pidParams["B2"]["Kd"], setpoint=target_temperature),
        PID(pidParams["B3"]["Kp"], pidParams["B3"]["Ki"], pidParams["B3"]["Kd"], setpoint=target_temperature)
        ]
for pid in pids:
    pid.output_limits = (0, 0.5)  # Constrain the duty cycle output to 0 (0%) and 1 (100%)

#### Functions ###

# adjust setpoint depending on max temperature difference setting
def adjust_setpoint():
    global target_temperature
    global temperatures
    global max_temp_difference

    min_temperature = min([temp[0] for temp in temperatures])
    new_setpoint = min(min_temperature + max_temp_difference, target_temperature)
    for pid in pids:
        pid.setpoint = new_setpoint  # Update all PIDs with the new target
    return new_setpoint


# Function for reading temperatures from 6 thermocouples
def read_temperatures():
    global temperatures
    global duty_cycles
    with nidaqmx.Task() as task:
        # Add all 6 thermocouple channels to the task
        for channel in thermocouple_channels:
            task.ai_channels.add_ai_thrmcpl_chan(channel, thermocouple_type=ThermocoupleType.J)
        task.timing.cfg_samp_clk_timing(sample_rate, sample_mode=AcquisitionType.CONTINUOUS)

        while running:
            temperatures_formatted = []
            # Read temperatures from all 6 channels
            if not simulate_temperature_rise:
                temps = task.read(number_of_samples_per_channel=1)
                for i in range(len(thermocouple_channels)):
                    temperatures[i] = temps[i]
                    #print(f"Temperature {i}: {temperatures[i][0]:.2f} °C")
                for list in temps:
                    temperatures_formatted.append(list[0])
            
            # simulate temperature rise
            if simulate_temperature_rise:
                temperatures_formatted = []
                for i in range(len(temperatures)):
                    temp = temperatures[i][0]
                    temp = temp + 0.01
                    temperatures[i][0] = temp
                    temperatures_formatted.append(temp)
                    #print(f"Temperature {i}: {temperatures[i][0]:.2f} °C")
            
            new_setpoint = adjust_setpoint()

            formatted_output = "Temp: " + " ".join(f"{num:.2f}" for num in temperatures_formatted) + " Duty Cycle: " + " ".join(f"{num:.2f}" for num in duty_cycles) + f" NewSetpoint: {new_setpoint:.2f} "
            print(formatted_output)

            time.sleep(1.0 / sample_rate)

# Function for generating PWM signals for 8 outputs with NI 9472
def generate_pwm():
    global duty_cycles
    with nidaqmx.Task() as task:
        # Add 8 digital output channels for PWM signals (one for each output line)
        for channel in pwm_channels:
            task.do_channels.add_do_chan(channel)

        # Initialize time tracking for each PWM channel
        last_update_times = [time.time()] * len(pwm_channels)
        pwm_states = [False] * len(pwm_channels)  # Track whether each output is on (True) or off (False)

        while running:
            current_time = time.time()

            # Loop over each PWM channel to update its state based on its duty cycle
            for i in range(len(pwm_channels)):
                with duty_cycle_locks[i]:
                    current_duty_cycle = duty_cycles[i]
                    period = 1.0 / frequency  # PWM period
                    on_time = period * current_duty_cycle  # Time the output should be ON
                    off_time = period - on_time  # Time the output should be OFF

                # Update the state of each channel based on its last update time
                elapsed_time = current_time - last_update_times[i]

                # Update the state of each channel based on the elapsed time
                if pwm_states[i]:  # If currently ON
                    if elapsed_time >= on_time:
                        pwm_states[i] = False  # Turn OFF if the ON time has elapsed
                        last_update_times[i] = current_time
                else:  # If currently OFF
                    if elapsed_time >= off_time:
                        pwm_states[i] = True  # Turn ON if the OFF time has elapsed
                        last_update_times[i] = current_time

            # Write the updated states (True/False) for all channels simultaneously
            #pwm_states
            task.write(pwm_states)
        
        # Turn all outputs OFF
        pwm_states = [False] * len(pwm_channels)
        task.write(pwm_states)

# Function to adjust the PWM duty cycles using PID control based on the temperatures
def control_pwm_duty_cycles():
    global duty_cycles
    previous_temperatures = [[0.0] for _ in range(len(pwm_channels))]
    previous_time = time.time()

    counter = 0
    print("start at ", time.time() - start_time)
    while running:
        elapsed_time = time.time() - start_time
        current_time = time.time()
        time_delta = current_time - previous_time

        if counter >= 10:
            with open(filename, 'a') as file:
                file.write(f"{elapsed_time:.0f}; ")

        for i in range(len(pwm_channels)):
            with duty_cycle_locks[i]:
                # Get the current temperature reading for each sensor
                current_temperature = temperatures[i][0]

                # Calculate the rate of temperature change
                temperature_rate_of_change = (current_temperature - previous_temperatures[i][0]) / time_delta

                # Apply PID control
                pid_output = pids[i](current_temperature)
                
                # Check if the heating rate exceeds the maximum allowable rate
                if temperature_rate_of_change > max_heating_rate:
                    # Reduce the duty cycle if the rate of change is too high
                    duty_cycles[i] = max(0, pid_output - 0.1 * (temperature_rate_of_change - max_heating_rate))
                    print(f"Heating rate exceeded for channel {i}, reducing duty cycle from {pid_output:.2f} to {duty_cycles[i]:.2f}")
                else:
                    # heating rate ok
                    duty_cycles[i] = pid_output
                # Update previous temperatures for the next rate calculation
                previous_temperatures[i][0] = current_temperature


                # test with fixed duty cycle
                #duty_cycles[i] = 0.1
            
            if counter >= 10:
                with open(filename, 'a') as file:
                    file.write(f"{current_temperature:.2f}; ")
                    file.write(f"{duty_cycles[i]:.2f}; ")
                    

            #print(f"Duty Cycle {i} (from PID): {duty_cycles[i]:.2f}")
        #formatted_output = "Duty Cycle: " + " ".join(f"{num:.2f}" for num in duty_cycles)
        #print(formatted_output)
        if counter >= 10:
            counter = 0
            with open(filename, 'a') as file:
                file.write("\n")
        
        # Update the previous timestamp and reset the counter
        previous_time = current_time
        counter += 1
        time.sleep(0.5)  # Update the duty cycles every 0.5 seconds

# Function to check for a keystroke (e.g., 'q' to quit) and exit
def check_for_exit():
    print("Press 'q' to exit the program.")

    def on_q_press(event):
        global running
        running = False
        print("\nExiting program...")
    
    # Set up a listener for 'q' key
    keyboard.on_press_key('q', on_q_press, suppress=True)  # suppress=True stops 'q' from appearing in the terminal

    while running:
        time.sleep(0.1)


# Create threads for temperature reading, PWM generation, and PID-based duty cycle control
temperature_thread = threading.Thread(target=read_temperatures)
pwm_thread = threading.Thread(target=generate_pwm)
control_thread = threading.Thread(target=control_pwm_duty_cycles)
exit_thread = threading.Thread(target=check_for_exit)  # Thread for exiting the program

# Start all threads
temperature_thread.start()
pwm_thread.start()
control_thread.start()
exit_thread.start()

# Join threads to wait for their completion (they run indefinitely in this case)
temperature_thread.join()
pwm_thread.join()
control_thread.join()
exit_thread.join()  # Exit the program after 'q' is pressed

