import nidaqmx
from nidaqmx.constants import AcquisitionType, ThermocoupleType
import time
import threading
from simple_pid import PID
import keyboard  # To detect key presses


running = True
device_name = "cDAQsim1"

pwm_channels = [
    device_name + "Mod3/port0/line0", device_name + "Mod3/port0/line1", device_name + "Mod3/port0/line2", 
    device_name + "Mod3/port0/line3", device_name + "Mod3/port0/line4", device_name + "Mod3/port0/line5"
]

# Parameters for temperature reading
thermocouple_channels = [
    device_name + "Mod1/ai0", device_name + "Mod1/ai1", device_name + "Mod1/ai2", device_name + "Mod1/ai3",
    device_name + "Mod1/ai4", device_name + "Mod1/ai5"
]  # List of 6 thermocouple channels
sample_rate = 1  # Number of temperature samples per second

# Parameters for PWM signal (initial values for each of the 8 channels)
frequency = 0.5  # Frequency in Hz
duty_cycles = [0.0] * len(pwm_channels)  # Initial duty cycles for 6 channels (0.0 to 1.0)
max_temperature_difference = 10.0 # max difference between setpoint and actual temperature

target_temperature = 30.0  # Desired temperature (setpoint) for all sensors
set_points = [target_temperature] * len(pwm_channels)

# Shared variables between threads (global)
temperatures = [[25.0]] * len(thermocouple_channels)  # Store the most recent temperature readings for 6 thermocouples
duty_cycle_locks = [threading.Lock() for _ in range(len(pwm_channels))]  # Lock for each channel to update duty cycle

# PID controller setup (one for each thermocouple-PWM pair)
pids = [PID(2.0, 0.0, 0.00, setpoint=target_temperature) for _ in range(len(pwm_channels))]
for pid in pids:
    pid.output_limits = (0, 1.0)  # Constrain the duty cycle output to 0 (0%) and 1 (100%)

# PID controller setup (one for each thermocouple-PWM pair)
prediction_pids = [PID(2.0, 0.0, 0.00, setpoint=target_temperature) for _ in range(len(pwm_channels))]
for pid in prediction_pids:
    pid.output_limits = (0, 1.0)  # Constrain the duty cycle output to 0 (0%) and 1 (100%) 

def adjust_setpoint():
    global temperatures
    global target_temperature
    global max_temperature_difference
    global set_points
    for i in len(set_points):
        set_points[i] = min(target_temperature, temperatures[i] + max_temperature_difference)
        pids[i].setpoint = set_points[i]
        print(f"Temperature {i}: {set_points[i]:.2f} °C")


# Function for reading temperatures from 8 thermocouples
def read_temperatures():
    global temperatures
    with nidaqmx.Task() as task:
        # Add all 6 thermocouple channels to the task
        for channel in thermocouple_channels:
            task.ai_channels.add_ai_thrmcpl_chan(channel, thermocouple_type=ThermocoupleType.J)
        task.timing.cfg_samp_clk_timing(sample_rate, sample_mode=AcquisitionType.CONTINUOUS)

        while running:
            # Read temperatures from all 8 channels
            temps = task.read(number_of_samples_per_channel=1)
            #for i in range(len(thermocouple_channels)):
                #temperatures[i] = temps[i]
                #print(f"Temperature {i}: {temperatures[i][0]:.2f} °C")
            time.sleep(1.0 / sample_rate)

            # test
            for i in range(len(temperatures)):
                temp = temperatures[i][0]
                temperatures[i][0] = temp + 0.1
                print(f"Temperature {i}: {temperatures[i][0]:.2f} °C")

# Function for generating PWM signals for 8 outputs with NI 9472
def generate_pwm():
    global duty_cycles
    with nidaqmx.Task() as task:
        # Add 8 digital output channels for PWM signals (one for each output line)
        for channel in pwm_channels:
            task.do_channels.add_do_chan(channel)

        # Initialize time tracking for each PWM channel
        last_update_time = time.time()
        pwm_states = [False] * len(pwm_channels)  # Track whether each output is on (True) or off (False)

        while running:
            current_time = time.time()
            elapsed_time = current_time - last_update_time

            # Loop over each PWM channel to update its state based on its duty cycle
            for i in range(len(pwm_channels)):
                with duty_cycle_locks[i]:
                    current_duty_cycle = duty_cycles[i]
                    period = 1.0 / frequency  # PWM period
                    on_time = period * current_duty_cycle  # Time the output should be ON
                    off_time = period - on_time  # Time the output should be OFF

                # Update the state of each channel based on the elapsed time
                if pwm_states[i]:  # If currently ON
                    if elapsed_time >= on_time:
                        pwm_states[i] = False  # Turn OFF if the ON time has elapsed
                        last_update_time = current_time
                else:  # If currently OFF
                    if elapsed_time >= off_time:
                        pwm_states[i] = True  # Turn ON if the OFF time has elapsed
                        last_update_time = current_time

            # Write the updated states (True/False) for all channels simultaneously
            pwm_states
            task.write(pwm_states)
        
        # Turn all outputs OFF
        pwm_states = [False] * len(pwm_channels)
        task.write(pwm_states)

# System parameters
K = 5.0  # System gain
T = 1.0  # System time constant
delay_steps = 10  # Number of time steps for the delay

# Initialize storage for delayed outputs
temperature_history = [25] * delay_steps

def model_temperature_without_delay(heater_power, _current_temperature):
    """Simulate temperature change without delay."""
    # Simple thermal model: next temperature depends on power and current temperature
    new_temperature = _current_temperature + K * heater_power / (T + 1)
    return new_temperature

def model_temperature_with_delay(heater_power):
    """Simulate temperature with delay."""
    # Append the temperature based on heater power, simulate delay by shifting
    predicted_temperature = model_temperature_without_delay(heater_power, temperature_history[-1])
    temperature_history.append(predicted_temperature)
    delayed_temperature = temperature_history.pop(0)
    return delayed_temperature

# Function to adjust the PWM duty cycles using PID control based on the temperatures
def control_pwm_duty_cycles():
    global duty_cycles
    while running:
        for i in range(len(pwm_channels)):
            with duty_cycle_locks[i]:
                # Get the current temperature reading for each sensor
                current_temperature = temperatures[i][0]

                # Smith Predictor: Calculate the temperature without the delay
                predicted_temperature = model_temperature_without_delay(prediction_pids[i](set_points[i] - current_temperature), current_temperature)

                # Calculate the control signal (heater power) based on predicted temperature
                duty_cycles[i] = pids[i](predicted_temperature)

                # Use the PID controller to calculate the new duty cycle based on temperature error
                #duty_cycles[i] = pids[i](current_temperature)

            print(f"Duty Cycle {i} (from PID): {duty_cycles[i]:.2f}")
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

