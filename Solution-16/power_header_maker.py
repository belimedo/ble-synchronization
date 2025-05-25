import numpy as np

# Parameters
frequency = 50          # Hz
sampling_rate = 500     # Hz
duration = 5            # seconds
samples = sampling_rate * duration

# Time vector
t = np.linspace(0, duration, samples, endpoint=False)

# Amplitudes
amplitude_current = 10
amplitude_voltage = 300
phase_shift_rad = np.pi / 6  # 30 degrees

# Generate signals
current_signal = (amplitude_current * np.sin(2 * np.pi * frequency * t)).astype(np.int16)
voltage_signal = (amplitude_voltage * np.sin(2 * np.pi * frequency * t + phase_shift_rad)).astype(np.int16)

# Cos(phi) table: from -(N-1) to +N
samples_per_period = sampling_rate // frequency  # 10 for 50 Hz @ 500 Hz
cos_phi_indices = np.arange(-(samples_per_period - 1), samples_per_period)
delta_t = cos_phi_indices / sampling_rate
phi = 360 * frequency * delta_t
print(phi)
cos_phi_values = np.cos(2 * np.pi * frequency * delta_t)
cos_phi_size = len(cos_phi_values)

# Format arrays for C++
def format_array(name, values, arr_size, dtype="int16_t"):
    lines = [f"const {dtype} {name}[{arr_size}] = {{"]
    for v in values:
        if dtype == "float":
            lines.append(f"    {v:.6f}f,")
        else:
            lines.append(f"    {v},")
    lines[-1] = lines[-1].rstrip(',')  # Remove trailing comma from last element
    lines.append("};\n")
    return lines

# Build header content
header_lines = [
    "#ifndef SIGNAL_DATA_H",
    "#define SIGNAL_DATA_H\n",
    "#include <cstdint>\n",
    f"#define TOTAL_SAMPLES {samples}",
    f"#define COS_PHI_TABLE_SIZE {cos_phi_size}\n",
]

# Add arrays
header_lines += format_array("CURRENT_SIGNAL", current_signal, "TOTAL_SAMPLES")
header_lines += format_array("VOLTAGE_SIGNAL", voltage_signal, "TOTAL_SAMPLES")
header_lines += format_array("COS_PHI_TABLE", cos_phi_values, "COS_PHI_TABLE_SIZE", dtype="float")

header_lines.append("#endif // SIGNAL_DATA_H")

# Write to header file
with open("power_signal_data.h", "w") as f:
    f.write("\n".join(header_lines))

print("Header file 'power_signal_data.h' generated with requested formatting.")
