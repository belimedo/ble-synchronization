import numpy as np

time_value_is_held = 0.05
sampling_frequency = 100

N = np.floor(time_value_is_held * sampling_frequency).astype(int)

pattern = np.arange(0x0000, 0xFFFF, 0x0101, dtype=np.uint16)
electric_current_data = np.repeat(pattern, N)
increments = np.arange(len(electric_current_data))
increments = increments % N
voltage_data = (electric_current_data - 0x5555 + increments) % 0xFFFF
print(max(voltage_data))
# voltage_data = (electric_current_data - 0x5555) % 0xFFFF

# Create header file content
header_content = """\
#ifndef LOGGED_DATA_H
#define LOGGED_DATA_H

#include <stdint.h>

#define ELECTRIC_CURRENT_DATA_SIZE {electric_current_size}
#define VOLTAGE_DATA_SIZE {voltage_size}

static const uint16_t ELECTRIC_CURRENT_DATA[ELECTRIC_CURRENT_DATA_SIZE] = {{
{electric_current}
}};

static const uint16_t VOLTAGE_DATA[VOLTAGE_DATA_SIZE] = {{
{voltage}
}};

#endif // LOGGED_DATA_H
""".format(
    electric_current_size=len(electric_current_data),
    electric_current=',\n'.join('    0x{:04X}'.format(x) for x in electric_current_data),
    voltage_size=len(voltage_data),
    voltage=',\n'.join('    0x{:04X}'.format(x) for x in voltage_data)
)

# Write to a .h file
with open('./sample_values.h', 'w') as f:
    f.write(header_content)

print("Header file 'sample_values.h' generated successfully!")


print(len(electric_current_data))
print(len(voltage_data))