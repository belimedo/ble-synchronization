import numpy as np

time_value_is_held = 4
sampling_frequency = 5

N = np.floor(time_value_is_held * sampling_frequency)

pattern = np.arange(0x00, 0xFF, 0x11, dtype=np.uint8)
logged_data = np.repeat(pattern, N)

# Create header file content
header_content = """\
#ifndef LOGGED_DATA_H
#define LOGGED_DATA_H

#include <stdint.h>

#define LOGGED_DATA_SIZE {size}

static const uint8_t LOGGED_DATA[LOGGED_DATA_SIZE] = {{
{data}
}};

#endif // LOGGED_DATA_H
""".format(
    size=len(logged_data),
    data=',\n'.join('    0x{:02X}'.format(x) for x in logged_data)
)

# Write to a .h file
with open('./logged_data.h', 'w') as f:
    f.write(header_content)

print("Header file 'logged_data.h' generated successfully!")


print(len(logged_data))