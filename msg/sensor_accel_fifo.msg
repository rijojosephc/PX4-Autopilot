uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id          # unique device ID for the sensor that does not change between power cycles

float32 dt                # delta time between samples (microseconds)
float32 scale

uint8 samples             # number of valid samples

int16[32] x               # array of raw acceleration in the sensor X axis
int16[32] y               # array of raw acceleration in the sensor Y axis
int16[32] z               # array of raw acceleration in the sensor Z axis

uint8 rotation            # Direction the sensor faces (see Rotation enum)

float32 temperature       # temperature in degrees celsius

uint32 error_count

uint8 ORB_QUEUE_LENGTH = 8
