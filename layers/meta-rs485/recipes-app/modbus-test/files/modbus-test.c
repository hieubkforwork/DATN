/* C example */
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>

struct sensor_data {
    int32_t  temperature;
    uint32_t humidity;
    uint32_t pressure;
    uint32_t lux;
};

int main(void) {
    int fd = open("/dev/sensorhub", O_RDONLY);
    struct sensor_data d;
    read(fd, &d, sizeof(d));
    printf("Temp: %d.%u°C  Hum: %u.%u%%  Pres: %u.%uhPa  Lux: %u\n",
           d.temperature / 10, abs(d.temperature) % 10,
           d.humidity / 10, d.humidity % 10,
           d.pressure / 10, d.pressure % 10,
           d.lux);
    close(fd);
}
