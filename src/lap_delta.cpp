#include "core_pins.h"
#include "map.hpp"
#include "math.h"

struct chronoPoint {
  uint32_t laptimeMili;
  int16_t energyWattH;
  uint16_t prg;
  int32_t lat;
  int32_t lon;

  chronoPoint(uint32_t laptimeMili, int16_t energyWattH, uint16_t prg,
              int32_t lat, int32_t lon);
};

class chronoMap {
  uint32_t startTime;
  uint16_t currentPoint = 0;
  float totalEnergy;
  uint32_t totalLapTime;

  chronoPoint data[4800];

public:
  // Define start time to get laptime
  void startLap() { startTime = millis(); };
  int32_t getEnergy() { return totalEnergy; };

  uint16_t getProgress() { return data[currentPoint].prg; }
  uint16_t getProgress(uint16_t index) {
    if (index < 4800)
      return data[index].prg;
  }

  uint32_t getTime() { return data[currentPoint].laptimeMili; }
  uint32_t getTime(uint16_t index) {
    if (index < 4800)
      return data[index].laptimeMili;
  };

  // Add a point to the lap list, if this returns true, the lap is completed
  bool addPoint(int32_t lat, int32_t lon, int16_t amps) {
    currentPoint++;

    // This if statement should only happen after 4 minutes @ 20 Hz and will end
    // the lap so the teensy doesn't fucking die
    if (currentPoint != 4800) {
      uint32_t currentTime = millis() - startTime;

      // get instant amp hours and add to the rolling number
      float instantPower =
          amps *
          ((currentTime - data[currentPoint - 1].laptimeMili) / 3600000.0);
      totalEnergy += instantPower;

      // Variables to hold calced values
      float distance;
      float minDistance;
      uint16_t minIndex;

      // Iterate thru every single point to find the closest one
      // TODO: Make this not stupid
      for (uint32_t i; i < numberOfPoints; i++) {
        distance =
            sqrt(pow((lat - points[i].lat), 2) + pow((lon - points[i].lon), 2));

        // Check if its closer, if it is replace our lowest and go
        if (distance < minDistance) {
          minDistance = distance;
          minIndex = i;
        }
      }

      // Set our progress to whatever the closest point was
      uint16_t progress = points[minIndex].prg;

      // Check if the lap is done by seeing if the driver went more than 90%
      // "backwards" with their progress in 20ms
      if (progress - data[currentPoint - 1].prg < -50000) {
        // Add last point
        data[currentPoint] =
            chronoPoint(currentTime, instantPower, uint16_t(65536), lat, lon);

        // Calc final laptime
        totalLapTime = millis() - startTime;

        // End the lap
        return true;
      } else {
        // Add point
        data[currentPoint] =
            chronoPoint(currentTime, instantPower, progress, lat, lon);

        // Get current laptime
        totalLapTime = millis() - startTime;

        // Don't end the lap
        return false;
      }
    } else {
      // End the lap
      return true;
    }
  };
};

class chronoDelta {
  chronoMap bestLap;
  chronoMap current;

  enum saveType {
    overBest,
    EEPROM,
    CAN,
  };

  int32_t updateDelta() {
    uint16_t currentProgress = current.getProgress();

    uint32_t delta = bestLap.getTime();

    return 0;
  };

  void saveLap(saveType type) {
    switch (type) {
    case overBest:
      bestLap = current;
      break;

    case EEPROM:
      break;

    case CAN:
      break;
    }
  };
};
