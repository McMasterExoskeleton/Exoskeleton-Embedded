/*
 * Loads and processes roll angles for a given IMU data (e.g., "Right Knee") from a JSON file.
 * Assumes:
 * - IMU data is in degrees and must be converted to radians.
 * - Offset is applied to align the most extended position to 0 rad.
 *
*/

#include <string>
#include <vector>

namespace IMUDataLoader {

    struct JointData {
        std::vector<double> timestamps;  // in seconds
        std::vector<double> angles_rad;  // shifted so that 0 = full extension
    };

    JointData loadJointData(const std::string& filepath, const std::string& location);
}
