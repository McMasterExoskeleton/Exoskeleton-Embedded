#ifndef LOW_PASS_FILTER_H
#define LOW_PASS_FILTER_H

class LowPassFilter {
public:
    // Default constructor required for std::array
    LowPassFilter() : alpha(0.5), initialized(false), filtered_value(0.0) {}

    // Constructor with custom alpha
    LowPassFilter(double alpha) : alpha(alpha), initialized(false), filtered_value(0.0) {}

    void initialize(double value) {
        filtered_value = value;
        initialized = true;
    }

    double update(double measurement) {
        if (!initialized) {
            initialize(measurement);
        }
        filtered_value = alpha * measurement + (1 - alpha) * filtered_value;
        return filtered_value;
    }

    bool is_initialized() const {
        return initialized;
    }

private:
    double alpha;
    double filtered_value;
    bool initialized;
};

#endif // LOW_PASS_FILTER_H
