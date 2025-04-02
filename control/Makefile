CXX = g++
CXXFLAGS = -std=c++17 -Wall

TARGET = exo_control_test

SRC = \
    FSM_general_controller/main.cpp \
    FSM_general_controller/ActiveState.cpp \
    FSM_general_controller/IdleState.cpp \
    FSM_general_controller/ErrorState.cpp \
    FSM_general_controller/State.cpp \
    FSM_general_controller/Sensor.cpp \
    Joint_Estimator/JointEstimator.cpp \
    Torque_Controller/TorqueController.cpp \
	  IMU_DataLoader/IMUDataLoader.cpp
    FSM_general_controller/StateParameters.cpp \
    FSM_general_controller/Timer.cpp

INCLUDES = -IFSM_general_controller -IJoint_Estimator -ITorque_Controller -IIMU_DataLoader

all: $(TARGET)

$(TARGET):
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(SRC) -o $(TARGET)

run: $(TARGET)
	./$(TARGET)

plot:
	python3 Testing/outputAnalysis.py

run_plot: run plot

clean:
	rm -f $(TARGET)

