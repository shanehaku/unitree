# 指定編譯器
CXX = g++

# 指定來源檔案
SRC = cameraSDK.cpp

# 指定目標執行檔名稱
TARGET = test

# 包含 OpenCV 和 realsense2 的標頭檔和庫
OPENCV_FLAGS = $(shell pkg-config --cflags --libs opencv4)
REALENSE_FLAGS = -lrealsense2
PTHREAD_FLAGS = -pthread

# 編譯目標規則
$(TARGET): $(SRC)
	$(CXX) $(SRC) -o $(TARGET) $(OPENCV_FLAGS) $(REALENSE_FLAGS) $(PTHREAD_FLAGS)

# 清除編譯產物
clean:
	rm -f $(TARGET)
