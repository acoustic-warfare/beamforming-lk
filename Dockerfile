# Use an official C++ base image
FROM gcc:latest

# Set the working directory in the container
WORKDIR /app

# Install necessary packages - OpenCV and Eigen3
RUN apt-get update && \
    apt-get install -y \
    libopencv-dev \
    libeigen3-dev \
    librtaudio-dev \
    # rtaudio \
    cmake \
    && rm -rf /var/lib/apt/lists/*

# Copy the CMakeLists.txt file into the container
COPY CMakeLists.txt .

# Create src and build directories in the container
RUN mkdir src build lib 

# Copy the contents of the src directory into the container
COPY src/ ./src/
COPY lib/ ./lib/

# Set the working directory to the build directory
WORKDIR /app/build

# Run CMake to build the project inside the container
#RUN cmake ../src

# Default command to run when the container starts
CMD ["bash"]
