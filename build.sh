#!/bin/bash

# StreamManager Build Script

set -e  # Exit on any error

echo "StreamManager Build Script"
echo "=========================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if we're in the right directory
if [ ! -f "CMakeLists.txt" ]; then
    print_error "CMakeLists.txt not found. Please run this script from the StreamManager root directory."
    exit 1
fi

# Parse command line arguments
BUILD_TYPE="Debug"
CLEAN_BUILD=false
BUILD_EXAMPLES=true
ROS2_BUILD=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --release)
            BUILD_TYPE="Release"
            shift
            ;;
        --clean)
            CLEAN_BUILD=true
            shift
            ;;
        --no-examples)
            BUILD_EXAMPLES=false
            shift
            ;;
        --ros2)
            ROS2_BUILD=true
            shift
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo "Options:"
            echo "  --debug        Build in Debug mode (default: Release)"
            echo "  --clean        Clean build directory before building"
            echo "  --no-examples  Don't build examples"
            echo "  --ros2         Build with ROS2 support using colcon"
            echo "  --help         Show this help message"
            exit 0
            ;;
        *)
            print_error "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

print_status "Build configuration:"
echo "  Build type: $BUILD_TYPE"
echo "  Clean build: $CLEAN_BUILD"
echo "  Build examples: $BUILD_EXAMPLES"
echo "  ROS2 build: $ROS2_BUILD"
echo ""

# Check dependencies
print_status "Checking dependencies..."

# Check for OpenCV
if ! pkg-config --exists opencv4; then
    print_warning "OpenCV4 not found via pkg-config, checking for opencv..."
    if ! pkg-config --exists opencv; then
        print_error "OpenCV not found. Please install OpenCV development libraries."
        echo "  Ubuntu/Debian: sudo apt install libopencv-dev"
        exit 1
    fi
fi

# Check for yaml-cpp
if ! pkg-config --exists yaml-cpp; then
    print_warning "yaml-cpp not found via pkg-config. Make sure it's installed."
    echo "  Ubuntu/Debian: sudo apt install libyaml-cpp-dev"
fi

# Check for iceoryx (this is trickier as it might not have pkg-config)
if [ ! -d "/usr/local/include/iceoryx_posh" ] && [ ! -d "/usr/include/iceoryx_posh" ]; then
    print_warning "iceoryx headers not found in standard locations."
    print_warning "Make sure iceoryx is installed and headers are available."
fi

# ROS2 specific checks
if [ "$ROS2_BUILD" = true ]; then
    print_status "Checking ROS2 environment..."
    
    if [ -z "$ROS_DISTRO" ]; then
        print_error "ROS2 environment not sourced. Please run:"
        echo "  source /opt/ros/humble/setup.bash  # or your ROS2 distro"
        exit 1
    fi
    
    print_status "ROS2 distro: $ROS_DISTRO"
    
    # Check for colcon
    if ! command -v colcon &> /dev/null; then
        print_error "colcon not found. Please install colcon:"
        echo "  sudo apt install python3-colcon-common-extensions"
        exit 1
    fi
fi

# Build process
if [ "$ROS2_BUILD" = true ]; then
    print_status "Building with ROS2/colcon..."
    
    # Go to workspace root (assuming we're in src/StreamManager)
    cd ../..
    
    if [ "$CLEAN_BUILD" = true ]; then
        print_status "Cleaning previous build..."
        rm -rf build/ install/ log/
    fi
    
    # Build with colcon
    print_status "Running colcon build..."
    colcon build --packages-select stream_manager --cmake-args -DCMAKE_BUILD_TYPE=$BUILD_TYPE
    
    if [ $? -eq 0 ]; then
        print_status "Build successful!"
        print_status "To use the library, source the workspace:"
        echo "  source install/setup.bash"
    else
        print_error "Build failed!"
        exit 1
    fi
    
else
    print_status "Building with CMake..."
    
    # Create build directory
    if [ "$CLEAN_BUILD" = true ] && [ -d "build" ]; then
        print_status "Cleaning previous build..."
        rm -rf build/
    fi
    
    mkdir -p build
    cd build
    
    # Configure
    print_status "Configuring with CMake..."
    CMAKE_ARGS="-DCMAKE_BUILD_TYPE=$BUILD_TYPE"
    
    if [ "$BUILD_EXAMPLES" = true ]; then
        CMAKE_ARGS="$CMAKE_ARGS -DBUILD_EXAMPLES=ON"
    fi
    
    cmake .. $CMAKE_ARGS
    
    if [ $? -ne 0 ]; then
        print_error "CMake configuration failed!"
        exit 1
    fi
    
    # Build
    print_status "Building..."
    make -j$(nproc)
    
    if [ $? -eq 0 ]; then
        print_status "Build successful!"
        print_status "Library built in: $(pwd)/libStreamManager.so"
        
        if [ "$BUILD_EXAMPLES" = true ] && [ -f "examples/basic_usage" ]; then
            print_status "Example executable: $(pwd)/examples/basic_usage"
        fi
        
        # Optional: Install
        read -p "Do you want to install the library system-wide? (y/N): " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            print_status "Installing..."
            sudo make install
            print_status "Installation complete!"
        fi
        
    else
        print_error "Build failed!"
        exit 1
    fi
fi

print_status "Build process completed successfully!"

# Show next steps
echo ""
print_status "Next steps:"
if [ "$ROS2_BUILD" = true ]; then
    echo "1. Source the workspace: source install/setup.bash"
    echo "2. Run the example: ros2 run stream_manager basic_usage"
else
    echo "1. Copy config file: cp config/default_config.yaml /path/to/your/project/"
    echo "2. Include headers: #include \"stream_manager/stream_manager.hpp\""
    echo "3. Link library: -lStreamManager"
    if [ -f "build/examples/basic_usage" ]; then
        echo "4. Try the example: ./build/examples/basic_usage"
    fi
fi

echo ""
print_status "For more information, see README.md"