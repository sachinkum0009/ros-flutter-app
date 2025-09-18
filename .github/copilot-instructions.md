# ROS Flutter App

ROS Flutter App is a mobile application for teleoperation of ROS robots. It provides a joystick interface, camera streaming capabilities, and WebSocket communication with ROS systems. The app is built using Flutter and targets primarily Android devices, with iOS support available.

Always reference these instructions first and fallback to search or bash commands only when you encounter unexpected information that does not match the info here.

## Working Effectively

### Environment Setup Reality Check
- **CRITICAL**: This environment may have network restrictions affecting Flutter SDK downloads
- Java 17 and Android SDK are available (ANDROID_HOME=/usr/local/lib/android/sdk)
- Gradle 9.0.0 is installed system-wide
- **What Works**: Git clone, file editing, repository analysis, pub.dev API access
- **What Fails**: Flutter Dart SDK download from storage.googleapis.com (returns HTML instead of binary)
- **Expected Challenges**: 
  - Flutter SDK auto-download from storage.googleapis.com fails with "End-of-central-directory signature not found"
  - Must use git clone method for Flutter installation (gets repository but not working SDK)
  - Cannot actually run `flutter pub get`, `flutter build`, or `flutter run` due to missing Dart SDK
  - CI environment likely has better network access and will work normally

### Bootstrap and Setup
- **CRITICAL**: Flutter SDK download may fail due to network restrictions:
  ```bash
  # Method 1: Git clone (recommended)
  cd /tmp
  git clone https://github.com/flutter/flutter.git -b stable --depth 1
  export PATH="/tmp/flutter/bin:$PATH"
  
  # Method 2: Direct Download (may fail due to network restrictions)
  # curl -L https://storage.googleapis.com/flutter_infra_release/releases/stable/linux/flutter_linux_3.13.9-stable.tar.xz -o flutter.tar.xz
  # tar -xf flutter.tar.xz
  
  # Add to shell profile for persistence
  echo 'export PATH="/tmp/flutter/bin:$PATH"' >> ~/.bashrc
  source ~/.bashrc
  ```

- **WARNING**: Flutter SDK downloads from storage.googleapis.com may be blocked/corrupted in some environments.
  If you get "End-of-central-directory signature not found" errors, this indicates network blocking.

- Verify Flutter installation:
  ```bash
  flutter --version
  flutter doctor
  ```
  **TIMING**: Initial setup may take 5-10 minutes to download Dart SDK. NEVER CANCEL. Set timeout to 15+ minutes.

- Install project dependencies:
  ```bash
  cd /home/runner/work/ros-flutter-app/ros-flutter-app
  flutter pub get
  ```
  **TIMING**: Takes 2-5 minutes. NEVER CANCEL. Set timeout to 10+ minutes.

### Build and Test
- **CRITICAL WARNING**: Network restrictions in this environment prevent Flutter SDK from downloading required Dart SDK components
  - Git clone of Flutter repository succeeds
  - Subsequent `flutter doctor` or `flutter pub get` fails with "End-of-central-directory signature not found"
  - This is a known limitation - storage.googleapis.com downloads are blocked/corrupted

- If Flutter SDK setup succeeds:
  ```bash
  flutter build apk
  ```
  **TIMING**: Takes 10-20 minutes on first build. NEVER CANCEL. Set timeout to 30+ minutes.

- Build for debug (faster iteration):
  ```bash
  flutter build apk --debug
  ```
  **TIMING**: Takes 5-10 minutes. NEVER CANCEL. Set timeout to 15+ minutes.

- Run tests (if available):
  ```bash
  flutter test
  ```
  **NOTE**: Currently no test files exist in the project. Tests are commented out in CI workflow.

- Check for linting issues:
  ```bash
  flutter analyze
  ```
  **TIMING**: Takes 1-2 minutes. NEVER CANCEL. Set timeout to 5+ minutes.

### Running the Application
- **LIMITATION**: Cannot run Flutter apps in this environment due to network restrictions preventing Dart SDK download
- If Flutter SDK were working, these would be the commands:
  ```bash
  flutter run                    # Run on connected Android device or emulator
  flutter run --release         # Run in release mode
  ```
  **REQUIREMENT**: Requires Android device with USB debugging enabled or Android emulator running.

- Hot reload during development (if Flutter were working):
  - Press `r` in the running Flutter session to hot reload
  - Press `R` for hot restart
  - Press `q` to quit

## Validation
- **CRITICAL**: Always test the complete user journey after making changes:
  1. App launches with splash screen showing "Teleop Joy" for 5 seconds
  2. Main screen displays with joystick interface and camera view area
  3. CONNECT button allows WebSocket connection to ROS (requires ROS system at configured IP)
  4. Joystick movements generate appropriate ROS Twist messages
  5. Camera stream displays (requires camera stream at configured URL)

- **Manual Testing Requirements**:
  - Test app startup sequence and navigation flow
  - Verify joystick input generates console output with degree/distance values
  - Check ROS WebSocket connection status changes
  - Validate UI responsiveness and layout on different screen sizes

- **IMPORTANT**: Full validation requires external ROS system running at configured IP addresses:
  - `ws://192.168.1.11:9090` (primary configuration in joystick_page.dart)
  - `ws://192.168.1.4:9090` (alternative in publisher.dart)
  - Camera stream: `http://192.168.1.11:8080/stream?topic=/camera/rgb/image_raw&type=mjpeg`

- **ROS System Requirements for Full Testing**:
  ```bash
  # These commands need to be run on a ROS system for full validation
  roscore
  export TURTLEBOT3_MODEL=waffle
  roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
  roslaunch rosbridge_server rosbridge_websocket.launch
  ```

- **Environment Limitations**:
  - Flutter SDK downloads may fail due to network restrictions
  - ROS systems not available in CI environment
  - Full end-to-end testing requires physical robot or ROS simulation

- Always run `flutter analyze` before committing changes or CI will fail
- Build verification must complete successfully: `flutter build apk`

## Common Tasks

### Key Directories and Files
```
/home/runner/work/ros-flutter-app/ros-flutter-app/
├── lib/
│   ├── main.dart                    # App entry point
│   ├── app.dart                     # Root widget configuration
│   ├── screen/
│   │   ├── splash_page/            # 5-second splash screen
│   │   ├── home_page/              # Main navigation hub
│   │   ├── joystick_page/          # Primary joystick interface
│   │   ├── camera_page/            # WebView for camera streams
│   │   └── button_page/            # Additional control buttons
│   └── repos/models/
│       ├── publisher.dart          # ROS message publishing
│       ├── subscriber.dart         # ROS message subscription
│       └── random_words.dart       # Demo word generator
├── android/                        # Android-specific configuration
├── ios/                            # iOS-specific configuration
├── pubspec.yaml                    # Dependencies and configuration
└── .github/workflows/test.yml      # CI/CD pipeline
```

### Key Dependencies
- `roslib: ^0.0.3` - ROS WebSocket communication
- `control_pad: ^1.1.1+1` - Joystick widget
- `webview_flutter: ^1.0.3` - Camera stream display
- `flutter_webview_plugin: ^0.3.11` - Alternative WebView implementation

### Configuration Points
- **ROS WebSocket URLs**: Hardcoded in joystick_page.dart and publisher.dart
  - Default: `ws://192.168.1.11:9090` (joystick_page.dart)
  - Alternative: `ws://192.168.1.4:9090` (publisher.dart)
- **Camera Stream URL**: `http://192.168.1.11:8080/stream?topic=/camera/rgb/image_raw&type=mjpeg`
- **ROS Topics**:
  - `/cmd_vel` - Robot movement commands (geometry_msgs/Twist)
  - `/chatter` - Test communication (std_msgs/String)
  - `/counter` - Counter messages (std_msgs/String)

### Troubleshooting Common Issues
- **Flutter SDK download failures**: Network restrictions may block storage.googleapis.com downloads
  - Solution: Use `git clone https://github.com/flutter/flutter.git` method instead
  - Error message: "End-of-central-directory signature not found" indicates blocked downloads
- **Build failures**: Usually dependency-related, run `flutter pub get` first
- **WebSocket connection issues**: Check ROS system IP addresses in source code
- **Camera not displaying**: Verify camera stream URL and network connectivity
- **APK installation issues**: Ensure Android device has unknown sources enabled
- **Gradle wrapper missing**: Project has gradle-wrapper.properties but no gradlew script
  - Flutter creates this automatically during first build

### CI/CD Pipeline
- Builds on Ubuntu, Windows, and macOS
- Uses Java 12.x and stable Flutter channel
- Runs `flutter pub get` and `flutter build apk`
- Tests are currently disabled in workflow
- **TIMING**: CI build typically takes 15-25 minutes per platform

## Development Workflow
- **In Normal Environments**:
  1. Make code changes using hot reload for rapid iteration
  2. Test functionality manually with complete user scenarios
  3. Run `flutter analyze` to check for issues
  4. Build APK to verify no build errors: `flutter build apk`
  5. Commit changes - CI will automatically build and validate

- **In This Restricted Environment**:
  1. Make code changes by editing Dart files directly
  2. Analyze code structure and dependencies manually
  3. Use repository analysis and file examination for validation
  4. Commit changes - rely on CI for build validation
  5. Reference this documentation for Flutter-specific guidance

- **Code Analysis You CAN Do**:
  - Edit Dart source files in lib/ directory
  - Modify pubspec.yaml dependencies
  - Update Android configuration in android/ directory
  - Analyze code structure and imports
  - Review ROS WebSocket communication logic
  - Validate IP addresses and topic names in source code

## Architecture Notes
- Uses StatefulWidget pattern throughout
- ROS communication via WebSockets using roslib package
- Joystick generates polar coordinates (degrees, distance) converted to linear/angular velocities
- Camera display implemented with WebView widget
- Navigation uses MaterialPageRoute with replacement for splash screen