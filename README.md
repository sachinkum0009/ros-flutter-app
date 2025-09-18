# ROS FLUTTER APP

Flutter Base Project for ROS Teleop Joy control.

## Screenshots

### Main Application
![Main App](flutter_01.png)

### Settings Page
![Settings Page](https://github.com/user-attachments/assets/a86842e5-21e0-40f5-aed3-54bdc60d418f)

The settings page allows users to configure:
- ROS connection (IP address and port)
- ROS topics (chatter, cmd_vel, counter)
- Camera settings (topic and port)

## CI/CD Pipeline

This repository is configured with GitHub Actions to automatically build and release the Flutter app.

### Build Process ✅
- **On every push/PR**: Builds and tests the Flutter app
- **Modern Actions**: Uses latest GitHub Actions with caching for faster builds
- **Code Analysis**: Runs `flutter analyze` to catch potential issues
- **Artifacts**: APK files are stored as build artifacts for 30 days

### Release Process 🚀
- **Automatic releases**: Created when you push a version tag (e.g., `v1.0.0`)
- **APK attachment**: Release APK is automatically built and attached to GitHub releases
- **Rich release notes**: Auto-generated with emojis, installation instructions, and configuration details

### Creating a Release
To create a new release with APK:

```bash
# Create and push a version tag
git tag v1.0.0
git push origin v1.0.0
```

This will trigger the release workflow, build a fresh APK, and create a GitHub release with the APK attached.

### Recent CI/CD Improvements
- 🔧 **Fixed deprecated actions**: Updated to modern GitHub Actions
- ⚡ **Added caching**: Faster builds with Flutter dependency caching  
- 🎯 **Better error handling**: Improved workflow reliability
- 📱 **Enhanced release notes**: Rich formatting with installation guides

## Task List

- [x] Splash Screen
- [ ] BLoC Architecture
- [x] JoyStick
- [x] ROS send topic messages
- [ ] View Map
- [ ] Visualize URDF
- [x] Editable IP Address and topic names
- [x] CI/CD Pipeline with APK releases
- [ ] add files for the web camera and navigation map

## ROS launch files

```bash
roscore

export TURTLEBOT3_MODEL=waffle

roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

roslaunch rosbridge_server rosbridge_websocket.launch


```
