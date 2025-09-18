import 'package:shared_preferences/shared_preferences.dart';

class SettingsService {
  static const String _ipAddressKey = 'ros_ip_address';
  static const String _portKey = 'ros_port';
  static const String _chatterTopicKey = 'chatter_topic';
  static const String _cmdVelTopicKey = 'cmd_vel_topic';
  static const String _counterTopicKey = 'counter_topic';
  static const String _cameraTopicKey = 'camera_topic';
  static const String _cameraPortKey = 'camera_port';

  // Default values
  static const String _defaultIpAddress = '192.168.1.11';
  static const String _defaultPort = '9090';
  static const String _defaultChatterTopic = '/chatter';
  static const String _defaultCmdVelTopic = '/cmd_vel';
  static const String _defaultCounterTopic = '/counter';
  static const String _defaultCameraTopic = '/camera/rgb/image_raw';
  static const String _defaultCameraPort = '8080';

  static SettingsService? _instance;
  SharedPreferences? _prefs;

  SettingsService._();

  static Future<SettingsService> getInstance() async {
    if (_instance == null) {
      _instance = SettingsService._();
      await _instance!._init();
    }
    return _instance!;
  }

  Future<void> _init() async {
    _prefs = await SharedPreferences.getInstance();
  }

  // IP Address
  String get ipAddress => _prefs?.getString(_ipAddressKey) ?? _defaultIpAddress;
  Future<void> setIpAddress(String value) async => await _prefs?.setString(_ipAddressKey, value);

  // Port
  String get port => _prefs?.getString(_portKey) ?? _defaultPort;
  Future<void> setPort(String value) async => await _prefs?.setString(_portKey, value);

  // Topics
  String get chatterTopic => _prefs?.getString(_chatterTopicKey) ?? _defaultChatterTopic;
  Future<void> setChatterTopic(String value) async => await _prefs?.setString(_chatterTopicKey, value);

  String get cmdVelTopic => _prefs?.getString(_cmdVelTopicKey) ?? _defaultCmdVelTopic;
  Future<void> setCmdVelTopic(String value) async => await _prefs?.setString(_cmdVelTopicKey, value);

  String get counterTopic => _prefs?.getString(_counterTopicKey) ?? _defaultCounterTopic;
  Future<void> setCounterTopic(String value) async => await _prefs?.setString(_counterTopicKey, value);

  String get cameraTopic => _prefs?.getString(_cameraTopicKey) ?? _defaultCameraTopic;
  Future<void> setCameraTopic(String value) async => await _prefs?.setString(_cameraTopicKey, value);

  // Camera Port
  String get cameraPort => _prefs?.getString(_cameraPortKey) ?? _defaultCameraPort;
  Future<void> setCameraPort(String value) async => await _prefs?.setString(_cameraPortKey, value);

  // Convenience methods
  String get rosUrl => 'ws://$ipAddress:$port';
  String get cameraUrl => 'http://$ipAddress:$cameraPort/stream?topic=$cameraTopic&type=mjpeg&quality=30&width=100&height=100&default_transport=compressed';

  // Reset to defaults
  Future<void> resetToDefaults() async {
    await setIpAddress(_defaultIpAddress);
    await setPort(_defaultPort);
    await setChatterTopic(_defaultChatterTopic);
    await setCmdVelTopic(_defaultCmdVelTopic);
    await setCounterTopic(_defaultCounterTopic);
    await setCameraTopic(_defaultCameraTopic);
    await setCameraPort(_defaultCameraPort);
  }
}