import 'package:flutter/material.dart';
import '../../services/settings_service.dart';

class SettingsPage extends StatefulWidget {
  @override
  _SettingsPageState createState() => _SettingsPageState();
}

class _SettingsPageState extends State<SettingsPage> {
  SettingsService? _settingsService;
  late TextEditingController _ipController;
  late TextEditingController _portController;
  late TextEditingController _chatterTopicController;
  late TextEditingController _cmdVelTopicController;
  late TextEditingController _counterTopicController;
  late TextEditingController _cameraTopicController;
  late TextEditingController _cameraPortController;

  bool _isLoading = true;

  @override
  void initState() {
    super.initState();
    _ipController = TextEditingController();
    _portController = TextEditingController();
    _chatterTopicController = TextEditingController();
    _cmdVelTopicController = TextEditingController();
    _counterTopicController = TextEditingController();
    _cameraTopicController = TextEditingController();
    _cameraPortController = TextEditingController();
    _loadSettings();
  }

  Future<void> _loadSettings() async {
    _settingsService = await SettingsService.getInstance();
    setState(() {
      _ipController.text = _settingsService!.ipAddress;
      _portController.text = _settingsService!.port;
      _chatterTopicController.text = _settingsService!.chatterTopic;
      _cmdVelTopicController.text = _settingsService!.cmdVelTopic;
      _counterTopicController.text = _settingsService!.counterTopic;
      _cameraTopicController.text = _settingsService!.cameraTopic;
      _cameraPortController.text = _settingsService!.cameraPort;
      _isLoading = false;
    });
  }

  Future<void> _saveSettings() async {
    if (_settingsService != null) {
      await _settingsService!.setIpAddress(_ipController.text);
      await _settingsService!.setPort(_portController.text);
      await _settingsService!.setChatterTopic(_chatterTopicController.text);
      await _settingsService!.setCmdVelTopic(_cmdVelTopicController.text);
      await _settingsService!.setCounterTopic(_counterTopicController.text);
      await _settingsService!.setCameraTopic(_cameraTopicController.text);
      await _settingsService!.setCameraPort(_cameraPortController.text);
      
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text('Settings saved successfully!')),
      );
    }
  }

  Future<void> _resetToDefaults() async {
    if (_settingsService != null) {
      await _settingsService!.resetToDefaults();
      await _loadSettings();
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text('Settings reset to defaults!')),
      );
    }
  }

  @override
  void dispose() {
    _ipController.dispose();
    _portController.dispose();
    _chatterTopicController.dispose();
    _cmdVelTopicController.dispose();
    _counterTopicController.dispose();
    _cameraTopicController.dispose();
    _cameraPortController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    if (_isLoading) {
      return Scaffold(
        appBar: AppBar(title: Text('Settings')),
        body: Center(child: CircularProgressIndicator()),
      );
    }

    return Scaffold(
      appBar: AppBar(
        title: Text('Settings'),
        actions: [
          TextButton(
            onPressed: _resetToDefaults,
            child: Text('Reset', style: TextStyle(color: Colors.white)),
          ),
        ],
      ),
      body: Padding(
        padding: const EdgeInsets.all(16.0),
        child: ListView(
          children: [
            Text(
              'ROS Connection',
              style: Theme.of(context).textTheme.headline6,
            ),
            SizedBox(height: 16),
            TextField(
              controller: _ipController,
              decoration: InputDecoration(
                labelText: 'IP Address',
                hintText: '192.168.1.11',
                border: OutlineInputBorder(),
              ),
            ),
            SizedBox(height: 16),
            TextField(
              controller: _portController,
              keyboardType: TextInputType.number,
              decoration: InputDecoration(
                labelText: 'ROS Port',
                hintText: '9090',
                border: OutlineInputBorder(),
              ),
            ),
            SizedBox(height: 24),
            Text(
              'ROS Topics',
              style: Theme.of(context).textTheme.headline6,
            ),
            SizedBox(height: 16),
            TextField(
              controller: _chatterTopicController,
              decoration: InputDecoration(
                labelText: 'Chatter Topic',
                hintText: '/chatter',
                border: OutlineInputBorder(),
              ),
            ),
            SizedBox(height: 16),
            TextField(
              controller: _cmdVelTopicController,
              decoration: InputDecoration(
                labelText: 'Cmd Vel Topic',
                hintText: '/cmd_vel',
                border: OutlineInputBorder(),
              ),
            ),
            SizedBox(height: 16),
            TextField(
              controller: _counterTopicController,
              decoration: InputDecoration(
                labelText: 'Counter Topic',
                hintText: '/counter',
                border: OutlineInputBorder(),
              ),
            ),
            SizedBox(height: 24),
            Text(
              'Camera Settings',
              style: Theme.of(context).textTheme.headline6,
            ),
            SizedBox(height: 16),
            TextField(
              controller: _cameraTopicController,
              decoration: InputDecoration(
                labelText: 'Camera Topic',
                hintText: '/camera/rgb/image_raw',
                border: OutlineInputBorder(),
              ),
            ),
            SizedBox(height: 16),
            TextField(
              controller: _cameraPortController,
              keyboardType: TextInputType.number,
              decoration: InputDecoration(
                labelText: 'Camera Port',
                hintText: '8080',
                border: OutlineInputBorder(),
              ),
            ),
            SizedBox(height: 32),
            ElevatedButton(
              onPressed: _saveSettings,
              child: Text('Save Settings'),
              style: ElevatedButton.styleFrom(
                padding: EdgeInsets.symmetric(vertical: 16),
              ),
            ),
          ],
        ),
      ),
    );
  }
}