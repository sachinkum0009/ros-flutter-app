import 'package:flutter/material.dart';
import 'package:control_pad/control_pad.dart';
import 'package:my_app/screen/camera_page/camera_page.dart';
import 'package:my_app/services/settings_service.dart';
import 'dart:math';
import 'package:roslib/roslib.dart';

class JoyStickPage extends StatefulWidget {
  @override
  _JoyStickPageState createState() => _JoyStickPageState();
}

class _JoyStickPageState extends State<JoyStickPage> {
  Ros? ros;
  Topic? chatter;
  Topic? counter;
  Topic? cmd_vel;
  SettingsService? _settingsService;
  bool _isInitialized = false;

  void _move(double _degrees, double _distance) {
    print(
        'Degree:' + _degrees.toString() + ' Distance:' + _distance.toString());
    double radians = _degrees * ((22 / 7) / 180);
    double linear_speed = cos(radians) * _distance;
    double angular_speed = -sin(radians) * _distance;

    publishCmd(linear_speed, angular_speed);
  }

  @override
  void initState() {
    super.initState();
    _initializeSettings();
  }

  Future<void> _initializeSettings() async {
    _settingsService = await SettingsService.getInstance();
    await _initializeROS();
    setState(() {
      _isInitialized = true;
    });
  }

  Future<void> _initializeROS() async {
    if (_settingsService != null) {
      ros = Ros(url: _settingsService!.rosUrl);
      chatter = Topic(
          ros: ros!,
          name: _settingsService!.chatterTopic,
          type: "std_msgs/String",
          reconnectOnClose: true,
          queueLength: 10,
          queueSize: 10);

      cmd_vel = Topic(
          ros: ros!,
          name: _settingsService!.cmdVelTopic,
          type: "geometry_msgs/Twist",
          reconnectOnClose: true,
          queueLength: 10,
          queueSize: 10);

      counter = Topic(
        ros: ros!,
        name: _settingsService!.counterTopic,
        type: "std_msgs/String",
        reconnectOnClose: true,
        queueSize: 10,
        queueLength: 10,
      );
    }
  }

  void initConnection() async {
    if (ros != null && chatter != null && counter != null && cmd_vel != null) {
      ros!.connect();
      await chatter!.subscribe();

      await counter!.advertise();
      await cmd_vel!.advertise();
      setState(() {});
    }
  }

  void publishCounter() async {
    if (counter != null) {
      var msg = {'data': 'hello'};
      await counter!.publish(msg);
      print('done publihsed');
    }
  }

  void publishCmd(double _linear_speed, double _angular_speed) async {
    if (cmd_vel != null) {
      var linear = {'x': _linear_speed, 'y': 0.0, 'z': 0.0};
      var angular = {'x': 0.0, 'y': 0.0, 'z': _angular_speed};
      var twist = {'linear': linear, 'angular': angular};
      await cmd_vel!.publish(twist);
      print('cmd published');
    }
  }

  void destroyConnection() async {
    if (chatter != null) {
      await chatter!.unsubscribe();
    }
    if (counter != null) {
      await counter!.unadvertise();
    }
    if (ros != null) {
      await ros!.close();
    }
    setState(() {});
  }

  @override
  Widget build(BuildContext context) {
    if (!_isInitialized || ros == null) {
      return Center(child: CircularProgressIndicator());
    }

    return StreamBuilder<Object>(
      stream: ros!.statusStream,
      builder: (context, snapshot) {
        return Center(
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.center,
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              Container(
                height: 205,
                child: MyWebView(
                    title: 'Camera',
                    selectedUrl: _settingsService!.cameraUrl),
              ),
              Padding(padding: EdgeInsets.all(40)),
              ActionChip(
                label: Text(snapshot.data == Status.CONNECTED
                    ? 'DISCONNECT'
                    : 'CONNECT'),
                backgroundColor: snapshot.data == Status.CONNECTED
                    ? Colors.green[300]
                    : Colors.grey[300],
                onPressed: () {
                  print(snapshot.data);
                  if (snapshot.data != Status.CONNECTED) {
                    this.initConnection();
                  } else {
                    this.destroyConnection();
                  }
                },
              ),
              Padding(padding: EdgeInsets.all(20)),
              JoystickView(
                onDirectionChanged: (_degrees, _distance) =>
                    _move(_degrees, _distance),
              ),
            ],
          ),
        );
      },
    );
  }
}

/*
JoystickView(
          onDirectionChanged: (_degrees, _distance) =>
              _move(_degrees, _distance),
        ),
*/
