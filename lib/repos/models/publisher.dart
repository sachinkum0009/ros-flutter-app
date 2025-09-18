import 'package:flutter/material.dart';
import 'package:roslib/roslib.dart';
import 'package:my_app/services/settings_service.dart';

class Publisher extends StatefulWidget {
  @override
  _PublisherState createState() => _PublisherState();
}

class _PublisherState extends State<Publisher> {
  Ros? ros;
  Topic? chatter;
  Topic? counter;
  Topic? cmd_vel;
  SettingsService? _settingsService;
  bool _isInitialized = false;

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

  void publishCmd() async {
    if (cmd_vel != null) {
      var linear = {'x': 34.0, 'y': 0.0, 'z': 0.0};
      var angular = {'x': 0.0, 'y': 0.0, 'z': 32.0};
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
              StreamBuilder(
                stream: chatter!.subscription,
                builder: (context2, snapshot2) {
                  if (snapshot2.hasData) {
                    return Text('${snapshot2.data['msg']}');
                  } else {
                    return CircularProgressIndicator();
                  }
                },
              ),
              RaisedButton(onPressed: () {
                publishCounter();
              }),
              RaisedButton(onPressed: () {
                publishCmd();
              }),
            ],
          ),
        );
      },
    );
  }
}
