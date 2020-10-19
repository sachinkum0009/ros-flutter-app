import 'package:flutter/material.dart';
import 'package:roslib/roslib.dart';

class Publisher extends StatefulWidget {
  @override
  _PublisherState createState() => _PublisherState();
}

class _PublisherState extends State<Publisher> {
  Ros ros;
  Topic chatter;
  Topic counter;
  Topic cmd_vel;
  @override
  void initState() {
    ros = Ros(url: 'ws://192.168.42.104:9090');
    chatter = Topic(
        ros: ros,
        name: '/chatter',
        type: "std_msgs/String",
        reconnectOnClose: true,
        queueLength: 10,
        queueSize: 10);

    cmd_vel = Topic(
        ros: ros,
        name: '/cmd_vel',
        type: "geometry_msgs/Twist",
        reconnectOnClose: true,
        queueLength: 10,
        queueSize: 10);

    counter = Topic(
      ros: ros,
      name: '/counter',
      type: "std_msgs/String",
      reconnectOnClose: true,
      queueSize: 10,
      queueLength: 10,
    );

    super.initState();
  }

  void initConnection() async {
    ros.connect();
    await chatter.subscribe();

    await counter.advertise();
    await cmd_vel.advertise();
    setState(() {});
  }

  void publishCounter() async {
    var msg = {'data': 'hello'};
    await counter.publish(msg);
    print('done publihsed');
  }

  void publishCmd() async {
    var linear = {'x': 34.0, 'y': 0.0, 'z': 0.0};
    var angular = {'x': 0.0, 'y': 0.0, 'z': 32.0};
    var twist = {'linear': linear, 'angular': angular};
    await cmd_vel.publish(twist);
    print('cmd published');
  }

  void destroyConnection() async {
    await chatter.unsubscribe();

    await counter.unadvertise();
    await ros.close();
    setState(() {});
  }

  @override
  Widget build(BuildContext context) {
    return StreamBuilder<Object>(
      stream: ros.statusStream,
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
                stream: chatter.subscription,
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
