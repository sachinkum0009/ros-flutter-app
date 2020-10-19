import 'package:flutter/material.dart';
import 'package:control_pad/control_pad.dart';

class JoyStickPage extends StatefulWidget {
  @override
  _JoyStickPageState createState() => _JoyStickPageState();
}

class _JoyStickPageState extends State<JoyStickPage> {
  double _x = 50;
  double _y = 50;

  void _move(double _degrees, double _distance) {
    print(
        'Degree:' + _degrees.toString() + ' Distance:' + _distance.toString());
    //setState(() {
    _x += 1;
    _y += 1;
    //});
  }

  @override
  void initState() {
    print("joystick page is ready");
    super.initState();
  }

  @override
  Widget build(BuildContext context) {
    return Container(
        color: Colors.white,
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            JoystickView(
              onDirectionChanged: (_degrees, _distance) =>
                  _move(_degrees, _distance),
            ),
          ],
        ));
  }
}
