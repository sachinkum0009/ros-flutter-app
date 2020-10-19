import 'package:flutter/material.dart';
import 'package:my_app/repos/models/publisher.dart';
import 'package:my_app/screen/button_page/button_page.dart';
import 'package:my_app/screen/joystick_page/joystick_page.dart';

class HomePage extends StatefulWidget {
  @override
  _HomePageState createState() => _HomePageState();
}

class _HomePageState extends State<HomePage> {
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text("Teleop Joy"),
        centerTitle: true,
      ),
      body: JoyStickPage(),
    );
  }
}
