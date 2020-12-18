import 'package:flutter/material.dart';
import 'package:my_app/repos/models/publisher.dart';
import 'package:my_app/screen/button_page/button_page.dart';
import 'package:my_app/screen/camera_page/camera_page.dart';
import 'package:my_app/screen/joystick_page/joystick_page.dart';
import 'package:my_app/screen/navigation_screen/navigation_screen.dart';
import 'package:my_app/screen/settings_page/setting_page.dart';

class HomePage extends StatefulWidget {
  @override
  _HomePageState createState() => _HomePageState();
}

class _HomePageState extends State<HomePage> {
  @override
  Widget build(BuildContext context) {
    return Scaffold(
        appBar: AppBar(
          title: Text("ROBO JOY"),
          centerTitle: true,
        ),
        body: MyHomePage() //SettingsScreen(), //MyWebView(
        //title: 'Navigation', selectedUrl: 'http://192.168.1.4:8000/'),
        //JoyStickPage(),
        );
  }
}
