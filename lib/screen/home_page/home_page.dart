import 'package:flutter/material.dart';
import 'package:my_app/repos/models/publisher.dart';
import 'package:my_app/screen/button_page/button_page.dart';
import 'package:my_app/screen/camera_page/camera_page.dart';
import 'package:my_app/screen/fancy_navigation_bar/fancy_navigation_bar.dart';
import 'package:my_app/screen/joystick_page/joystick_page.dart';
import 'package:my_app/screen/navigation_screen/navigation_screen.dart';
import 'package:my_app/screen/settings_page/setting_page.dart';

class HomePage extends StatefulWidget {
  @override
  _HomePageState createState() => _HomePageState();
}

class _HomePageState extends State<HomePage> {
  final myController = TextEditingController();
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text("ROBO JOY"),
        centerTitle: true,
      ),
      body: Center(
        child: Container(
          width: 250,
          child: Center(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.center,
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                Text(
                  'Enter Ip Address',
                  style: TextStyle(
                    fontSize: 28,
                  ),
                ),
                SizedBox(
                  height: 40,
                ),
                TextField(
                  controller: myController,
                  decoration: InputDecoration(
                    hintText: "Enter Ip Address",
                    enabledBorder: OutlineInputBorder(
                      borderSide: BorderSide(color: Colors.blueAccent),
                    ),
                  ),
                ),
              ],
            ),
          ),
        ),
      ),
      //MyHomePage() // FancyNavigationBar() //SettingsScreen(), //MyWebView(
      //title: 'Navigation', selectedUrl: 'http://192.168.1.4:8000/'),
      //JoyStickPage(),
      floatingActionButton: FloatingActionButton(
        child: Icon(Icons.play_arrow),
        onPressed: () {
          print(myController.text);
          Navigator.pushReplacement(
              context,
              MaterialPageRoute(
                  builder: (context) => MyHomePage(
                        ip: myController.text,
                      )));
        },
      ),
    );
  }
}
