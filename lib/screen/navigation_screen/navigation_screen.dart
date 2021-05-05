import 'package:my_app/screen/camera_page/camera_page.dart';
import 'package:my_app/screen/joystick_page/joystick_page.dart';
import 'package:fancy_bottom_navigation/fancy_bottom_navigation.dart';
import 'package:flutter/material.dart';
import 'package:my_app/screen/settings_page/setting_page.dart';

class MyHomePage extends StatefulWidget {
  MyHomePage({Key key, this.ip}) : super(key: key);
  final String ip;
  @override
  _MyHomePageState createState() => _MyHomePageState();
}

class _MyHomePageState extends State<MyHomePage> {
  int currentPage = 1;
  List<Widget> pages;
  @override
  void initState() {
    super.initState();
    String ipAddr = widget.ip;
    String webIp = "http://$ipAddr:8081";
    pages = [
      MyWebView(title: 'Map', selectedUrl: 'http://' + ipAddr + ':8000/'),
      JoyStickPage(
        ip: widget.ip,
      ),
      MyWebView(title: 'RViz', selectedUrl: webIp),
      SettingsScreen(),
    ];
  }

  GlobalKey bottomNavigationKey = GlobalKey();

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Container(
        decoration: BoxDecoration(color: Colors.white),
        child: IndexedStack(
          index: currentPage,
          children: pages,
          /*child: Center(
            child: _getPage(currentPage),
          ),
          */
        ),
      ),
      bottomNavigationBar: FancyBottomNavigation(
        tabs: [
          TabData(
            iconData: Icons.map,
            title: "Map",
          ),
          TabData(
            iconData: Icons.gamepad,
            title: "Control",
          ),
          TabData(iconData: Icons.radio, title: "RViz"),
          TabData(iconData: Icons.settings, title: "Setting"),
        ],
        initialSelection: 1,
        key: bottomNavigationKey,
        onTabChangedListener: (position) {
          setState(() {
            currentPage = position;
          });
        },
      ),
    );
  }

  _getPage(int page) {
    switch (page) {
      case 0:
        return MyWebView(
            title: 'Navigation', selectedUrl: 'http://192.168.1.4:8000/');
      case 1:
        return JoyStickPage();
      case 2:
        return SettingsScreen();

      default:
        return SettingsScreen();
    }
  }
}
