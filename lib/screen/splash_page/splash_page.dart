import 'dart:async';

import 'package:flutter/material.dart';
import 'package:my_app/screen/home_page/home_page.dart';

class SplashPage extends StatefulWidget {
  @override
  _SplashPageState createState() => _SplashPageState();
}

class _SplashPageState extends State<SplashPage> {
  @override
  void initState() {
    Timer(
        Duration(
          seconds: 5,
        ),
        () => {
              Navigator.pushReplacement(
                  context, MaterialPageRoute(builder: (context) => HomePage()))
            });
    super.initState();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.green[600],
      body: Container(
        child: Center(
          child: Text(
            "Splash Screen",
            style: TextStyle(
              color: Colors.white,
              fontSize: 24,
            ),
          ),
        ),
      ),
    );
  }
}
