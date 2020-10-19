import 'package:flutter/material.dart';
import 'package:my_app/screen/splash_page/splash_page.dart';

class MyApp extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: "My App",
      home: SplashPage(),
    );
  }
}
