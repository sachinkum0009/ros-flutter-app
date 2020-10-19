import 'package:flutter/material.dart';

class ButtonPage extends StatefulWidget {
  @override
  _ButtonPageState createState() => _ButtonPageState();
}

class _ButtonPageState extends State<ButtonPage> {
  @override
  Widget build(BuildContext context) {
    return Container(
      child: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          crossAxisAlignment: CrossAxisAlignment.center,
          children: [
            RaisedButton(
              onPressed: () {
                print("button 1");
              },
              child: Text("1"),
            ),
            RaisedButton(
              onPressed: () {
                print("button 2");
              },
              child: Text("2"),
            ),
            RaisedButton(
              onPressed: () {
                print("button 3");
              },
              child: Text("3"),
            ),
            RaisedButton(
              onPressed: () {
                print("button 4");
              },
              child: Text("4"),
            ),
          ],
        ),
      ),
    );
  }
}
