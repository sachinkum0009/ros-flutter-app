import 'package:flutter/material.dart';
import 'package:settings_ui/settings_ui.dart';

import 'languages_screen.dart';

class SettingsScreen extends StatefulWidget {
  @override
  _SettingsScreenState createState() => _SettingsScreenState();
}

class _SettingsScreenState extends State<SettingsScreen> {
  // _showDialog() async {
  //   await showDialog<String>(
  //     context: context,
  //     child: AlertDialog(
  //       contentPadding: const EdgeInsets.all(16.0),
  //       content: Row(
  //         children: <Widget>[
  //           Expanded(
  //             child: TextField(
  //               autofocus: true,
  //               decoration: InputDecoration(
  //                   labelText: 'ROS MASTER URI', hintText: 'eg. 192.168.1.4'),
  //             ),
  //           )
  //         ],
  //       ),
  //       actions: <Widget>[
  //         FlatButton(
  //             child: const Text('CANCEL'),
  //             onPressed: () {
  //               Navigator.pop(context);
  //             }),
  //         FlatButton(
  //             child: const Text('OPEN'),
  //             onPressed: () {
  //               Navigator.pop(context);
  //             })
  //       ],
  //     ),
  //   );
  // }

  // show topic dialog
  // _showTopicDialog() async {
  //   await showDialog<String>(
  //     context: context,
  //     child: AlertDialog(
  //       contentPadding: const EdgeInsets.all(16.0),
  //       content: Row(
  //         children: <Widget>[
  //           Expanded(
  //             child: TextField(
  //               autofocus: true,
  //               decoration: InputDecoration(
  //                   labelText: 'Topic Name', hintText: 'eg. /cmd_vel'),
  //             ),
  //           )
  //         ],
  //       ),
  //       actions: <Widget>[
  //         FlatButton(
  //             child: const Text('CANCEL'),
  //             onPressed: () {
  //               Navigator.pop(context);
  //             }),
  //         FlatButton(
  //             child: const Text('OPEN'),
  //             onPressed: () {
  //               Navigator.pop(context);
  //             })
  //       ],
  //     ),
  //   );
  // }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      //appBar: AppBar(title: Text('Settings UI')),
      body: SettingsList(
        // backgroundColor: Colors.orange,

        sections: [
          SettingsSection(
            title: 'ROS SETTINGS',
            tiles: [
              SettingsTile(
                title: 'ROS MASTER URI',
                leading: Icon(Icons.cloud_queue),
                onTap: () {
                  // show text field
                  // _showDialog();
                  print('some tapped');
                },
              ),
              SettingsTile(
                title: 'TOPIC NAME',
                leading: Icon(Icons.info_outline_rounded),
                onTap: () {
                  // show topic field
                  // _showTopicDialog();
                  print('topic dialog');
                },
              ),
            ],
          ),
        ],
      ),
    );
  }
}
