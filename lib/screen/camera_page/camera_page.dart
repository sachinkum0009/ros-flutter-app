import 'dart:async';
import 'package:flutter/material.dart';

import 'package:webview_flutter/webview_flutter.dart';
import 'package:flutter_webview_plugin/flutter_webview_plugin.dart';

class MyWebView extends StatelessWidget {
  final String title;
  final String selectedUrl;

  Completer<WebViewController> _controller = Completer<WebViewController>();

  MyWebView({
    @required this.title,
    @required this.selectedUrl,
  });

  @override
  Widget build(BuildContext context) {
    return Stack(
      children: <Widget>[
        WebView(
          initialUrl: selectedUrl,
          javascriptMode: JavascriptMode.unrestricted,
          onWebViewCreated: (WebViewController webViewController) {
            _controller.complete(webViewController);
          },
        ),
      ],
    );

    // return WebviewScaffold(
    //   url: selectedUrl,
    //   withJavascript: true,
    //   withZoom: true,
    // );
  }
}
