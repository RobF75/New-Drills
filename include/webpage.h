#ifndef WEBPAGE_H
#define WEBPAGE_H

// Include style.h for DRILL_CONTROL_STYLE
#include "style.h"

const char DRILL_CONTROL_HTML[] = R"rawliteral(
<html>
<head>
  <title>Drill Control</title>
  <style>
    %STYLE%
  </style>
</head>
<body>
  <div class='container'>
    <h2>Drill Control Panel</h2>
    <div style='text-align:center; margin-bottom:18px;'>
      <span style='font-size:1em; color:#555;'>Firmware Version: <b>%VERSION%</b></span>
    </div>
    <form action='/update' method='POST'>
      Z Axis Move Down Distance: <input type='number' name='z' value='%ZAXIS%'><br>
      X Axis Move Distance: <input type='number' name='x' value='%XAXIS%'><br>
      <input type='submit' value='Update Values'>
    </form>
    <form action='/start' method='POST' style='margin-top:20px;'>
      <input type='submit' value='Start Drilling'>
    </form>
    <form action='/ota' method='POST' style='margin-top:20px;'>
      <input type='submit' value='Update Firmware'>
    </form>
    <div style='text-align:center; margin-bottom:12px;'>
      <span id="startbtn-status">%STARTBTN%</span>
    </div>
    <div>Pins Status:
      <ul>
        <li>Z Axis Step Pin: %ZAXIS_STEP%</li>
        <li>Z Axis Dir Pin: %ZAXIS_DIR%</li>
        <li>Z Axis Home Pin: %ZAXIS_HOME%</li>
        <li>Drills Pin: %DRILLS%</li>
        <li>X Axis Step Pin: %XAXIS_STEP%</li>
        <li>X Axis Dir Pin: %XAXIS_DIR%</li>
        <li>X Axis Home Pin: %XAXIS_HOME%</li>
        <li>Start Button Pin: %START_BUTTON%</li>
      </ul>
    </div>
    <script>
    function updateStatus() {
      fetch('/status').then(r => r.text()).then(txt => {
        document.getElementById('startbtn-status').innerHTML = txt;
      });
    }
    setInterval(updateStatus, 1000);
    </script>
  </div>
</body>
</html>
)rawliteral";

#endif // WEBPAGE_H
