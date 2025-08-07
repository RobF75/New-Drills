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
  </div>
</body>
</html>
)rawliteral";

#endif // WEBPAGE_H
