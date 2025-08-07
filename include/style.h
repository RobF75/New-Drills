#ifndef STYLE_H
#define STYLE_H

const char DRILL_CONTROL_STYLE[] = R"rawliteral(
body {
  font-family: Arial, sans-serif;
  background: #f4f4f4;
  color: #222;
  margin: 0;
  padding: 0;
}
.container {
  max-width: 400px;
  margin: 40px auto;
  background: #fff;
  border-radius: 8px;
  box-shadow: 0 2px 8px rgba(0,0,0,0.1);
  padding: 32px 24px;
}
h2 {
  text-align: center;
  margin-bottom: 24px;
}
form {
  margin-bottom: 18px;
}
input[type='number'] {
  width: 80px;
  padding: 4px;
  margin: 6px 0;
  border-radius: 4px;
  border: 1px solid #ccc;
}
input[type='submit'] {
  background: #007bff;
  color: #fff;
  border: none;
  border-radius: 4px;
  padding: 8px 16px;
  cursor: pointer;
  font-size: 1em;
  margin-top: 8px;
}
input[type='submit']:hover {
  background: #0056b3;
}
)rawliteral";

#endif // STYLE_H
