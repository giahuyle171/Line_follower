String data_recv;

void serial_update() {
  while (1) {
    if (Serial.available()) {
      int16_t c = Serial.read();
      if (c!='\n' && c!=' ')
        data_recv += (char)c;
      else break;
    }
    else return;
  }
  String id = "";
  int16_t val = 0;
  int i = 0;
  while (i<data_recv.length() && (data_recv[i]<48 || data_recv[i]>57)){
    id += data_recv[i];
    i++;
  }
  for (; i<data_recv.length(); i++){
    val = val*10+(data_recv[i]-48);
  }
  data_recv = "";
  set_data(val, id);
}
void set_data(int16_t val, String id) {
  if (id == "p") {
    Kp = val/1000.0f;
  }
  if (id == "i") {
    Ki = val/1000.0f;
  }
  if (id == "d") {
    Kd = val/1000.0f;
  }
  if (id == "c") {
    calibrate(val);
  }
  if (id == "r") {
    run = !run;
    reset();
  }
}

void telemetryTask() {
  uint32_t time = millis();
  if (time-t_telemetryTask >= 50){
    t_telemetryTask = time;
    serial_update();
    for (int i = 0; i < 9; i++) {
      print(sensor_digital[i]);
    }
    // for (int i = 0; i < 9; i++) {
    //   print(sensor_raw[i]);
    // }
    print(calculatePosition());
    print(run);
    print(digitalRead(PA8));
    Serial.println("");
  }
}