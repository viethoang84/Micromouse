//Đo ngưỡng bằng cách cho xe lệch sang bên đối diện tối đa

bool wallLeft() {
  if(analogRead(IRR1) >= 600) {
    // strip.fill(strip.Color(128,0,0,0));
    // strip.show();
    return 1;
  }else{
    return 0;
  }
}

bool wallFront() {
  if(analogRead(IRR3) >= 500) {
    return 1;
  }else{
    return 0;
  }
}

bool wallRight() {
  if(analogRead(IRR4) >= 600) {
    // strip.fill(strip.Color(0,128,0,0));
    // strip.show();
    return 1;
  }else{
    return 0;
  }
}