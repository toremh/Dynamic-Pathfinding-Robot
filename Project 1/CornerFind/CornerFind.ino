void setup() {
  // put your setup code here, to run once:




  // rotate and find closest wall
  //awhrdxcfads

  void goCorner() {

    closeAngle = 0;
    angGyro = 0;
    closeWall = 100;
    cw();
    while (angGyro < 359) {
      if (distUltra < closeWall) {
        closeAngle = angGyro;
      }
    }
    stop();

    //turn towards closest wall
    cw();
    while ( angGyro < closeangle) {
      delay(10);
    }
    stop();

    // go towards closest wall
    forwards();
    while (distUltra < 15) {
      delay(10);
    }
    stop();

    // turn right 90
    angGyro = 0;
    cw();
    while (angGyro < 90) {
      delay(10);
    }
    stop();

    // go forwards until equidistant Replace with goStraight function
    forwards();
    while (distUltra > 15) {
      delay(10);
    }
    stop();

    // turn right 180, point towards furthest wall
    angGyro = 0;
    cw;
    while (angGyro < 90) {
      delay(10);
    }
    stop();
    Lcorner = distUltra;

    while ((angGyro - startOffset) < 180) {
      cw();
    }
    stop();
    Rcorner = distUltra;

    if (Lcorner > Rcorner) {
      angGyro = 0;
      while (angGyro > 90) {
        ccw();
      }
      stop();
    }
  }
}


void loop() {
  // put your main code here, to run repeatedly:
  //
}
