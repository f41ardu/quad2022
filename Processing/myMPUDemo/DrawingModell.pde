// quad2022 edition
void drawCylinder(float topRadius, float bottomRadius, float tall, int sides) {
    float angle = 0;
    float angleIncrement = TWO_PI / sides;
    beginShape(QUAD_STRIP);
    for (int i = 0; i < sides + 1; ++i) {
        vertex(topRadius*cos(angle), 0, topRadius*sin(angle));
        vertex(bottomRadius*cos(angle), tall, bottomRadius*sin(angle));
        angle += angleIncrement;
    }
    endShape();
    
    // If it is not a cone, draw the circular top cap
    if (topRadius != 0) {
        angle = 0;
        beginShape(TRIANGLE_FAN);
        
        // Center point
        vertex(0, 0, 0);
        for (int i = 0; i < sides + 1; i++) {
            vertex(topRadius * cos(angle), 0, topRadius * sin(angle));
            angle += angleIncrement;
        }
        endShape();
    }
  
    // If it is not a cone, draw the circular bottom cap
    if (bottomRadius != 0) {
        angle = 0;
        beginShape(TRIANGLE_FAN);
    
        // Center point
        vertex(0, tall, 0);
        for (int i = 0; i < sides + 1; i++) {
            vertex(bottomRadius * cos(angle), tall, bottomRadius * sin(angle));
            angle += angleIncrement;
        }
        endShape();
    }
}

void drawModell() {
 rotateY(PI/2);
  // draw main body in red
    fill(255, 0, 0, 200);
    box(10, 10, 200);
    
    // draw front-facing tip in blue
    fill(0, 0, 255, 200);
    pushMatrix();
    translate(0, 0, -120);
    rotateX(PI/2);
    drawCylinder(0, 20, 20, 8);
    popMatrix();
    
    // draw wings and tail fin in green
    fill(0, 255, 0, 200);
    beginShape(TRIANGLES);
    vertex(-100,  2, 30); vertex(0,  2, -80); vertex(100,  2, 30);  // wing top layer
    vertex(-100, -2, 30); vertex(0, -2, -80); vertex(100, -2, 30);  // wing bottom layer
    vertex(-2, 0, 98); vertex(-2, -30, 98); vertex(-2, 0, 70);  // tail left layer
    vertex( 2, 0, 98); vertex( 2, -30, 98); vertex( 2, 0, 70);  // tail right layer
    endShape();
    beginShape(QUADS);
    vertex(-100, 2, 30); vertex(-100, -2, 30); vertex(  0, -2, -80); vertex(  0, 2, -80);
    vertex( 100, 2, 30); vertex( 100, -2, 30); vertex(  0, -2, -80); vertex(  0, 2, -80);
    vertex(-100, 2, 30); vertex(-100, -2, 30); vertex(100, -2,  30); vertex(100, 2,  30);
    vertex(-2,   0, 98); vertex(2,   0, 98); vertex(2, -30, 98); vertex(-2, -30, 98);
    vertex(-2,   0, 98); vertex(2,   0, 98); vertex(2,   0, 70); vertex(-2,   0, 70);
    vertex(-2, -30, 98); vertex(2, -30, 98); vertex(2,   0, 70); vertex(-2,   0, 70);
    endShape();
    
}

void drawsimple() {
      //     fill(255,255,0);
//     box(100,100,10);
   noFill(); 
   stroke(255);
//        triangle(100, 0, 0, 0, 100, 0);
//        sphere(100);
        fill(0,255,0);
   stroke(0,0,255);
   box(10,100,100);
   box(75,5,5);
   box(300,0,0);
   box(0,300,0);
   box(0,0,300);
   translate(-60,0,0);
   sphere(10);
   translate(120,0,0);
   stroke(255,0,0);
   sphere(10);
  // triangle(0, 0, 58, 20, 86, 75);
}

void drawbox() {
     noFill();
     stroke(255,255,255);
     box(1000);
     drawAxes();
}

void drawAxes() {
  drawboard();
  stroke(255, 0, 0);
  fill(255, 0, 0);
  //line(-300, 0, 0, 300, 0, 0);
  box(400,3,3);
  stroke(0, 255, 0);
  fill(0, 255, 0);
  //line(0, -300, 0, 0, 300, 0);
  box(3,400,3);
  stroke(0, 0, 255);
  fill(0, 0, 255);
  //line(0, 0, -300, 0, 0, 300);
  box(3,3,400);
  fill(37,83,206);
  box(200,8,100);
  fill(255,255,255);
  text("X", -210, 0, 0);
  text("X: "  + nf(degrees(value[0]),2,1)  + "°" , 210, 0, 0);
  text("Z: " + nf(degrees(value[2]),2,1)  + "°" , 0, 210, 0);
  text("Z", 0, -210, 0);
  text("Y: "  + nf(degrees(value[1]),2,1) + "°" , 0, 0, 210);
  text("Y", 0, 0, -210);
}

void drawboard() {
  pushMatrix();
  translate(0,-5,0);
  rotateX(HALF_PI);
  image(img, 0, 0);
  popMatrix();
  pushMatrix();
  translate(0,5,0);
  rotateX(3*HALF_PI);
  image(img2, 0, 0);
  popMatrix();
  
  
}
