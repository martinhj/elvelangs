// Kinect Physics Example by Amnon Owed (15/09/12)

//edited by Arindam Sen
 
// import libraries
import processing.opengl.*; // opengl
import SimpleOpenNI.*; // kinect
import blobDetection.*; // blobs
import toxi.geom.*; // toxiclibs shapes and vectors
import toxi.processing.*; // toxiclibs display
import shiffman.box2d.*; // shiffman's jbox2d helper library
import org.jbox2d.collision.shapes.*; // jbox2d
import org.jbox2d.dynamics.joints.*;
import org.jbox2d.common.*; // jbox2d
import org.jbox2d.dynamics.*; // jbox2d

// declare SimpleOpenNI object
SimpleOpenNI context;
// declare BlobDetection object
BlobDetection theBlobDetection;
// ToxiclibsSupport for displaying polygons
ToxiclibsSupport gfx;
// declare custom PolygonBlob object (see class for more info)
PolygonBlob poly;
PolygonBlob poly2;
PolygonBlob [] polys = new PolygonBlob [100];

int lastBlob = 0;
 
// PImage to hold incoming imagery and smaller one for blob detection
PImage blobs;
// the kinect's dimensions to be used later on for calculations
int kinectWidth = 640;
int kinectHeight = 480;
PImage cam = createImage(640, 480, RGB);

// to center and rescale from 640x480 to higher custom resolutions
float reScale;
 
// background and blob color
color bgColor, blobColor;
color black = color(0,0,0);


// depthMaps used for background subtraction.
int [] depthMap, bgDepthMap;



// three color palettes (artifact from me storingmany interesting color palettes as strings in an external data file ;-)
String[] palettes = {
  "-1117720,-13683658,-8410437,-9998215,-1849945,-5517090,-4250587,-14178341,-5804972,-3498634", 
  "-67879,-9633503,-8858441,-144382,-4996094,-16604779,-588031", 
  "-1978728,-724510,-15131349,-13932461,-4741770,-9232823,-3195858,-8989771,-2850983,-10314372"
};
color[] colorPalette;
 
// the main PBox2D object in which all the physics-based stuff is happening
Box2DProcessing box2d;
// list to hold all the custom shapes (circles, polygons)
ArrayList<CustomShape> polygons = new ArrayList<CustomShape>();
 
void setup() {
  println("SET UP");
  // it's possible to customize this, for example 1920x1080
  size(1024, 768);
  context = new SimpleOpenNI(this);
  //smooth(8);

  // initialize SimpleOpenNI object
  if (!context.enableDepth() || !context.enableUser()) { 
    // if context.enableScene() returns false
    // then the Kinect is not working correctly
    // make sure the green light is blinking
    println("Kinect not connected!"); 
    exit();
  } else {
    // mirror the image to be more intuitive
    context.setMirror(true);
    // calculate the reScale value
    // currently it's rescaled to fill the complete width (cuts of top-bottom)
    // it's also possible to fill the complete height (leaves empty sides)
    reScale = (float) width / kinectWidth;
    // create a smaller blob image for speed and efficiency
    blobs = createImage(kinectWidth/3, kinectHeight/3, RGB);
    // initialize blob detection object to the blob image dimensions
    theBlobDetection = new BlobDetection(blobs.width, blobs.height);
    theBlobDetection.setThreshold(0.3);
    // initialize ToxiclibsSupport object
    gfx = new ToxiclibsSupport(this);
    // setup box2d, create world, set gravity
    box2d = new Box2DProcessing(this);
    box2d.createWorld();


    /*
     *
     * Gravity direction!!
     *
     */
    box2d.setGravity(20, 0);
    //box2d.setGravity(0, -40);
    // set random colors (background, blob)
    setRandomColors(1);
    
    float gap = kinectWidth / 21;
    for (int i=0; i<20; i++)
    {
      drawString(gap * (i+1), 2, 10);
    }
  }
  saveBackground();
}

void drawString(float x, float size, int cards) {
  
  float gap = kinectHeight/cards;
  // anchor card
  CustomShape s1 = new CustomShape(x, -40, size, BodyType.DYNAMIC);
  polygons.add(s1);
  
  CustomShape last_shape = s1;
  CustomShape next_shape;
  for (int i=0; i<cards; i++)
  {
    float y = -20 + gap * (i+1);
    next_shape = new CustomShape(x, -20 + gap * (i+1), size, BodyType.DYNAMIC);
    DistanceJointDef jd = new DistanceJointDef();

    Vec2 c1 = last_shape.body.getWorldCenter();
    Vec2 c2 = next_shape.body.getWorldCenter();
  // offset the anchors so the cards hang vertically
    c1.y = c1.y + size / 5;
    c2.y = c2.y - size / 5;
    jd.initialize(last_shape.body, next_shape.body, c1, c2);
    jd.length = box2d.scalarPixelsToWorld(gap - 1);
    box2d.createJoint(jd);
    polygons.add(next_shape);
    last_shape = next_shape;
  }
}
 
void draw() {
  background(bgColor);
  background(0);
  // update the SimpleOpenNI object
  context.update();

  depthMap = context.depthMap();
  cam = context.depthImage();
  
  ///*
  cam.loadPixels();
  for (int i = 0; i < cam.pixels.length; i++) {
    if (depthMap[i] > bgDepthMap[i] - 100) {
      cam.pixels[i] = black;
    }
  }
  cam.updatePixels();
  //*/


  // copy the image into the smaller blob image
  /*
   *
   * set which part of the kinect picture to crop out
   *
   */
  blobs.copy(cam, 133, 0, 378, 282, 0, 0, blobs.width, blobs.height);
  //blobs.copy(cam, 0, 0, cam.width, cam.height, 0, 0, blobs.width, blobs.height);
  // blur the blob image
  blobs.filter(BLUR, 1);
  // detect the blobs
  theBlobDetection.computeBlobs(blobs.pixels);
  // initialize a new polygon
  for (int i = 0; i < polys.length; i++) {
    polys[i] = new PolygonBlob();
  }
  poly = new PolygonBlob();
  // create the polygon from the blobs (custom functionality, see class)
  poly.createPolygon(0);
  for (int i = 0; i < polys.length; i++) {
    polys[i].createPolygon(0);
  }
  // create the box2d body from the polygon
  poly.createBody(true);
  for (int i = 0; i < polys.length; i++) {
    polys[i].createBody(true);
  }
  // update and draw everything (see method)
  updateAndDrawBox2D();
  // destroy the person's body (important!)
  poly.destroyBody();
  for (int i = 0; i < polys.length; i++) {
    polys[i].destroyBody();
  }
  // set the colors randomly every 240th frame
  //setRandomColors(240);
  //image(blobs, 0, 0, width, height);
  printFrameRate();
  drawBlobsAndEdges(false,true);
}
 
void updateAndDrawBox2D() {
  // if frameRate is sufficient, add a polygon and a circle with a random radius

  

  /*
   *
   * Set where the shapes are comeing in the costumshape constructor.
   *
   */
  if (frameRate > 18 && frameCount % 13 == 0) {
    CustomShape shape1 = new CustomShape(-50, kinectHeight/2, -1,BodyType.DYNAMIC) ;
    CustomShape shape2 = new CustomShape(-50, kinectHeight/2, random(5, 35),BodyType.DYNAMIC);
     //CustomShape shape1 = new CustomShape(kinectWidth/2, -50, -1,BodyType.DYNAMIC) ;
     //CustomShape shape2 = new CustomShape(kinectWidth/2, -50, random(2.5, 20),BodyType.DYNAMIC);
    polygons.add(shape1);
    polygons.add(shape2);
  }
  // take one step in the box2d physics world
  box2d.step();
 
  // center and reScale from Kinect to custom dimensions
  translate(0, (height-kinectHeight*reScale)/2);
  scale(reScale);
 
 
  /*
   * 
   * // display the person's polygon
   *
   */  
  noStroke();
  fill(color(255,0,0));
  gfx.polygon2D(poly);
  fill(color(0,255,0));
  for (int i = 0; i < polys.length; i++) {
    gfx.polygon2D(polys[i]);
  }
 
  // display all the shapes (circles, polygons)
  // go backwards to allow removal of shapes
  for (int i=polygons.size()-1; i>=0; i--) {
    CustomShape cs = polygons.get(i);
    // if the shape is off-screen remove it (see class for more info)
    
    
    if (cs.done()) {
      polygons.remove(i);
    // otherwise update (keep shape outside person) and display (circle or polygon)
    } else {
      cs.update();
      cs.display();
    }
  }
}
 
// sets the colors every nth frame
void setRandomColors(int nthFrame) {
  if (frameCount % nthFrame == 0) {
    // turn a palette into a series of strings
    String[] paletteStrings = split(palettes[int(random(palettes.length))], ",");
    // turn strings into colors
    colorPalette = new color[paletteStrings.length];
    for (int i=0; i<paletteStrings.length; i++) {
      colorPalette[i] = int(paletteStrings[i]);
    }
    // set background color to first color from palette
    bgColor = colorPalette[0];
    // set blob color to second color from palette
    blobColor = colorPalette[1];
    // set all shape colors randomly
    for (CustomShape cs: polygons) { cs.col = getRandomColor(); }
  }
}
 
// returns a random color from the palette (excluding first aka background color)
color getRandomColor() {
  return color(255,255,255);
  //return colorPalette[int(random(1, colorPalette.length))];
}


void saveBackground() {
  bgDepthMap = new int [context.depthMap().length];
  int [] bgTemp;
  for (int i = 0; i < bgDepthMap.length; i++) {
    bgDepthMap[i] = 10000;
  }
  for (int i = 0; i < 90; i++) {
    ellipse(0,0,height,width);
    context.update();
    bgTemp = context.depthMap();

    // simplefy the if statements below
    /*
    for (int j = -1; j <= 1; j++) {
    }
    */

    for (int j = 0; j < bgTemp.length; j++) {
      if (bgTemp[j] < bgDepthMap[j] && bgTemp[j] > 1) {
        bgDepthMap[j] = bgTemp[j];
        if (j-1 >= 0 && bgDepthMap[j - 1] > bgTemp[j])
          bgDepthMap[j - 1] = bgTemp[j];
        if (j + 1 < 640*480 && bgDepthMap[j + 1] > bgTemp[j])
          bgDepthMap[j + 1] = bgTemp[j];
        if (j-640 >= 0 && bgDepthMap[j-640] > bgTemp[j])
          bgDepthMap[j-640] = bgTemp[j];
        if (j-640-1 >= 0 && bgDepthMap[j-640-1] > bgTemp[j])
          bgDepthMap[j-640-1] = bgTemp[j];
        if (j-640+1 >= 0 && bgDepthMap[j-640+1] > bgTemp[j])
          bgDepthMap[j-640+1] = bgTemp[j];
        if (j+640 < 640*480 && bgDepthMap[j+640] > bgTemp[j])
          bgDepthMap[j+640] = bgTemp[j];
        if (j+640-1 < 640*480 && bgDepthMap[j+640-1] > bgTemp[j])
          bgDepthMap[j+640-1] = bgTemp[j];
        if (j+640+1 < 640*480 && bgDepthMap[j+640+1] > bgTemp[j])
          bgDepthMap[j+640+1] = bgTemp[j];
      }
    }
  }
}

void printFrameRate() {
  /*fill(255);
  stroke(255);
  rect(50, 50, 80, 25, 5);
  fill(0);
  text(frameRate, 59, 67);
  */
  if (frameCount % 11 == 0) println(frameRate);
}



void keyPressed() {
  switch (key) {
    case 'b': println("saving background.");
              saveBackground();
              println("saved background.");
              break;
  }
}



void drawBlobsAndEdges(boolean drawBlobs, boolean drawEdges) {
  noFill();
  Blob b;
  EdgeVertex eA, eB;
  int drawWidth = width / 2;
  int drawHeight = kinectHeight;
  for (int n=0 ; n < theBlobDetection.getBlobNb() ; n++)
  {
    b=theBlobDetection.getBlob(n);
    if (b!=null)
    {
      // Edges
      if (drawEdges)
      {
        strokeWeight(2);
        stroke(0, 255, 0);
        for (int m=0;m<b.getEdgeNb();m++)
        {
          eA = b.getEdgeVertexA(m);
          eB = b.getEdgeVertexB(m);
          if (eA !=null && eB !=null)
            line(
                eA.x*drawWidth, eA.y*drawHeight, 
                eB.x*drawWidth, eB.y*drawHeight
                );
        }
      }

      // Blobs
      if (drawBlobs)
      {
        strokeWeight(1);
        if (b.w*b.h > 0.02f) stroke(255, 0, 0);
        if (b.w*b.h <= 0.02f) stroke(0,0,255);
        if (b.w*b.h > 0.02f) rect(
            b.xMin*drawWidth, b.yMin*drawHeight, 
            b.w*drawWidth, b.h*drawHeight
            );
        /*ellipse(b.xMin*width+b.w*width/2, b.yMin*height+b.h*height/2, 5,5);*/

      }
    }
  }
}
