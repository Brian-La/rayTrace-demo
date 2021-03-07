#pragma once

#include "ofMain.h"


//STRUCTURES FOR SPHERE/TRIANGLE============================
//SPHERE----------------------------------------------------
struct SphereTemp {
    float center[3];        //center coordinates
    float radius;            //radius of sphere
    float ambient_color[3];        //ambient col
    float diffuse_color[3];        //diffuse col
    float specular_color[3];    //specular col
    float shininess;            //shininess var
    float reflectivity;            //reflectivity var
};

//TRIANGLE--------------------------------------------------
struct Triangle {
    float v0[3];            //1st vertice
    float v1[3];            //2nd
    float v2[3];            //3rd
    float ambient_color[3];        //ambient col
    float diffuse_color[3];        //diffuse col
    float specular_color[3];    //specular col
    float shininess;            //shininess var
    float reflectivity;            //reflectivity var
};


//RAY_MARCHING===============================================
//RAY: defines ray w/ origin and distance
class Ray {
public:
    //Ray constructor(s)
    Ray() {}
    Ray(glm::vec3 origin, glm::vec3 direction) {
        p = origin;
        d = direction;
    }
    
    glm::vec3 p;        //origin of ray
    glm::vec3 d;    //direction of ray
};

//Shape: Base class FOR ALL PRIMITIVES (to be put into dynamic vector)
class Shape {
public:
    Shape() {}     //constructor
    
    virtual float sdf(const glm::vec3 &p) { return 0.0; }
    
    //Attributes---------------------------------------------
    glm::vec3 position;     //position of shape
    ofColor color;          //color of shape
    
};

//Sphere: sphere w/ inherited position/color and radius
class Sphere : public Shape {
public:
    //constructor(s)-----------------------------------------
    Sphere() {
        position = glm::vec3(0, 5, 0);      //center of sphere @ origin
        radius = 5.0;
        color = ofColor::red;
    }
    Sphere(glm::vec3 p, float r, ofColor c);
    //end constructor(s)-------------------------------------
    
    //-------------------------------------------------------
    float sdf (const glm::vec3 &p) { return (p - position).length() - radius; };        //in-line SDF function replicated from CS116B.Lec6 slides
    float radius;       //radius of circle
};

//Plane: plane w/ inherited position/color

class ofApp : public ofBaseApp{

public:
	void setup();
	void update();
	void draw();

	void keyPressed(int key);
    
    
    ofEasyCam marchCam;
    
    //RAY_MARCH COMPONENTS-----------------------------------
    vector<Shape *> scene;      //scene of pointer to objects within scene
    
    ofImage img;          //image output for scene
    
    float sceneSDF(const glm::vec3 &p);     //sceneSDF (based on CS116B.Lec6 pseudocode)
    bool rayMarch(Ray &r, glm::vec3 &p);   //ray-marching algorithm (replicated from CS116B.Lec6)
    void rayMarchLoop();                  //begin rendering img using raymarch (based on CS116B.Lec6 pseudocode)
    glm::vec3 getNormalRM(const glm::vec3 &p);      //normal after hit (replicated from CS116B.Lec6)
};
