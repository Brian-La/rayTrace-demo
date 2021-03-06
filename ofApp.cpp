
#include "ofApp.h"

//STRUCTS/DECLARATIONS===============================================================================
//SPHERE----------------------------------------------------
struct Sphere {
	float center[3];		//center coordinates
	float radius;			//radius of sphere
	float ambient_color[3];		//ambient col
	float diffuse_color[3];		//diffuse col
	float specular_color[3];	//specular col
	float shininess;			//shininess var
	float reflectivity;			//reflectivity var
};


//TRIANGLE--------------------------------------------------
struct Triangle {
	float v0[3];			//1st vertice
	float v1[3];			//2nd
	float v2[3];			//3rd
	float ambient_color[3];		//ambient col
	float diffuse_color[3];		//diffuse col
	float specular_color[3];	//specular col
	float shininess;			//shininess var
	float reflectivity;			//reflectivity var
};


//DECLARAIONS-----------------------------------------------
#define MAX_SPHERES 64			//constant definitions?
#define MAX_TRIANGLES 64

Sphere spheres[MAX_SPHERES];
int num_spheres = 0;
Triangle triangles[MAX_TRIANGLES];
int num_triangles = 0;


//collect triangle vertices
void SetTriangleVertices(int ind, float v0[], float v1[], float v2[]) {
	for (int i = 0; i < 3; i++) {
		triangles[ind].v0[i] = v0[i];
		triangles[ind].v1[i] = v1[i];
		triangles[ind].v2[i] = v2[i];
	}
}



//VECTOR FUNCTIONS====================================================================================
//calculate & return magnitude of a vector
float MyMagnitude(float vec[]) {
	float mag = 0.0;
	for (int i = 0; i < 3; i++) mag += vec[i] * vec[i];
	return sqrt(mag);
}


//normalize vector
void MyNormalize(float vec[]) {
	float mag = MyMagnitude(vec);
	for (int i = 0; i < 3; i++) vec[i] /= mag;
}


//calculate & return dot product of two vectors
float MyDotProduct(const float v1[], const float v2[]) {
	float product = 0;
	for (int i = 0; i < 3; i++) product += v1[i] * v2[i];
	return product;
}


//calculate cross product -> return via v3 vector
void MyCrossProduct(float v1[], float v2[], float v3[]) {
	v3[0] = v1[1] * v2[2] - v2[1] * v1[2];
	v3[1] = v1[2] * v2[0] - v2[2] * v1[0];
	v3[2] = v1[0] * v2[1] - v2[0] * v1[1];
}



//INTERSECTIONS=======================================================================================
//SPHERE (return t) = -DdotM +/- sqrt ((DdotM)^2 - (mag(M)^2 - r^2))
float RaySphereIntersection(
	const Sphere& s, float P[], float D[], float X[], float N[]) {
	float M[3];
	for (int i = 0; i < 3; i++) {
		M[i] = P[i] - s.center[i];				//M = P - C (camera position - center)
	}
	float DdotM = MyDotProduct(D, M);			//DdotM; D = ray
	float discriminant = DdotM * DdotM -
		(MyMagnitude(M) * MyMagnitude(M) - s.radius * s.radius);		//DdotM^2 - (mag(M^2 - r^2))

	//ray intersection occurs
	if (discriminant >= 0) {
		float t = -DdotM - sqrt(discriminant);		//calculate t
		for (int i = 0; i < 3; i++) {
			X[i] = P[i] + t * D[i];				//find/inc pts of intersection based on t (X = P + tD)
		}
		for (int i = 0; i < 3; i++) N[i] = X[i] - s.center[i];		//N = point of intersection - center
		MyNormalize(N);						//normal vector
		return t;
	}
	return -1.0;			//otherwise, return -1 to denote no intersection
}


//TRIANGLE---------------------------------------------------
float RayTriangleIntersection(
	const Triangle& tn, float P[], float D[], float X[], float N[]) {
	float E0[3];			//E0 = v1 - v0
	for (int i = 0; i < 3; i++) E0[i] = tn.v1[i] - tn.v0[i];
	float E1[3];			//E1 = v2 - v1
	for (int i = 0; i < 3; i++) E1[i] = tn.v2[i] - tn.v1[i];
	float E2[3];			//E2 = v0 - v2
	for (int i = 0; i < 3; i++) E2[i] = tn.v0[i] - tn.v2[i];

	MyCrossProduct(E0, E1, N);			//N = E0 x E1 (normal vec)
	MyNormalize(N);						//normalize normal vector
	float k = MyDotProduct(tn.v0, N);			//k = vertex x N
	float t = (k - MyDotProduct(P, N))/MyDotProduct(D, N);		//t = (k - PdotN)/ DdotN (P = cam position)

	for (int i = 0; i < 3; i++) X[i] = P[i] + t * D[i];			//find pts of intersection (X = P + tD)

	float V[3];					//locally allocate V 
	float C[3];					//locally allocate C
	for (int i = 0; i < 3; i++) V[i] = X[i] - tn.v0[i];		//V (R - P) = X - vector position
	MyCrossProduct(E0, V, C);						//C = E0 x V
	float dp0 = MyDotProduct(C, N);					//dp = CdotN...repeat
	for (int i = 0; i < 3; i++) V[i] = X[i] - tn.v1[i];
	MyCrossProduct(E1, V, C);
	float dp1 = MyDotProduct(C, N);
	for (int i = 0; i < 3; i++) V[i] = X[i] - tn.v2[i];
	MyCrossProduct(E2, V, C);
	float dp2 = MyDotProduct(C, N);
	if ((dp0 > -0.01) && (dp1 > -0.01) && (dp2 > -0.01)) {		//if values > 0, return t val
		return t;
	} else {
		return -1.0;			//otherwise, no intersection
	}
}



//LIGHTING============================================================================================
float light_vec[3] = { -0.6, -1, 0.8 };			//light attributes 
float light_pos[3] = { -15.0, -20.0, 30.0 };	//position of light
float cam[3] = { 0, -100, 0 };					//camera position


//SHADOWS----------------------------------------------------
bool show_shadows;
int soft_shadows = 0;

//check shadows (p = point, l = light position)
bool IsInShadow(float p[], float l[]) {
	float ray[3];				//locally allocate ray
	for (int i = 0; i < 3; i++) {
		ray[i] = l[i] - p[i];			//ray (D) = L - P
	}
	float light_mag = MyMagnitude(ray);		//magnitude of ray
	MyNormalize(ray);						//normalize ray


	//spheres----------------------------------------------------------
	for (int s = 0; s < num_spheres; s++) {
		float x[3];		//locally allocate x
		float n[3];		//locally allocate n
		float t = RaySphereIntersection(spheres[s], p, ray, x, n);		//get t (x, n filler)
		if ((t > 0.01) && (t < light_mag)) return true;		//return true if t satisfies conditions
	}

	//triangles--------------------------------------------------------
	for (int tn = 0; tn < num_triangles; tn++) {
		float x[3];
		float n[3];
		float t = RayTriangleIntersection(triangles[tn], p, ray, x, n);
		if ((t > 0.01) && (t < light_mag)) return true;
	}
	return false;
}


//REFLECTIONS------------------------------------------------
//return pixel "seen" by reflected ray
void reflectedPixel(float ray[], float P[], float N[], float t, float color[]) {
	float R[3];		//reflected ray
	//float X[3];		//world coordinates

	for (int i = 0; i < 3; i++) {
		R[i] = -ray[i] + (2 * N[i] * MyDotProduct(ray, N));		//reflected ray (R = -V + 2N(VdotN)
		//cout << R[i] << endl;
		//X[i] = P[i] + t * R[i];
		color[i] = R[i];				//how to get the reflected color from reflected ray?
	}

	//glReadPixels(X[0], X[1], 1, 1, GL_RGB, GL_FLOAT, color);
	//cout << X[0] << " " << X[1] << endl;
}


//LIGHT CALCULATIONS-----------------------------------------
bool calculate_lighting;
bool enable_reflection;

//calculate PIXEL color based on lighting, shadows, reflection (P, N, ambient, diffuse, specular, shininess, PIXEL)
void CalculateLighting(float P[], float N[],
	float amb[], float dif[], float spe[], float shininess, float reflectivity, float ray[], float t,
	float pixel[]) {
	// Ambient
	for (int i = 0; i < 3; i++) pixel[i] = amb[i];			//set pixel to ambient light
	// Diffuse
	float L[3];						//locally allocate L
	for (int i = 0; i < 3; i++) L[i] = light_pos[i] - P[i];	//L = light position - vertex position
	MyNormalize(L);					//normalize L
	for (int i = 0; i < 3; i++) {
		pixel[i] += dif[i] * MyDotProduct(N, L);			//add to pixel: diffuse col * NdotL
	}
	// Specular
	float V[3];					//locally allocate V
	for (int i = 0; i < 3; i++) V[i] = cam[i] - P[i];		//V = cam position - vertex position
	MyNormalize(V);				//normalize V
	float H[3];					//locally store H
	for (int i = 0; i < 3; i++) H[i] = V[i] + L[i];			//H = V + L
	MyNormalize(H);				//normalize H
	for (int i = 0; i < 3; i++) {
		pixel[i] += spe[i] * pow(MyDotProduct(N, H), shininess);	//add to pixel: spec color * (NdotH)^shininess
	}


	//SHADOW--------------------------------------------------------------------
	//if show_shadows enabled...
	if(show_shadows) {
		//if pixel is considered a shadow
		if (IsInShadow(P, light_pos)) {
			for (int i = 0; i < 3; i++) {
				pixel[i] = 0.1;			//convert pixel to shadow
			}
		}
	}

	//REFLECTION-----------------------------------------------------------------
	//if reflection enabled...
	if (enable_reflection) {
		//if reflectivity > 0...
		if (reflectivity > 0) {
			float rColor[3];
			reflectedPixel(ray, P, N, t, rColor);
			
			for (int i = 0; i < 3; i++)	pixel[i] = ((1 - reflectivity) * pixel[i]) + (reflectivity * rColor[i]);	//(1 - r) * InstrinsicCol + r * ReflectiveCol
		}
	}
}


//calculate sphere lighting (s = shininess)
void CalculateSphereLighting(float p[], float N[], int s, float ray[], float t, float pixel[]) {
	CalculateLighting(
		p, N, spheres[s].ambient_color, spheres[s].diffuse_color,
		spheres[s].specular_color, spheres[s].shininess, spheres[s].reflectivity, ray, t, pixel);
}


//calculate triangle lighting (N = vertices, t = shininess)
void CalculateTriangleLighting(float p[], float N[3], int t, float ray[], float t1, float pixel[]) {
	CalculateLighting(
		p, N, triangles[t].ambient_color, triangles[t].diffuse_color,
		triangles[t].specular_color, triangles[t].shininess, triangles[t].reflectivity, ray, t1, pixel);
}



//RAY-TRACE==========================================================================================
#define MAX_ITER 8
void RayTrace(float cam[], float ray[], float pixel[], int iter) {
	float current_t = -1.0;
	for (int i = 0; i < 3; i++) pixel[i] = 0.0;

	// Try to hit any sphere.
	for (int s = 0; s < num_spheres; s++) {
		float X[3];								//locally allocate X
		float N[3];								//locally allocate N
		float t = RaySphereIntersection(spheres[s], cam, ray, X, N);		//find t to get point intersection
		if (t > 0.00001) {									//if t exists and is closest to camera...
			if ((t < current_t) || (current_t < 0.0)) {			
				if (calculate_lighting) {
					CalculateSphereLighting(X, N, s, ray, t, pixel);	//calculate lighting
				} else {
					pixel[2] = 1;			//blue if bool off
				}
				current_t = t;			//set new current_t
			}
		}
	}

	// Try to hit any triangle.
	for (int tn = 0; tn < num_triangles; tn++) {
		float X[3];								//locally allocate X
		float N[3];								//locally allocate N
		float t = RayTriangleIntersection(triangles[tn], cam, ray, X, N);		//same procedure as sphere
		if (t > 0.00001) {
			if ((t < current_t) || (current_t < 0.0)) {
				if (calculate_lighting) {
					CalculateTriangleLighting(X, N, tn, ray, t, pixel);
				} else {
					pixel[2] = 1.0;
				}
				current_t = t;
			}
		}
	}
}



//RE-DRAW============================================================================================
#define WINWIDTH 512					
#define WINHEIGHT 512
float pixels[WINWIDTH][WINHEIGHT][3];			//colors @ each pixel thru window screen

bool more_objects;
int anti_alias = 0;

//int depth_of_field = 0;
void SetUpScene() {
	spheres[0].center[0] = 0;
	spheres[0].center[1] = 5;
	spheres[0].center[2] = 0;
	spheres[0].radius = 10;
	spheres[0].ambient_color[0] = 0.05;
	spheres[0].ambient_color[1] = 0.05;
	spheres[0].ambient_color[2] = 0.1;
	spheres[0].diffuse_color[0] = 0.0;
	spheres[0].diffuse_color[1] = 0.5;
	spheres[0].diffuse_color[2] = 0.5;
	spheres[0].specular_color[0] = 0.0;
	spheres[0].specular_color[1] = 0.0;
	spheres[0].specular_color[2] = 0.0;
	spheres[0].shininess = 0.0;
	spheres[0].reflectivity = 0.8;
	num_spheres = 1;


	//set triangle to initial setup
	float v0[3] = { -50.0, -50.0, -50.0 };	// bottom left front
	float v1[3] = { 50.0, -50.0, -50.0 };	// bottom right front
	float v2[3] = { 50.0, 50.0, -50.0 };	// bottom right back
	float v3[3] = { -50.0, 50.0, -50.0 };	// bottom left back
	SetTriangleVertices(0, v0, v1, v3);			//two triangles

	triangles[0].ambient_color[0] = 0.1;
	triangles[0].ambient_color[1] = 0.1;
	triangles[0].ambient_color[2] = 0.1;
	triangles[0].diffuse_color[0] = 0.9;
	triangles[0].diffuse_color[1] = 0.4;
	triangles[0].diffuse_color[2] = 0.0;
	triangles[0].specular_color[0] = 0.0;
	triangles[0].specular_color[1] = 0.2;
	triangles[0].specular_color[2] = 0.0;
	triangles[0].shininess = 0.0;
	triangles[0].reflectivity = 0.0;

	num_triangles = 1;

	if (more_objects) {
		spheres[1].center[0] = 15;
		spheres[1].center[1] = 18;
		spheres[1].center[2] = 20;
		spheres[1].radius = 20;
		spheres[1].ambient_color[0] = 0.05;
		spheres[1].ambient_color[1] = 0.05;
		spheres[1].ambient_color[2] = 0.1;
		spheres[1].diffuse_color[0] = 0.0;
		spheres[1].diffuse_color[1] = 0.0;
		spheres[1].diffuse_color[2] = 0.5;
		spheres[1].specular_color[0] = 0.4;
		spheres[1].specular_color[1] = 0.4;
		spheres[1].specular_color[2] = 0.4;
		spheres[1].shininess = 10.0;
		spheres[1].reflectivity = 0.0;


		spheres[2].center[0] = -15;
		spheres[2].center[1] = 0;
		spheres[2].center[2] = -15;
		spheres[2].radius = 15;
		spheres[2].ambient_color[0] = 0.05;
		spheres[2].ambient_color[1] = 0.1;
		spheres[2].ambient_color[2] = 0.1;
		spheres[2].diffuse_color[0] = 0.0;
		spheres[2].diffuse_color[1] = 0.7;
		spheres[2].diffuse_color[2] = 0.3;
		spheres[2].specular_color[0] = 0.3;
		spheres[2].specular_color[1] = 0.3;
		spheres[2].specular_color[2] = 0.3;
		spheres[2].shininess = 10.0;
		spheres[2].reflectivity = 0.0;

		num_spheres = 3;


		float v4[3] = { -50.0, -50.0, 50.0 };	// top left front
		float v5[3] = { 50.0, -50.0, 50.0 };	// top right front
		float v6[3] = { 50.0, 50.0, 50.0 };		// top right back
		float v7[3] = { -50.0, 50.0, 50.0 };	// top left back

		SetTriangleVertices(1, v1, v2, v3);
		SetTriangleVertices(2, v3, v2, v7);
		SetTriangleVertices(3, v2, v6, v7);
		SetTriangleVertices(4, v2, v1, v6);
		SetTriangleVertices(5, v1, v5, v6);
		SetTriangleVertices(6, v0, v3, v4);
		SetTriangleVertices(7, v3, v7, v4);


		float v8[3] = { 0.0, -50.0, -30.0 };		//middle left front
		float v9[3] = { 50.0, -50.0, -30.0 };		//middle right front
		float va[3] = { 50.0, 50.0, -30.0 };		//middle right back
		float vb[3] = { 0.0, 50.0, -30.0 };			//middle left back
		SetTriangleVertices(8, v8, v9, va);
		SetTriangleVertices(9, v8, va, vb);


		for (int i = 0; i < 10; i++) {
			triangles[i].ambient_color[0] = 0.1;
			triangles[i].ambient_color[1] = 0.1;
			triangles[i].ambient_color[2] = 0.1;
			triangles[i].diffuse_color[0] = 0.9;
			triangles[i].diffuse_color[1] = 0.0;
			triangles[i].diffuse_color[2] = 0.0;
			triangles[i].specular_color[0] = 0.0;
			triangles[i].specular_color[1] = 0.0;
			triangles[i].specular_color[2] = 0.0;
			triangles[i].shininess = 0.0;
			triangles[i].reflectivity = 0.0;
		}
		triangles[3].diffuse_color[2] = 0.7;
		triangles[8].diffuse_color[1] = 0.7;
		triangles[9].diffuse_color[1] = 0.7;
		triangles[8].reflectivity = 0.7;
		triangles[9].reflectivity = 0.7;

		num_triangles = 10;
	}

	MyNormalize(light_vec);
}


void DrawScene() {
	float screen[3] = { 0, 0, 0 };			//default 0, 0, 0
	for (int i = 0; i < WINWIDTH; i++) {
		screen[0] = 0.5 * (float)(-WINWIDTH / 2 + i);		
		for (int j = 0; j < WINHEIGHT; j++) {
			screen[2] = 0.5 * (float)(-WINHEIGHT / 2 + j);
			float ray[3];
			for (int k = 0; k < 3; k++) ray[k] = screen[k] - cam[k];
			MyNormalize(ray);
			RayTrace(cam, ray, pixels[i][j], 0);
		}
	}
}


//MAIN-FUNCS========================================================================================
void ofApp::setup() {

	calculate_lighting = true;		//default true
	more_objects = true;			
	show_shadows = false;
	soft_shadows = 0;
	anti_alias = 0;
	enable_reflection = false;
	SetUpScene();				//set up and draw scene
	DrawScene();
	ofSetFrameRate(2);

}


//--------------------------------------------------------------
void ofApp::update() {
	//cout << "update" << endl;
}


//--------------------------------------------------------------
void ofApp::draw() {

	//cout << "draw" << endl;

	ofBackground(0);			//default black
	ofMesh m;					//mesh for screen

	m.setMode(OF_PRIMITIVE_POINTS);
	int n = 0;

	//calculate_lighting = 1;
	for (int i = 0; i < WINWIDTH; i++) {				//horizontal
		for (int j = 0; j < WINHEIGHT; j++) {			//vertical
			m.addVertex({ i, WINHEIGHT-j, 0 });			//add vertex (start bottom left corner -> top left corner)
			m.addColor({ pixels[i][j][0], pixels[i][j][1], pixels[i][j][2] });			//add color for each pixel
			m.addIndex(n);					//add index to mesh and connect
			n++;
		}
	}

	m.draw();			//draw mesh

}


//--------------------------------------------------------------
void ofApp::keyPressed(int key) {

	cout << "key pressed: " << key << endl;

	switch (key) {
	case 'L':
	case 'l':
		calculate_lighting = !calculate_lighting;
		break;
	case 'M':
	case 'm':
		more_objects = !more_objects;
		break;
	case 'S':
	case 's':
		show_shadows = !show_shadows;
		break;
	case 'F':
	case 'f':
		soft_shadows = 1 - soft_shadows;
		break;
	case 'A':
	case 'a':
		anti_alias = 1 - anti_alias;
		break;
	case 'R':
	case 'r':
		enable_reflection = !enable_reflection;
		break;
	}

	//refresh every time a key is pressed
	SetUpScene();
	DrawScene();

}

/*
//--------------------------------------------------------------
void ofApp::keyReleased(int key) {

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {
	//cout << "mouse moved" << endl;
}


//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {

}
*/
