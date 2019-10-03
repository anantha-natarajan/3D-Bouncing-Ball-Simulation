/*------------------------------------------------------------------------------------------------------------------------------------------------------------

                                                   SIMULATING A BOUNCING BALL INSIDE A BOX
													 Code Written by ANANTHA NATARAJAN
													    Implemented using OpenGl 3

								              -----------------------------------------------------

External Libraries Used:
	*	GLFW for windowing operations
	*	GLM for mathematical operations on vectors and matrices
	*	ImGui for creating the User Interface elements

GUI instructions
	*	The Initial Position must be updated using "Update Initial Position" Button in the GUI before the start of the simulation
	*	The Velocity vector can be adjusted any time during the simulation but "Update Velocity" button should be pressed for the changes to apply
	*	The other buttons can be pressed during runtime.
	*	If the body comes to rest, the simulation must be stopped and started again for the changes to affect


--------------------------------------------------------------------------------------------------------------------------------------------------------------*/
#include <iostream>
#include <vector>

#define GLEW_STATIC
#include "GL/glew.h"
#include "GLFW/glfw3.h"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/constants.hpp"
#include "glm/gtx/scalar_multiplication.hpp"

#include "ShaderProgram.h"
#include "Camera.h"
#include "State.h"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

bool gFullScreen = false;
GLFWwindow *pwindow = NULL;
bool gWireframe = false;
int gWindowWidth = 1280;
int gWindowHeight = 800;

GLFWwindow *GUIwindow = NULL;
int guiWindowWidth = 630;
int guiWindowHeight = 600;


bool simulation = false;
float displayRate = 0.0125f;

OrbitCamera orbitCam;
float gYaw = 0.0f;
float gPitch = 0.0f;
float gRadius = 1000.0f;
const float MOUSE_SENSITIVITY = 0.25f;

float boxSize = 20.0f;
std::vector<glm::vec3> normalsCube;
std::vector<glm::vec3> positionsCube;

double fractionTimeStep, fX = 1.0f, fY = 1.0f, fZ = 1.0f;
double fracDistance[6];

float sphereRadius = 1.0f;
glm::vec3 startSpherePos = glm::vec3(0.0f, 0.0f, -100.0f);
glm::vec3 windVelocity = glm::vec3(0.0f, 0.0f, 0.0f);
glm::vec3 initialVelocity = glm::vec3(0.0f, 0.0f, 0.0f);
float g = -10.0;
float restitution = 0.9f, friction = 0.0f;

void onKeyPress(GLFWwindow* window, int key, int scancode,int action, int mode);
void glfw_OnFrameBufferSize(GLFWwindow* window,int width, int height);
void glfw_onMouseMove(GLFWwindow* window, double posX, double posY);
bool initOpengl();

bool detectCollision(glm::vec3 ballCenter, double newDistance[], double oldDistance[], int &colliderPlaneNormal);
void calculateDistance(glm::vec3 ballCenter, double distance[]);

int main() {

	if (!initOpengl()) {
		std::cout << "OpenGl initialization failed" << std::endl;
		return -1;
	}

	ImVec4 clear_color = ImVec4(0.22f, 0.289f, 0.42f, 1.00f);

	normalsCube.push_back(glm::vec3(0.0f, 0.0f, -1.0f)); //index0
	normalsCube.push_back(glm::vec3(0.0f, 0.0f, 1.0f)); //index1
	normalsCube.push_back(glm::vec3(1.0f, 0.0f, 0.0f)); //index2
	normalsCube.push_back(glm::vec3(-1.0f, 0.0f, 0.0f)); //index3
	normalsCube.push_back(glm::vec3(0.0f, -1.0f, 0.0f)); //index4
	normalsCube.push_back(glm::vec3(0.0f, 1.0f, 0.0f)); //index5

	positionsCube.push_back(glm::vec3(0.0f, 0.0f, -100 + boxSize / 2));   //index0
	positionsCube.push_back(glm::vec3(0.0f, 0.0f, -100 - boxSize / 2));  //index1
	positionsCube.push_back(glm::vec3(-boxSize / 2, 0.0f, 0.0f));    //index2
	positionsCube.push_back(glm::vec3(boxSize / 2, 0.0f, 0.0f));    //index3
	positionsCube.push_back(glm::vec3(0.0f, boxSize / 2, 0.0f));    //index4
	positionsCube.push_back(glm::vec3(0.0f, -boxSize / 2, 0.0f));    //index5

	GLfloat vertices1[] = {
		// position           // color
		-0.5f,  0.5f, 0.0f,   1.0f, 0.0f, 0.0f,  // left top
		 0.5f,  0.5f, 0.0f,   0.0f, 0.0f, 1.0f,  // left right bottom
		 0.5f, -0.5f, 0.0f,   0.0f, 1.0f, 0.0f,  // left left bottom		 
		-0.5f, -0.5f, 0.0f,   0.0f, 0.0f, 1.0f
	};

	float front[3],left[3],top[3];
	front[0] = (float)75 / 256;
	front[1] = (float)174 / 256;
	front[2] = (float)160 / 256;

	left[0] = (float)182 / 256;
	left[1] = (float)230 / 256;
	left[2] = (float)189/256;

	top[0] = (float)176/256;
	top[1] = (float)46/256;
	top[2] = (float)108/256;

	GLfloat vertices[] = {
		// Back face
		-0.5f, -0.5f, -0.5f, front[0], front[1] , front[2], // Bottom-left
		 0.5f,  0.5f, -0.5f, front[0], front[1] , front[2], // top-right
		 0.5f, -0.5f, -0.5f, front[0], front[1] , front[2], // bottom-right         
		 0.5f,  0.5f, -0.5f, front[0], front[1] , front[2], // top-right
		-0.5f, -0.5f, -0.5f, front[0], front[1] , front[2], // bottom-left
		-0.5f,  0.5f, -0.5f, front[0], front[1] , front[2], // top-left
		// Front face
		-0.5f, -0.5f,  0.5f, front[0], front[1] , front[2], // bottom-left
		 0.5f, -0.5f,  0.5f, front[0], front[1] , front[2], // bottom-right
		 0.5f,  0.5f,  0.5f, front[0], front[1] , front[2], // top-right
		 0.5f,  0.5f,  0.5f, front[0], front[1] , front[2], // top-right
		-0.5f,  0.5f,  0.5f, front[0], front[1] , front[2], // top-left
		-0.5f, -0.5f,  0.5f, front[0], front[1] , front[2], // bottom-left
		// Left face
		-0.5f,  0.5f,  0.5f, left[0], left[1],left[2], // top-right
		-0.5f,  0.5f, -0.5f, left[0], left[1],left[2], // top-left
		-0.5f, -0.5f, -0.5f, left[0], left[1],left[2], // bottom-left
		-0.5f, -0.5f, -0.5f, left[0], left[1],left[2], // bottom-left
		-0.5f, -0.5f,  0.5f, left[0], left[1],left[2], // bottom-right
		-0.5f,  0.5f,  0.5f, left[0], left[1],left[2], // top-right
		// Right face
		 0.5f,  0.5f,  0.5f, left[0], left[1],left[2], // top-right
		 0.5f, -0.5f, -0.5f, left[0], left[1],left[2], // top-left
		 0.5f,  0.5f, -0.5f, left[0], left[1],left[2], // bottom-left     
		 0.5f, -0.5f, -0.5f, left[0], left[1],left[2], // bottom-left
		 0.5f,  0.5f,  0.5f, left[0], left[1],left[2], // bottom-right
		 0.5f, -0.5f,  0.5f, left[0], left[1],left[2], // top-right   
		// Bottom face
		-0.5f, -0.5f, -0.5f, top[0], top[1],top[2], // top-right
		 0.5f, -0.5f, -0.5f, top[0], top[1],top[2], // top-left
		 0.5f, -0.5f,  0.5f, top[0], top[1],top[2], // bottom-left
		 0.5f, -0.5f,  0.5f, top[0], top[1],top[2], // bottom-left
		-0.5f, -0.5f,  0.5f, top[0], top[1],top[2], // bottom-right
		-0.5f, -0.5f, -0.5f, top[0], top[1],top[2], // top-right
		// Top face
		-0.5f,  0.5f, -0.5f, top[0], top[1],top[2], // top-right
		 0.5f,  0.5f,  0.5f, top[0], top[1],top[2], // top-left
		 0.5f,  0.5f, -0.5f, top[0], top[1],top[2], // bottom-left
		 0.5f,  0.5f,  0.5f, top[0], top[1],top[2], // bottom-left
		-0.5f,  0.5f, -0.5f, top[0], top[1],top[2], // bottom-right
		-0.5f,  0.5f,  0.5f, top[0], top[1],top[2], // top-right
	};


	GLfloat vertices2[] = {
	//front face
	-0.5f,  0.5f,  0.5f, 0.2f, 0.0f, 0.4f,
	 0.5f,  0.5f,  0.5f, 0.2f, 0.0f, 0.4f,
	 0.5f, -0.5f,  0.5f, 0.2f, 0.0f, 0.4f,
	-0.5f,  0.5f,  0.5f, 0.2f, 0.0f, 0.4f,
	 0.5f, -0.5f,  0.5f, 0.2f, 0.0f, 0.4f,
	-0.5f, -0.5f,  0.5f, 0.2f, 0.0f, 0.4f,

	//back face
	-0.5f,  0.5f, -0.5f, 1.0f, 0.2f, 0.0f,
	 0.5f,  0.5f, -0.5f, 1.0f, 0.2f, 0.0f,
	 0.5f, -0.5f, -0.5f, 1.0f, 0.2f, 0.0f,
	-0.5f,  0.5f, -0.5f, 1.0f, 0.2f, 0.0f,
	 0.5f, -0.5f, -0.5f, 1.0f, 0.2f, 0.0f,
	-0.5f, -0.5f, -0.5f, 1.0f, 0.2f, 0.0f,

	
	//left face
	-0.5f,  0.5f, -0.5f, 0.5f, 0.5f, 0.0f,
	-0.5f,  0.5f,  0.5f, 0.5f, 0.5f, 0.0f,
	-0.5f, -0.5f,  0.5f, 0.5f, 0.5f, 0.0f,
	-0.5f,  0.5f, -0.5f, 0.5f, 0.5f, 0.0f,
	-0.5f, -0.5f,  0.5f, 0.5f, 0.5f, 0.0f,
	-0.5f, -0.5f, -0.5f, 0.5f, 0.5f, 0.0f,

	//right face
	 0.5f,  0.5f, -0.5f, 0.5f, 0.5f, 0.0f,
	 0.5f,  0.5f,  0.5f, 0.5f, 0.5f, 0.0f,
	 0.5f, -0.5f,  0.5f, 0.5f, 0.5f, 0.0f,
	 0.5f,  0.5f, -0.5f, 0.5f, 0.5f, 0.0f,
	 0.5f, -0.5f,  0.5f, 0.5f, 0.5f, 0.0f,
	 0.5f, -0.5f, -0.5f, 0.5f, 0.5f, 0.0f,

	 //top face
	-0.5f,  0.5f, -0.5f, 0.5f, 0.0f, 0.5f,
	 0.5f,  0.5f, -0.5f, 0.5f, 0.0f, 0.5f,
	 0.5f,  0.5f,  0.5f, 0.5f, 0.0f, 0.5f,
	-0.5f,  0.5f, -0.5f, 0.5f, 0.0f, 0.5f,
	 0.5f,  0.5f,  0.5f, 0.5f, 0.0f, 0.5f,
	-0.5f,  0.5f,  0.5f, 0.5f, 0.0f, 0.5f,

	//bottom face

    -0.5f, -0.5f, -0.5f, 0.5f, 0.0f, 0.5f,
	 0.5f, -0.5f, -0.5f, 0.5f, 0.0f, 0.5f,
	 0.5f, -0.5f,  0.5f, 0.5f, 0.0f, 0.5f,
    -0.5f, -0.5f, -0.5f, 0.5f, 0.0f, 0.5f,
  	 0.5f, -0.5f,  0.5f, 0.5f, 0.0f, 0.5f,
    -0.5f, -0.5f,  0.5f, 0.5f, 0.0f, 0.5f,

	};

	//     ---------------------      STARTING VERTEX GENERATION OF A SPHERE WITH RADIUS 1       ----------------------

	int sectorCount = 18;
	int stackCount = 9;
	GLfloat radius = sphereRadius;

	std::vector<GLfloat> sphereVertices;
	GLfloat sx, sy, sz, sxy;

	GLfloat sectorStep = 2 * glm::pi<float>() / sectorCount;
	GLfloat stackStep = glm::pi<float>() / stackCount;
	GLfloat sectorAngle, stackAngle;
	glm::vec3 baseColor(0.0f, 0.0f, 0.0f);

	for (int i = 0; i <= stackCount; ++i)
	{
		stackAngle = glm::pi<float>() / 2 - i * stackStep;        // starting from pi/2 to -pi/2
		sxy = radius * cosf(stackAngle);                          // r * cos(u)
		sz = radius * sinf(stackAngle);                           // r * sin(u)

		for (int j = 0; j <= sectorCount; ++j)
		{
			sectorAngle = j * sectorStep;                         // starting from 0 to 2pi

			sx = sxy * cosf(sectorAngle);                          // r * cos(u) * cos(v)
			sy = sxy * sinf(sectorAngle);                          // r * cos(u) * sin(v)
			sphereVertices.push_back(sx);
			sphereVertices.push_back(sy);
			sphereVertices.push_back(sz);
		
		}

	}

	std::vector<GLuint> sphereIndices;
	int k1, k2;
	for (int i = 0; i < stackCount; ++i)
	{
		k1 = i * (sectorCount + 1);     // beginning of current stack
		k2 = k1 + sectorCount + 1;      // beginning of next stack

		for (int j = 0; j < sectorCount; ++j, ++k1, ++k2)
		{
			// 2 triangles per sector excluding first and last stacks
			// k1 => k2 => k1+1
			if (i != 0)
			{
				sphereIndices.push_back(k1);
				sphereIndices.push_back(k2);
				sphereIndices.push_back(k1 + 1);
			}

			// k1+1 => k2 => k2+1
			if (i != (stackCount - 1))
			{
				sphereIndices.push_back(k1 + 1);
				sphereIndices.push_back(k2);
				sphereIndices.push_back(k2 + 1);
			}
		}
	}


	GLuint vboS, vaoS, eboS;
	glGenVertexArrays(1, &vaoS);
	glBindVertexArray(vaoS);

	glGenBuffers(1, &vboS);
	glBindBuffer(GL_ARRAY_BUFFER, vboS);
	glBufferData(GL_ARRAY_BUFFER, sphereVertices.size() * sizeof(GLfloat), &sphereVertices[0], GL_STATIC_DRAW);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE,0, NULL);
	glEnableVertexAttribArray(0);

	glGenBuffers(1, &eboS);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eboS);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sphereIndices.size() * sizeof(GLuint), &sphereIndices[0], GL_STATIC_DRAW);

	//     ---------------------      ENDING VERTEX GENERATION & BUFFER ACTIONS OF A SPHERE WITH     ------------------------------------


	GLuint vbo, vao, ebo;

	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);

	glGenBuffers(1, &vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

	//position
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat)*6, NULL);
	glEnableVertexAttribArray(0);

	//color
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(GLfloat) * 6, (GLvoid*)(sizeof(GLfloat)*3));
	glEnableVertexAttribArray(1);

	//     ---------------------      ENDING VERTEX GENERATION & BUFFER ACTIONS OF A CUBE     ------------------------------------

	ShaderProgram shaderProgram;
	shaderProgram.loadShader("basic.vert" , "basic.frag");

	glfwSetFramebufferSizeCallback(pwindow, glfw_OnFrameBufferSize);

	glm::vec3 endSpherePos = glm::vec3(0.0f, -15.0f, 0.0f);
	glm::vec3 origin = glm::vec3(0.0f, 0.0f, -100.0f);
	glm::vec3 cubePos = glm::vec3(0.0f, 0.0f, -5.0f);
	float cubeangle = 0.0f;
	double d = 0.4;
	double mass = 1;

	float t = 0;
	double n = 0;
	float h = 0.00550;
	float vOld = 0;
	float xOld = 100;
	float xNew, vNew;

	bool CollisionDetected = false;
	State s;
	s.velocity = initialVelocity;
	s.position = startSpherePos;
	s.acceleration = glm::vec3(0.0f, g, 0.0f);
	s.printState();
	glm::vec3 ballPosition = startSpherePos;
	bool rest = false;

	double distance[6];
	calculateDistance(s.position, distance);
	double displayTimer = displayRate;
	double lastTime = glfwGetTime();
	double simulationTimer = h;
	double simulationLastTime = glfwGetTime();


	glm::vec3 pos = startSpherePos;
	glm::vec3 vel = initialVelocity;
	
	//setting default values to be used for disaply in the GUI
	float p[3], v[3], wv[3];

	p[0] = startSpherePos.x;
	p[1] = startSpherePos.y;
	p[2] = startSpherePos.z;

	v[0] = initialVelocity.x;
	v[1] = initialVelocity.y;
	v[2] = initialVelocity.z;

	wv[0] = windVelocity.x;
	wv[1] = windVelocity.y;
	wv[2] = windVelocity.z;

	while (!glfwWindowShouldClose(pwindow)) {

		for (int i = 0; i < 6; i++) {
			positionsCube.pop_back();
		}

		startSpherePos.x = p[0];
		startSpherePos.y = p[1];
		startSpherePos.z = p[2];

		initialVelocity.x = v[0];
		initialVelocity.y = v[1];
		initialVelocity.z = v[2];
		
		s.acceleration = glm::vec3(0.0f, g, 0.0f);

		positionsCube.push_back(glm::vec3(0.0f, 0.0f, -100 + boxSize / 2));   //index0
		positionsCube.push_back(glm::vec3(0.0f, 0.0f, -100 - boxSize / 2));  //index1
		positionsCube.push_back(glm::vec3(-boxSize / 2, 0.0f, 0.0f));    //index2
		positionsCube.push_back(glm::vec3(boxSize / 2, 0.0f, 0.0f));    //index3
		positionsCube.push_back(glm::vec3(0.0f, boxSize / 2, 0.0f));    //index4
		positionsCube.push_back(glm::vec3(0.0f, -boxSize / 2, 0.0f));    //index5

		glfwPollEvents();

		double currentTime = glfwGetTime();
		double deltaTime = currentTime - lastTime;
		displayTimer -= deltaTime;
		lastTime = currentTime;

		if (displayTimer < 0.0f) {
			displayTimer = displayRate;
			ballPosition = s.position;
			n = 0;
		}

		double simulationDeltaTime = currentTime - simulationLastTime;
		simulationTimer -= simulationDeltaTime;
		simulationLastTime = currentTime;

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		
		glm::mat4 model, view, projection, groundModel, boxModel;
		model = glm::mat4(1.0f);
		groundModel = glm::mat4(1.0f);
		
		orbitCam.setLookAt(origin);
		orbitCam.rotate(gYaw, gPitch);
		orbitCam.setRadius(gRadius);

		shaderProgram.use();

		if (simulation) {

			if (n < displayRate) {
				glm::vec3 newV, newP, netAcceleration;

				if (!rest) {
					s.time = n;
					netAcceleration = s.acceleration + (windVelocity - s.velocity) * d;
					newV = s.velocity + netAcceleration * h;
					newP = s.position + s.velocity * h;

					//check for resting conditions
					double Velocity = glm::length(newV);
					if (Velocity > -0.05 && Velocity < 0.05) {
						if (distance[5] > -0.05 && distance[5] < 0.05){
							double netForce = glm::dot((windVelocity - s.velocity) * d, normalsCube[5]);
							if (netForce > -0.01 && netForce <0.01) {
								std::cout << "Sphere comes to rest" << std::endl;
								rest = true;
							}
							
						}
					}
					
					double newDistance[6];
					calculateDistance(newP, newDistance);
					int collidingPlaneIndex;
					CollisionDetected = detectCollision(newP, newDistance, distance, collidingPlaneIndex);
					double timeRemaining;
					if (CollisionDetected && n < displayRate) {
						
						timeRemaining = h;
						L:
						glm::vec3 collisionPos, collisionV;
						int collidingPlane;
						glm::vec3 normalCollidingPlane = normalsCube[collidingPlaneIndex];
						netAcceleration = s.acceleration + (windVelocity - s.velocity) * d;
						collisionV = s.velocity + netAcceleration * timeRemaining * fractionTimeStep;
						collisionPos = s.position + s.velocity * timeRemaining * fractionTimeStep;

						glm::vec3 vNormBC, vTangBC, vNormAC, vTangAC;

						vNormBC = glm::dot(collisionV, normalCollidingPlane) * normalCollidingPlane;
						vTangBC = collisionV - vNormBC;

						vNormAC = -(restitution * vNormBC);
						vTangAC = (1 - friction)*vTangBC;

						collisionV = vNormAC + vTangAC;
						newV = collisionV + netAcceleration * timeRemaining * (1 - fractionTimeStep);
						newP = collisionPos + collisionV * timeRemaining * (1 - fractionTimeStep);
						
						s.time = fractionTimeStep * timeRemaining;
						s.velocity = collisionV;

						calculateDistance(s.position, distance);
						calculateDistance(newP, newDistance);
						timeRemaining = timeRemaining - s.time;
						CollisionDetected = detectCollision(newP, newDistance, distance, collidingPlaneIndex);

						if (CollisionDetected && timeRemaining > 0) {
							std::cout << "Another Collision happening" << std::endl;
							goto L;
						}

						n = n + h;
					}

					s.velocity = newV;
					s.position = newP;

					calculateDistance(newP, distance);

					n = n + h;
					t = n * h;
					simulationTimer = h;
				}

			}
			
		} else {

			ballPosition = startSpherePos;
		}


		model = glm::translate(model, ballPosition) * glm::scale(model, glm::vec3(sphereRadius, sphereRadius, sphereRadius));
		view = orbitCam.getViewMatrix();
		projection = glm::perspective(glm::radians(45.0f), (float)gWindowWidth / (float)gWindowHeight, 0.1f, 500.0f);

		shaderProgram.setUniform("model", model);
		shaderProgram.setUniform("view", view);
		shaderProgram.setUniform("projection", projection);

		glBindVertexArray(vaoS);
		glDrawElements(GL_TRIANGLES, sphereIndices.size(), GL_UNSIGNED_INT, (void*)0);
		glBindVertexArray(vao);

		boxModel = glm::translate(boxModel, origin) * glm::scale(boxModel, glm::vec3(boxSize, boxSize, boxSize));
		shaderProgram.setUniform("model", boxModel);
		glDrawArrays(GL_TRIANGLES, 0, 36);


		glBindVertexArray(0);

		glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);

		glfwMakeContextCurrent(GUIwindow);
		glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT);

		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		static float f = 0.0f;
		static int counter = 0;

		ImGui::Begin("Simulation Control");                          
		ImGui::Text("This GUI provides controls to test out various simulating conditions"); 
	
		ImGui::NewLine();
		ImGui::Checkbox("WireFrame Mode", &gWireframe);// 

		ImGui::NewLine();
		ImGui::InputFloat3("Initial Position", p, 2, 0);
		if (ImGui::Button("Update Initial Position")) {
			s.position.x = p[0];
			s.position.y = p[1];
			s.position.z = p[2];
		}

		ImGui::InputFloat3("Velocity", v, 2, 0);
		if (ImGui::Button("Update Velocity")) {
			s.velocity.x = v[0];
			s.velocity.y = v[1];
			s.velocity.z = v[2];
		}
		ImGui::InputFloat3("Wind Velocity", wv, 2, 0);

		if (ImGui::Button("Update Wind Velocity")) {
			windVelocity.x = wv[0];
			windVelocity.y = wv[1];
			windVelocity.z = wv[2];
		}

		ImGui::NewLine();
		ImGui::SliderFloat("Gravity", &g, -10.0f, 10.0f);
		ImGui::SliderFloat("Friction", &friction, 0.0f, 1.0f);
		ImGui::SliderFloat("Elasticity", &restitution, 0.0f, 2.0f);

		ImGui::NewLine();
		ImGui::SliderFloat("Sphere Size", &sphereRadius, 0.5f, 2.5f);
		ImGui::SliderFloat("Box Size", &boxSize, 10.0f, 40.0f);      
		ImGui::ColorEdit3("BackGround Color", (float*)&clear_color);

		ImGui::NewLine();
		if (ImGui::Button("Start Simulation")) {
			simulation = true;
		}
		ImGui::SameLine();
		if (ImGui::Button("Stop Simulation")) {
			s.velocity = initialVelocity;
			s.position = startSpherePos;
			s.acceleration = glm::vec3(0.0f, g, 0.0f);
			rest = false;
			float t = 0;
			double n = 0;
			calculateDistance(s.position, distance);
			simulation = false;
		}


		ImGui::NewLine();
		ImGui::SliderFloat("Display Rate", &displayRate, 0.0f, 0.5f);
		ImGui::SliderFloat("Time Step", &h, 0.001f, 0.05f);

		//ImGui::Text("Display rate is (%.1f DPS)", displayRate*60);
		ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
		ImGui::End();


		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		glfwSwapBuffers(GUIwindow);

		glfwMakeContextCurrent(pwindow);

		if (gWireframe) {
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		}
		else {
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		}
		

		glfwSwapBuffers(pwindow);
		
	}

	glDeleteVertexArrays(1, &vao);
	glDeleteBuffers(1, &vbo);
	glDeleteBuffers(1, &eboS);
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();

	glfwTerminate();
	return 0;

}

bool initOpengl() {

	if (!glfwInit()) {
		std::cout << " GLFW initialization failed" << std::endl;
		return false;
	}

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	if (gFullScreen) {
		GLFWmonitor* pMonitor = glfwGetPrimaryMonitor();
		const GLFWvidmode* pVmode = glfwGetVideoMode(pMonitor);
		if (pVmode != NULL) {
			pwindow = glfwCreateWindow(pVmode->width, pVmode->height, "Render Window", pMonitor, NULL);
			GUIwindow = glfwCreateWindow(pVmode->width, pVmode->height, "GUI Window", pMonitor, NULL);
		}
	}
	else {
		pwindow = glfwCreateWindow(gWindowWidth, gWindowHeight, "Render Window", NULL, NULL);
		GUIwindow = glfwCreateWindow(guiWindowWidth, guiWindowHeight, "GUI Window", NULL, NULL);
	}
		

	if (pwindow == NULL) {
		std::cout << "Window Creation Failed";
		glfwTerminate();
		return false;
	}

	if (GUIwindow == NULL) {
		std::cout << "Window Creation Failed";
		glfwTerminate();
		return false;
	}

	glfwMakeContextCurrent(pwindow);
	glfwSetKeyCallback(pwindow, onKeyPress);
	glfwSetCursorPosCallback(pwindow, glfw_onMouseMove);

	glewExperimental = GL_TRUE;
	if (glewInit() != GLEW_OK) {
		std::cout << "Glew initialization failed";
		return false;
	}

	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glViewport(0, 0, gWindowWidth, gWindowHeight);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
	glEnable(GL_CULL_FACE);
	glCullFace(GL_FRONT);


	// Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGui::StyleColorsDark();
	ImGui_ImplGlfw_InitForOpenGL(GUIwindow, true);
	ImGui_ImplOpenGL3_Init("#version 430 core");


	return true;
}

void calculateDistance(glm::vec3 ballCenter, double distance[]) {
	
	for (int i = 0; i < 6; i++) {
		distance[i] = glm::dot(ballCenter - positionsCube[i], normalsCube[i]); 
		if (distance[i] < 0) {
			distance[i] += sphereRadius;
		}
		else {
			distance[i] -= sphereRadius;
		}
	}
	
}


bool detectCollision(glm::vec3 ballCenter, double newDistance[], double oldDistance[], int &colliderPlaneNormal) {

	bool check = false;
	
	double fracDistance[6];
	for (int i = 0; i < 6; i++) {

		fracDistance[i] = 1.0f;

		if (oldDistance[i] * newDistance[i] < 0) {
			fracDistance[i] = oldDistance[i] / (oldDistance[i] - newDistance[i]);
			check = true;
			//ballPosition = s.position;
		}

	}

	//find minimum of fracDistanceArray
	double minimum = fracDistance[0];
	colliderPlaneNormal = 0;
	for (int i = 1; i < 6; i++) {
		if (fracDistance[i] <= minimum) {
			minimum = fracDistance[i];
			colliderPlaneNormal = i;
		}
	}

	fractionTimeStep = minimum;

	return check;

}

void onKeyPress(GLFWwindow* window, int key, int scancode, int action, int mode) {
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window,GL_TRUE);

	if (key == GLFW_KEY_W && action == GLFW_PRESS) {
		gWireframe = !gWireframe;
		if (gWireframe) {
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		}
		else {
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		}
	}

	if (key == GLFW_KEY_S && action == GLFW_PRESS) {
		simulation = true;
	}

}

void glfw_onMouseMove(GLFWwindow* window, double posX, double posY)
{
	static glm::vec2 lastMousePos = glm::vec2(0, 0);

	if (glfwGetMouseButton(pwindow, GLFW_MOUSE_BUTTON_LEFT) == 1) {
		//std::cout << "here  " << posX << "--" << posY << std::endl;
		gYaw -= ((float)posX - lastMousePos.x) * MOUSE_SENSITIVITY;
		gPitch += ((float)posY - lastMousePos.y) * MOUSE_SENSITIVITY;
		//std::cout << "here  " << gYaw << "--" << gPitch << std::endl;
	}

	if (glfwGetMouseButton(pwindow, GLFW_MOUSE_BUTTON_RIGHT) == 1) {
		float dx = 10.0f * ((float)posX - lastMousePos.x);
		float dy = 10.0f * ((float)posY - lastMousePos.y);

		//std::cout << "here  " << dx << "--" << dy << std::endl;

		
	}

	lastMousePos.x = (float)posX;
	lastMousePos.y = (float)posY;

}

void glfw_OnFrameBufferSize(GLFWwindow* window, int width, int height)
{
	gWindowWidth = width;
	gWindowHeight = height;
	glViewport(0, 0, gWindowWidth, gWindowHeight);
}