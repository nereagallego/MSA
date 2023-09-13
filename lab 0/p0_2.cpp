#include <iostream>
#include <math.h>

float euclideanDistance(float x1, float y1, float x2, float y2){
	// TO-DO: Calculate and return the Euclidean distance between (x1,y1) and (x2,y2) 
	return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}

float triangleAreaFromVertices(float vertex_1_x, float vertex_1_y, 
								float vertex_2_x, float vertex_2_y, 
								float vertex_3_x, float vertex_3_y){
	// TO-DO: Calculate and return the triangle area from the input vertices
	
	return (vertex_1_x*(vertex_2_y - vertex_3_y) + vertex_2_x*(vertex_3_y - vertex_1_y) + vertex_3_x*(vertex_1_y - vertex_2_y))/2;
}

int main() {
	// Now you can give the triangle the values on the code itself
	float vertex_1_x = 1.0f;
	float vertex_1_y = 0.0f;
	float vertex_2_x = 3.0f;
	float vertex_2_y = 0.0f;
	float vertex_3_x = 2.0f;
	float vertex_3_y = 2.0f;
	std::cout << "The area of the triangle is " 
				<< triangleAreaFromVertices(vertex_1_x, vertex_1_y, vertex_2_x, vertex_2_y, vertex_3_x, vertex_3_y) 
				<< std::endl;


    return 0;
}