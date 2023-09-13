#include <iostream>
#include <math.h>

float euclideanDistance(float x1, float y1, float x2, float y2){
	// TO-DO: Calculate and return the Euclidean distance between (x1,y1) and (x2,y2) 
	return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}

int main() {
	float x1, y1, x2, y2;
    std::cout << "Introduce x and y coordinates (sep. by space): ";
    std::cin >> x1 >> y1;
    std::cout << "Introduce (another) x and y coordinates (sep. by space): ";
    std::cin >> x2 >> y2;
    std::cout << "The distance between coordinates is " << euclideanDistance(x1,y1,x2,y2) << std::endl;
    return 0;
}