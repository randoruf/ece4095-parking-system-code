#include "RGBAPixel.h"

RGBAPixel::RGBAPixel() {
	r = 0; 
	g = 0; 
	b = 0; 
	a = 0; 
}

RGBAPixel::RGBAPixel(unsigned char red, unsigned char green, unsigned char blue) {
	r = red; 
	g = green; 
	b = blue; 
	a = 0; 
}

RGBAPixel::RGBAPixel(unsigned char red, unsigned char green, unsigned char blue, unsigned char alpha) {
	r = red; 
	g = green; 
	b = blue; 
	a = alpha; 
}