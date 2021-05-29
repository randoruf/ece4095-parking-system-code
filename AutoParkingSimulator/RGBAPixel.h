#pragma once
class RGBAPixel
{
public:
	unsigned char r, g, b, a; // [0, 255]  
	RGBAPixel(); 
	RGBAPixel(unsigned char red, unsigned char green, unsigned char blue);
	RGBAPixel(unsigned char red, unsigned char green, unsigned char blue, unsigned char alpha);
};

