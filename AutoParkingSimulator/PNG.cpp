#pragma once
#include <string>
#include <vector>
#include <iostream>
#include <cassert>
#include "PNG.h"
#include "lodepng/lodepng.h"

PNG::PNG() {
	width_ = 0;
	height_ = 0; 
	size_ = 0; 
	imageData_ = NULL; 
}

PNG::PNG(unsigned int width, unsigned int height) {
	width_ = width; 
	height_ = height; 
	size_ = width * height;
	imageData_ = new RGBAPixel[size_];
}

PNG::~PNG() {
	delete[] imageData_;  // clean "new" ImageData 
}

unsigned int PNG::getWidth() const{
	return width_; 
}

unsigned int PNG::getHeight() const {
	return height_; 
}


RGBAPixel& PNG::getPixel(unsigned int i, unsigned int j) const {
	if (width_ == 0 || height_ == 0) {
		std::cerr << "ERROR: " << "\033[31m" << "Call to PNG::getPixel() made on an image with no pixels." << "\033[0m" << std::endl; 
		assert(width_ > 0);
		assert(height_ > 0);
	}

	// The Cartesian coordinate...
	if (i >= width_) {
		std::cerr << "ERROR: " << "\033[31m" << "Call to PNG::getPixel(" << i << ", " << j << ") tries to access x which is outside of the image (x < "
			<< width_ << ")"  << "\033[0m" << std::endl;
		assert(i < width_); 
	}

	if (j >= height_) {
		std::cerr << "ERROR: " << "\033[31m" << "Call to PNG::getPixel(" << i << ", " << i << ") tries to access y which is outside of the image (y < "
			<< height_ << ")" << "\033[0m" << std::endl;
		assert(j < height_); 
	}

	unsigned int index = i + (j * width_); 
	return imageData_[index]; 
}

bool PNG::readFromFile(std::string const& fileName) {
	std::vector<unsigned char> byteData; 
	// note that `byteData`, `width_` and `height_` may be changed (see `lodepng.cpp` line 6634). 
	unsigned error = lodepng::decode(byteData, width_, height_, fileName);

	if (error) {
		std::cerr << "ERROR: " << "\033[31m" << "PNG decoder error" << error << ": " << lodepng_error_text(error) << "\033[0m" << std::endl;
		return false;
	}

	delete[] imageData_; 
	size_ = width_ * height_;
	imageData_ = new RGBAPixel[size_]; 

	for (unsigned int i = 0; i < byteData.size(); i += 4) {
		RGBAPixel& pixel = imageData_[i / 4];
		pixel.r = byteData[i];					//  [0, 255]
		pixel.g = byteData[i + 1]; 
		pixel.b = byteData[i + 2]; 
		pixel.a = byteData[i + 3]; 
	}

	return true; 
}


bool PNG::writeToFile(std::string const& fileName) {
	unsigned char* byteData = new unsigned char[width_ * height_ * 4]; 

	for (unsigned i = 0; i < width_ * height_; i++) {
		byteData[(i * 4)] = imageData_[i].r;
		byteData[(i * 4) + 1] = imageData_[i].g;
		byteData[(i * 4) + 2] = imageData_[i].b;
		byteData[(i * 4) + 3] = imageData_[i].a;
	}

	
	unsigned error = lodepng::encode(fileName, byteData, width_, height_); 
	if (error) {
		std::cerr << "ERROR: " << "\033[31m" << "PNG encoding error :" << lodepng_error_text(error) << "\033[0m" << std::endl;
	}

	delete[] byteData;	// clean the new operation. 
	return (error == 0); 
}