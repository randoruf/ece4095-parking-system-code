#include <iostream>
#include <cmath>
#include <vector>
#include "ParkingLot.h"
#include "Simulator.h"
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/planners/rrt/RRT.h>
#include "Visualization.h"



Visualization::Car::Car(SDL_Renderer* gRend, Agent& agent, double iWidth, double iLength, double iRearAxleBackDistance, double iMapResolution, int iMapHeight, 
	const unsigned color[3]) : agentInfo(agent)
{

	carRearAxleBackDistanceMeter = iRearAxleBackDistance;
	carWidthMeter = iWidth;
	mapHeight = iMapHeight;

	// the resolution of the map 
	mapResolution = iMapResolution;

	// the color is black by default.
	carColorR = color[0];
	carColorG = color[1]; 
	carColorB = color[2];

	/*************** the tire texture **********************/
	tireWidth = 5;
	tireHeight = 18;
	tireTexture = SDL_CreateTexture(
		gRend,
		SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_TARGET,
		tireWidth, tireHeight
	);
	// set the tire texture as render target 
	SDL_SetRenderTarget(gRend, tireTexture);
	SDL_SetRenderDrawColor(gRend, carColorR, carColorG, carColorB, 0xFF);
	tireRect = {
		0, 0,
		tireWidth, tireHeight
	};
	SDL_RenderFillRect(gRend, &tireRect);
	// reset to the main renderer after rendering 
	SDL_SetRenderTarget(gRend, NULL);


	/******************goal texture ********************/


	/***************** the car texture *******************/ 
	carTireSpace = tireWidth * 3;
	carWidth = static_cast<int>(iWidth / mapResolution);
	carHeight = static_cast<int>(iLength / mapResolution);
	carCenter = {
		static_cast<int>(carRearAxleBackDistanceMeter / mapResolution),
		static_cast<int>(carWidth / 2) + carTireSpace
	};

	carTexture = SDL_CreateTexture(
		gRend,
		SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_TARGET,
		carHeight, static_cast<int>(carWidth + carTireSpace * 2)
	);

	// color blending multiplication ( white * white = white, but if color * white = color)... 
	SDL_SetTextureBlendMode(carTexture, SDL_BLENDMODE_MOD);
	// the vehicle's bounding box and the arrow base...
	vehicleRect = { 0, carTireSpace, carHeight, carWidth };
	// the arrow at the vehicle 
	arrowBaseRect = {
			static_cast<int>(carHeight / 4), carTireSpace + static_cast<int>(carWidth / 2) - 3,
			static_cast<int>(carHeight / 2), 6 };
	// four wheels's location. 
	leftRearTireRect = {
		static_cast<int>(carHeight / 12), carTireSpace + static_cast<int>(carWidth / 10),
		tireHeight, tireWidth
	};
	rightRearTireRect = {
		static_cast<int>(carHeight / 12), carTireSpace + static_cast<int>(4 * carWidth / 5),
		tireHeight, tireWidth
	};

	leftFrontTireRect = {
		static_cast<int>(3 * carHeight / 4), carTireSpace + static_cast<int>(carWidth / 10),
		tireHeight, tireWidth
	};
	rightFrontTireRect = {
		static_cast<int>(3 * carHeight / 4), carTireSpace + static_cast<int>(4 * carWidth / 5),
		tireHeight, tireWidth
	};

	// the rotation centre of the wheel. 
	frontTireCenter = {
		static_cast<int>(tireHeight / 2), static_cast<int>(tireWidth / 2),
	};


}; 

Visualization::Car::~Car() 
{
	// SDL_DestroyTexture(tireTexture);
	// SDL_DestroyTexture(carTexture);
}

void Visualization::Car::setColor(Uint8 r, Uint8 g, Uint8 b) {
	carColorR = r; 
	carColorG = g; 
	carColorB = b; 
}


void Visualization::Car::showGoal(SDL_Renderer* gRend){
	circleRGBA(gRend, 
		static_cast<int>((agentInfo.goalPose.getPoseX()) / mapResolution),
		mapHeight - static_cast<int>((agentInfo.goalPose.getPoseY()) / mapResolution),
		20,
		carColorR, carColorG, carColorB, 255
	);
}


void Visualization::Car::showTrajectory(SDL_Renderer* gRend) {

	if (agentInfo.carPathPtr) {
		// iterate the points in the car Path.....
		for (auto p = agentInfo.carPathPtr->getStates().begin(); p != agentInfo.carPathPtr->getStates().end(); p++) {
			int x = (int)std::ceil((*p)->as<ompl::base::SE2StateSpace::StateType>()->getX() / mapResolution);
			int y = (int)std::ceil((*p)->as<ompl::base::SE2StateSpace::StateType>()->getY() / mapResolution);
			filledCircleRGBA(gRend, x, mapHeight - y, 3,
				carColorR, carColorG, carColorB, 255
			);

			// std::cout << x << " " << y << std::endl; 
		}
	}
	// else {
		// std::cout << "No solution path to show..." << std::endl; 
	// }
}


void Visualization::Car::update(SDL_Renderer* gRend, double iX, double iY, double iTheta, double iSteer) {
	// vehicle state 
	x = iX;
	y = iY; 
	theta = iTheta; 
	steer = iSteer;


	// set the render target as carTexture 
	SDL_SetRenderTarget(gRend, carTexture);
	// clean the vehicle texture 
	SDL_SetRenderDrawColor(gRend, 0xFF, 0xFF, 0xFF, 0xFF);
	// SDL_SetRenderDrawColor(gRend, 0, 0, 0, 0xFF);
	SDL_RenderClear(gRend);
	// draw the vehicle bounding box and the arrow on the carTexture 
	SDL_SetRenderDrawColor(gRend, carColorR, carColorG, carColorB, 0xFF);
	SDL_RenderDrawRect(gRend, &vehicleRect);
	SDL_RenderFillRect(gRend, &arrowBaseRect);
	filledTrigonRGBA(gRend,
		static_cast<int>(7 * carHeight / 8), carTireSpace + static_cast<int>(carWidth / 2),
		static_cast<int>(3 * carHeight / 4), carTireSpace + static_cast<int>(carWidth / 2) - 10, 
		static_cast<int>(3 * carHeight / 4), carTireSpace + static_cast<int>(carWidth / 2) + 10,
		carColorR, carColorG, carColorB,
		0xFF
	 );
	// copy the tire to the vehicle texture 
	SDL_RenderCopy(gRend, tireTexture, NULL, &leftRearTireRect);
	SDL_RenderCopy(gRend, tireTexture, NULL, &rightRearTireRect);
	// copy the front tire to the vechile texture 
	SDL_RenderCopyEx(gRend, tireTexture, NULL, &leftFrontTireRect, -(steer / M_PI * 180), &frontTireCenter, SDL_FLIP_NONE);
	SDL_RenderCopyEx(gRend, tireTexture, NULL, &rightFrontTireRect, -(steer / M_PI * 180), &frontTireCenter, SDL_FLIP_NONE);
	// set the render as the main render target
	SDL_SetRenderTarget(gRend, NULL);


	// copy the vehicle texture to the renderer....
	// std::cout << mapHeight << std::endl;
	SDL_Rect vehiclePosRect = {
		static_cast<int>((x - carRearAxleBackDistanceMeter)/mapResolution),
		mapHeight - static_cast<int>((y)/mapResolution) - carTireSpace - static_cast<int>(carWidth/2),
		carHeight, 
		static_cast<int>(carWidth + carTireSpace * 2)
	};

	SDL_RenderCopyEx(gRend, carTexture, NULL, &vehiclePosRect, -(theta/M_PI *180), &carCenter, SDL_FLIP_NONE);


}



Visualization::Visualizator::Visualizator(ParkingLot& lot) : lot_(lot)
{
	if (lot.agents.size() == 0) {
		std::cout << "\033[31m ERROR - You must add some agents to the parking lot.... \033[0m \n";
		exit(1);
	}

	// SDL library initialization 
	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) < 0) {
		std::cout << "SDL could not initialize! SDL Error:" << SDL_GetError() << std::endl;
		exit(1);
	}

	if (TTF_Init() < 0) {
		std::cout << "SDL_ttf library can not initialize! SDL TTF Error: " << TTF_GetError() << std::endl;
		exit(2);
	}

	gFont_ = TTF_OpenFont("./font/Roboto-Regular.ttf", 22);
	if (gFont_ == NULL) {
		std::cout << " Failed to load myFont font ! SDL_ttf Error : " << TTF_GetError() << std::endl;
		exit(2);
	}

	middleFont_ = TTF_OpenFont("./font/Roboto-Regular.ttf", 16);
	if (middleFont_ == NULL) {
		std::cout << " Failed to load myFont font ! SDL_ttf Error : " << TTF_GetError() << std::endl;
		exit(2);
	}


	// load the image
	parkingLotSurface_ = IMG_Load(lot.map.getMapImageName().c_str());
	if (!parkingLotSurface_) {
		std::cout << "SDL_image could not load the parking lot image" << IMG_GetError() << std::endl;
		SDL_Quit();
		exit(1);
	}


	// create a window based on the image 
	const int SCREEN_WIDTH = parkingLotSurface_->w + destInfoWidth_;
	const int SCREEN_HEIGHT = parkingLotSurface_->h;
	gWindow_ = SDL_CreateWindow("Parking Lot Simulator",
		SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
		SCREEN_WIDTH, SCREEN_HEIGHT,
		SDL_WINDOW_SHOWN);
	if (!gWindow_) {
		std::cout << "SDL could not create a window" << SDL_GetError() << std::endl;
		SDL_FreeSurface(parkingLotSurface_);
		SDL_Quit();
		exit(1);
	}

	// create a render and setup the graphics hardware 
	Uint32 render_flags = SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC;
	gRend_ = SDL_CreateRenderer(gWindow_, -1, render_flags);  // we don't care about the driver, so -1
	if (!gRend_) {
		std::cout << "Error creating renderer" << SDL_GetError() << std::endl;
		SDL_FreeSurface(parkingLotSurface_);
		SDL_DestroyWindow(gWindow_);
		SDL_Quit();
		exit(1);
	}

	// load the image data into the graphics hardware memory (to a renderer)
	// (hence, the image will be "texture" instead of "surface")
	parkingLotTex_ = SDL_CreateTextureFromSurface(gRend_, parkingLotSurface_); // GPU memory
	if (!parkingLotTex_) {
		std::cout << "SDL could not create texture!" << SDL_GetError() << std::endl;
		SDL_FreeSurface(parkingLotSurface_);
		SDL_DestroyRenderer(gRend_);
		SDL_DestroyWindow(gWindow_);
		SDL_Quit();
		exit(1);
	}

	// clear the window (clear the GPU render)
	SDL_RenderClear(gRend_);

	// copy the texture to the render
	SDL_RenderCopy(gRend_, parkingLotTex_, NULL, NULL);
	SDL_RenderPresent(gRend_);	// show the renderer.... 

	// create a carList to maintain a list of agents...
	for (size_t i = 0; i < lot.agents.size(); i++) {
		carList_.push_back(Car(
			gRend_, 
			lot_.agents[i], lot.vehcicleInfo.carWidth, lot.vehcicleInfo.carLength,
			lot.vehcicleInfo.carRearAxleBackEndDistance,
			lot.map.getMapResolution(), lot.map.getHeight(), lot.agents[i].color));
	}

	// viewPort 
	destParkingLotViewPort_ = {
		0, 0, 
		parkingLotSurface_->w, 
		parkingLotSurface_->h
	};

	destInfoViewPort_ = {
		parkingLotSurface_->w, 0, 
		destInfoWidth_, 
		parkingLotSurface_->h
	};

	// set a viewport 
	// SDL_RenderSetViewport(gRend, &destParkingLotViewPort);
}


Visualization::Visualizator::~Visualizator()
{
	SDL_FreeSurface(parkingLotSurface_);
	parkingLotSurface_ = NULL;
	// close the font file... 
	TTF_CloseFont(gFont_);
	gFont_ = NULL;
	TTF_CloseFont(middleFont_);
	middleFont_ = NULL;
	// clean up 
	SDL_DestroyRenderer(gRend_);
	gRend_ = NULL;
	SDL_DestroyWindow(gWindow_);
	gWindow_ = NULL; 
	// Quit SDL subsystems
	IMG_Quit();
	TTF_Quit();
	SDL_Quit();
}


void Visualization::Visualizator::display() {
	int bx = 0, by = 0;
	SDL_Color textColor = { 255, 255, 100 };

	// show the FPS
	std::string FPSLabelText("FPS ");
	UILabel FPSLabel(gRend_, gFont_, bx + 10, by + 10, 1, 1);

	// the stop/play button 
	by = 60;
	bool rendering = true;
	UIButton stopButton(gRend_, bx, by, 200, 50);
	std::string stopButtonText("Rendering(Y/N)");
	// the stop/play button texture 
	UILabel stopButtonLabel(gRend_, gFont_, bx + 10, by + 10, 1, 1);
	stopButtonLabel.setLabelText(stopButtonText, textColor);

	
	// increase simulation speed
	by = 110; 
	UIButton increaseButton(gRend_, bx, by, 200, 50);
	std::string increaseButtonText("Speed(+)");
	UILabel increaseButtonLabel(gRend_, gFont_, bx + 10, by + 10, 1, 1);
	increaseButtonLabel.setLabelText(increaseButtonText, textColor);


	// increase simulation speed
	by = 160;
	UIButton decreaseButton(gRend_, bx, by, 200, 50);
	std::string decreaseButtonText("Speed(-)");
	UILabel decreaseButtonLabel(gRend_, gFont_, bx + 10, by + 10, 1, 1);
	decreaseButtonLabel.setLabelText(decreaseButtonText, textColor);


	// show time elapsed
	by = 210;
	int timeElapsed = 0;
	std::string timeElapsedLabelText("Time Elapsed ");
	UILabel timeElapsedLabel(gRend_, middleFont_, bx + 10, by + 10, 1, 1);


	// update the window..... 
	bool quit = false;
	SDL_Event e;
	int x, y;
	Uint32 lastMoment = SDL_GetTicks();
	Uint32 startTime = SDL_GetTicks();

	while (!quit)
	{	
		// interation with the user 
		while (SDL_PollEvent(&e))
		{
			if (e.type == SDL_QUIT) {
				quit = true;
			}
			if (e.type == SDL_MOUSEBUTTONDOWN) {
				SDL_GetMouseState(&x, &y);
				if (x > lot_.map.getWidth()) {
					x -= lot_.map.getWidth();
					// check if it is the stop button 
					if (stopButton.hitUIRect(x, y)) {
						stopButton.status = UIButton::UI_CLICKED;
						rendering = !rendering;
					}
					// check if it is the speed decreasement button
					if (increaseButton.hitUIRect(x, y)) {
						increaseButton.status = UIButton::UI_CLICKED;
						if (FPS < 60) {
							FPS = static_cast<int>(FPS + 10); 
							if (FPS > 60) FPS = 60;
						}
					}
					// check if it is the speed decreasement button
					if (decreaseButton.hitUIRect(x, y)) {
						decreaseButton.status = UIButton::UI_CLICKED;
						if (FPS > 1) {
							FPS = static_cast<int>(FPS - 10);
							if (FPS < 1) FPS = 1;
						}
					}


				}
			}
			if (e.type == SDL_MOUSEBUTTONUP) {
				SDL_GetMouseState(&x, &y);
				if (x > lot_.map.getWidth()) {
					x -= lot_.map.getWidth();
					// releaase the stop button...
					if (stopButton.hitUIRect(x, y)) {
						stopButton.status = UIButton::UI_RELEASED;
					}
					// release the speed decreasement button...
					if (increaseButton.hitUIRect(x, y)) {
						increaseButton.status = UIButton::UI_RELEASED;
					}
					// release the speed decreasement button...
					if (decreaseButton.hitUIRect(x, y)) {
						decreaseButton.status = UIButton::UI_RELEASED;
					}

				}
			}
		}

		// clear the whole screen....
		SDL_SetRenderDrawColor(gRend_, 0, 0, 0, 0xFF);
		SDL_RenderClear(gRend_);

		/********************************************************************************************************************/
		SDL_RenderSetViewport(gRend_, &destParkingLotViewPort_);
		// Copy the texture to the render (parking lot)
		SDL_RenderCopyEx(gRend_, parkingLotTex_, NULL, NULL, 0, NULL, SDL_FLIP_VERTICAL);
		/********************************************************************************************************************/
		
		// 
		lot_.updateScene();

		/********************************************************************************************************************/
		// Update the screen
		if (rendering) {
			for (size_t i = 0; i < carList_.size(); i++) {
				Car& currCar = carList_[i];

				// the car's current pose
				currCar.update(
					gRend_,
					currCar.agentInfo.currPose.getPoseX(),
					currCar.agentInfo.currPose.getPoseY(),
					currCar.agentInfo.currPose.getPoseTheta(),
					currCar.agentInfo.currPose.getPoseSteer());

				// the car's goal 
				currCar.showGoal(gRend_); 

				// the car's trajactory 
				currCar.showTrajectory(gRend_);

			}


		}


		/********************************************************************************************************************/
		// render the viewport of the info panel
		SDL_RenderSetViewport(gRend_, &destInfoViewPort_);
		// show the FPS 
		FPSLabelText = "FPS " + std::to_string(FPS);;
		FPSLabel.setLabelText(FPSLabelText, textColor);
		FPSLabel.display();

		// the stop button 
		stopButton.display();
		stopButtonLabel.display();
		// the speed increasement button 
		increaseButton.display();
		increaseButtonLabel.display();
		// the speed increasement button 
		decreaseButton.display();
		decreaseButtonLabel.display();

		// 
		timeElapsedLabelText = "Time Elapsed " + std::to_string(timeElapsed) + " S";
		timeElapsedLabel.setLabelText(timeElapsedLabelText, textColor);
		timeElapsedLabel.display();


		/*******************************************************************************************************************/

		//Update screen
		SDL_RenderPresent(gRend_);

		// update the elapsed time 
		timeElapsed = static_cast<int>((SDL_GetTicks() - startTime) / 1000); // Convert to seconds.

		// each frame takes equal duration.  
		Uint32 currMoment = SDL_GetTicks();
		Uint32 deltaTime = currMoment - lastMoment;
		lastMoment = currMoment;

		// let the loop run exactly one second....
		if (deltaTime < 1000 / FPS) {
			SDL_Delay(1000 / FPS - deltaTime);
		}
	}
};