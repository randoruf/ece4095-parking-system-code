#pragma once
#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include <SDL2/SDL_timer.h>
#include <SDL2/SDL_ttf.h>
#include <SDL2/SDL2_gfxPrimitives.h>
#include "ParkingLot.h"

namespace Visualization {


	class UILabel {
	public:
		SDL_Rect rect;
		SDL_Renderer* rdr;
		TTF_Font* UIfont;

		// the actuaal text and its texture in the UI lable class....  
		std::string text;
		SDL_Texture* mTexture = NULL;

		UILabel(SDL_Renderer* render, TTF_Font* font, int x, int y, int w, int h) {
			rdr = render;
			UIfont = font;
			rect = { x, y, w, h };
		}

		void setTextLocation(int x, int y) {
			rect.x = x;
			rect.y = y;
		}

		// Set text for a label
		void setLabelText(const std::string &itext, SDL_Color textColor) {
			text = itext;
			aux_freeLTexture(); // free the old texture.... 
			aux_allocateTextureFromString(textColor);
		}

		// Render(display) to screen
		void display() {
			SDL_RenderCopy(rdr, mTexture, NULL, &rect);
		}

	private:

		void aux_freeLTexture() {
			if (mTexture != NULL) {
				SDL_DestroyTexture(mTexture);
				mTexture = NULL;
			}
		}


		void aux_allocateTextureFromString(SDL_Color textColor) {
			// Render text surface
			SDL_Surface* textSurface = TTF_RenderText_Solid(UIfont, text.c_str(), textColor);

			if (textSurface == NULL) {
				printf(" Unable to render text surface ! SDL_ttf Error: %s\n", TTF_GetError());
			}
			else {
				// Create texture from surface pixels
				mTexture = SDL_CreateTextureFromSurface(rdr, textSurface);
				if (mTexture == NULL) {
					printf(" Unable to create texture from rendered text !SDL Error : %s\n", SDL_GetError());
				}
				else {
					// Get image dimensions
					rect.w = textSurface->w;
					rect.h = textSurface->h;
				}
				// Get rid of old surface
				SDL_FreeSurface(textSurface);
			}
		}
	};

	class UIButton {
	public:
		// the UI Status
		enum UIStatus {
			UI_CLICKED,
			UI_RELEASED,
		};

		int status;
		SDL_Rect rect;
		SDL_Renderer *rdr;

		UIButton(SDL_Renderer *iRend, int x, int y, int w, int h) : rdr(iRend) {
			rect = { x, y, w, h };
			status = UI_RELEASED;
		}

		int hitUIRect(int x, int y) {
			return (x >= rect.x && x <= rect.x + rect.w && y >= rect.y && y <= rect.y + rect.h);
		}

		void display() {
			aux_buttonOnDraw();
		}


	private:
		// thick -- the size of the bump
		// dark -- dark intensity
		// bright -- bright intensity
		void aux_drawBumpRect(Sint16 x1, Sint16 y1, Sint16 x2, Sint16 y2, Sint16 thick, Uint8 dark, Uint8 bright)
		{
			Sint16 k;
			for (k = 0; k < thick; k++)
			{
				SDL_SetRenderDrawColor(rdr, dark, dark, dark, 0xFF);
				SDL_RenderDrawLine(rdr, x1 + k, y1 + k, x2 - k, y1 + k);
				SDL_RenderDrawLine(rdr, x1 + k, y1 + k, x1 + k, y2 - k);
				SDL_SetRenderDrawColor(rdr, bright, bright, bright, 0xFF);
				SDL_RenderDrawLine(rdr, x1 + k, y2 - k, x2 - k, y2 - k);
				SDL_RenderDrawLine(rdr, x2 - k, y1 + k, x2 - k, y2 - k);
			}
		}

		// draw button
		int aux_buttonOnDraw()
		{
			int bright = 200, dark = 100, grey = (bright + dark) / 2;
			int thick = rect.h / 10;

			SDL_SetRenderDrawColor(rdr, grey, grey, grey, 0xFF);
			SDL_RenderFillRect(rdr, &rect);
			if (status == UI_CLICKED)
			{
				aux_drawBumpRect(rect.x, rect.y, rect.x + rect.w, rect.y + rect.h, thick, dark, bright);
			}
			else if (status == UI_RELEASED)
			{
				aux_drawBumpRect(rect.x, rect.y, rect.x + rect.w, rect.y + rect.h, thick, bright, dark);
			}
			return 1;
		}


	};

	class Car
	{
	public:
		
		Car(SDL_Renderer* gRend, Agent& agent, double iWidth, double iHeight, double iRearAxleBackDistance, double iMapResoluion, int iMapHeight, 
			const unsigned color[3]);
		~Car();
		void update(SDL_Renderer* gRend, double x, double y, double theta, double steerAngle);
		void showGoal(SDL_Renderer* gRend); 
		void showTrajectory(SDL_Renderer* gRend);

		// 
		void setColor(Uint8 r, Uint8 g, Uint8 b);

		/************ Vehicle State ************/
		Agent& agentInfo;
		double x, y, theta, steer; 

		// the color of the car 
		Uint8 carColorR, carColorG,  carColorB;
		double mapResolution;
		int mapHeight; 

		/***********  Tire Material  ****************/
		// the tire texture 
		int tireWidth;
		int tireHeight;
		int carTireSpace;
		// the trie shape (a rectangle on the (0, 0) origin)
		SDL_Rect tireRect; 
		// the rear tire location 
		SDL_Rect leftRearTireRect;
		SDL_Rect rightRearTireRect;
		// the front tire location 
		SDL_Rect leftFrontTireRect;
		SDL_Rect rightFrontTireRect;
		SDL_Point frontTireCenter;
		// the tire Texture (danger)
		SDL_Texture* tireTexture;

		/***********  Vehicle Material  *************/
		int carWidth, carHeight;
		double carWidthMeter, carRearAxleBackDistanceMeter; 
		SDL_Point carCenter;
		SDL_Rect arrowBaseRect;
		SDL_Rect vehicleRect;
		// the vehicle texture (danger)
		SDL_Texture* carTexture;
	};


	class Visualizator 
	{
	public:
		Visualizator(ParkingLot &lot);
		~Visualizator();
		void display();

	private:
		unsigned int FPS = 60;
		ParkingLot& lot_;
		SDL_Surface* parkingLotSurface_;
		SDL_Texture* parkingLotTex_;
		SDL_Rect destParkingLotViewPort_;
		SDL_Rect destInfoViewPort_;
		std::vector<Car> carList_;
		SDL_Window* gWindow_;
		SDL_Renderer* gRend_;
		TTF_Font* gFont_;
		TTF_Font* middleFont_;
		const int destInfoWidth_ = 200; 
	};





}