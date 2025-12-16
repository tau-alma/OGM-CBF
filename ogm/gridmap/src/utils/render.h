/**
 * @file render.h
 * @author Teemu Mökkönen (teemu.mokkonen@tuni.com)
 * @brief Utilities for visualizing points cloud and gridmaps
 * @version 0.1
 * @date 2024-02-26
 * 
 * @copyright Copyright (c) 2024
 * 
 */


#include <SDL2/SDL.h>
#include "datatypes.h"

/**
 * @brief Render one intance of the pointcloud
 * @param _renderer pointer to SDL_Renderer fpr rendering the image
 * @param numPoints how many points are in the array of points
 * @param pointer to pointArray, which contains the points for rendering
 */
void render_cloud(SDL_Renderer* _renderer, int numPoints, LidarData *pointArray ) {



    // Clear the renderer
    SDL_SetRenderDrawColor(_renderer, 0, 0, 0, 255);
    SDL_RenderClear(_renderer);

    // Draw points from pointArray with origin at the middle of the window
    for (int i = 0; i < numPoints; i++) {
        // Move the point by the half of the window size to reposition it
        int x = round((pointArray[i].x * 4) + 1000 / 2);
        int y = round((pointArray[i].y * 4) + 1000 / 2);
        int intensity = (int)(pointArray[i].intensity * 255);
        int colorValue = 255; //- intensity; // Invert the intensity value for higher intensity to be brighter
        SDL_SetRenderDrawColor(_renderer, intensity, 0, 0, 255);
        SDL_RenderDrawPoint(_renderer, x, y);
    }

    // Present the renderer
    SDL_RenderPresent(_renderer);

    // Cleanup and exit
}

/**
 * @brief Render one intance of the pointcloud
 * @param _renderer pointer to SDL_Renderer fpr rendering the image
 * @param xw grid width (x)
 * @param yw drig height (y)
 * @param cellSize Size of one cell to be
 * @param gridmap Float array containing the grids to render
 * @param pose_x pose of the machine
 * @param pose_y pose of the machine
*/
void render_gridmap(SDL_Renderer* _renderer, int xw, int yw, int cellSize, float** gridmap, int pose_x, int pose_y) {
    // Clear the renderer
    SDL_SetRenderDrawColor(_renderer, 0, 0, 0, 255);
    SDL_RenderClear(_renderer);

    // Render the gridmap
    for (int x = 0; x < xw; x++) {
        for (int y = 0; y < yw; y++) {
            //double value = gridmap[x * yw + y];
            float val = 1.0 - 1/exp(1 + gridmap[x][y]);
            float scaledValue = (float)(val * 255);

            //printf("scaled %f", scaledValue);
            //printf("scaled values is %f \n", scaledValue);
            SDL_SetRenderDrawColor(_renderer, scaledValue, scaledValue, scaledValue, 255);
            SDL_Rect cell = { x * cellSize, y * cellSize, cellSize, cellSize };
            SDL_RenderFillRect(_renderer, &cell);
        }

        // render ego pose
        SDL_SetRenderDrawColor(_renderer, 155, 0, 0 , 255);
        SDL_Rect cell = { pose_x * cellSize, pose_y * cellSize, cellSize, cellSize };
        SDL_RenderFillRect(_renderer, &cell);
    }

    float scaledValue = 20;
    SDL_SetRenderDrawColor(_renderer, scaledValue, scaledValue, scaledValue, 255);
    SDL_Rect cell = { pose_x *2 , pose_y *2 , 4, 4 };
    SDL_RenderFillRect(_renderer, &cell);

    // Present the renderer
    SDL_RenderPresent(_renderer);
}


/**
 * @brief Render one intance of the pointcloud
 * @param _renderer pointer to SDL_Renderer fpr rendering the image
 * @param xw grid width (x)
 * @param yw drig height (y)
 * @param cellSize Size of one cell to be
 * @param gridmap Float array containing the grids to render
 * @param pose_x pose of the machine
 * @param pose_y pose of the machine
 * @param numPoints how many points are in the array of points
 * @param pointer to pointArray, which contains the points for rendering
*/
void render_gridmap_with_lidar(SDL_Renderer* _renderer, int xw, int yw, int cellSize, float** gridmap, int pose_x, int pose_y,  int numPoints, LidarData *pointArray) {
    // Clear the renderer
    SDL_SetRenderDrawColor(_renderer, 0, 0, 0, 255);
    SDL_RenderClear(_renderer);

    // Render the gridmap
    for (int x = 0; x < xw; x++) {
        for (int y = 0; y < yw; y++) {
            //double value = gridmap[x * yw + y];
            float val = 1.0 - 1/exp(1 + gridmap[x][y]);
            float scaledValue = (float)(val * 255);

            //printf("scaled %f", scaledValue);
            //printf("scaled values is %f \n", scaledValue);
            SDL_SetRenderDrawColor(_renderer, scaledValue, scaledValue, scaledValue, 255);
            SDL_Rect cell = { x * cellSize, y * cellSize, cellSize, cellSize };
            SDL_RenderFillRect(_renderer, &cell);
        }
    }

        // Draw points from pointArray with origin at the middle of the window

    for (int i = 0; i < numPoints; i++) {
        // Move the point by the half of the window size to reposition it
        int x = (pointArray[i].x);
        int y = (pointArray[i].y);
        int center_x = (int)round((x - -25) / 0.5);
        int center_y = (int)round((y - -25) / 0.5);
        int intensity = (int)((pointArray[i].intensity) * 255);
        int colorValue = 255; //- intensity; // Invert the intensity value for higher intensity to be brighter
        //SDL_RenderDrawPoint(_renderer, center_x * cellSize, center_y *cellSize);
        SDL_SetRenderDrawColor(_renderer, 255, 0, 0, 155);
        SDL_Rect cell = { center_x * cellSize, center_y * cellSize , cellSize, cellSize};
        SDL_RenderFillRect(_renderer, &cell);
    }
    //    // render ego pose
    //SDL_SetRenderDrawColor(_renderer, 155, 0, 0 , 255);
    //SDL_Rect cell = { pose_x * cellSize, pose_y * cellSize, cellSize, cellSize };
    //SDL_RenderFillRect(_renderer, &cell);
//
    //// Present the renderer
    SDL_RenderPresent(_renderer);
}



void render_ego_machine(SDL_Renderer* _renderer, int xw, int yw, int cellSize, int x, int y) {
    // Clear the renderer
    SDL_SetRenderDrawColor(_renderer, 0, 0, 0, 255);
    SDL_RenderClear(_renderer);

    //printf("scaled %f", scaledValue);
    //printf("scaled values is %f \n", scaledValue);
    float scaledValue = 155;
    SDL_SetRenderDrawColor(_renderer, scaledValue, scaledValue, scaledValue, 255);
    SDL_Rect cell = { x * cellSize, y * cellSize, cellSize, cellSize };
    SDL_RenderFillRect(_renderer, &cell);


    // Present the renderer
    SDL_RenderPresent(_renderer);
}


/**
 * @brief Init two windows for showing the raw lidar data and the resulting map
 * @param window_name name of the window for first render
 * @param _window pointer to SDL_window instance
 * @param _renderer pointer to SDL_renderer instance
 * @param window_name_2 name of the window for second render
 * @param _window_2 pointer to SDL_window instance
 * @param _renderer_2 pointer to SDL_renderer instance
*/

void init_windows(const char* window_name, SDL_Window** _window, SDL_Renderer** _renderer, const char* window_name_2, SDL_Window** _window_2, SDL_Renderer** _renderer_2){
    // needs some way to monitor the window quit action, at the moment it does not work.
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        fprintf(stderr, "SDL_Init Error: %s\n", SDL_GetError());
        return;
    }

    *_window = SDL_CreateWindow(window_name, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1000, 1000, 0);
    if (!_window) {
        fprintf(stderr, "SDL_CreateWindow Error: %s\n", SDL_GetError());
        SDL_Quit();
        return;
    }

    *_renderer = SDL_CreateRenderer(*_window, -1, SDL_RENDERER_ACCELERATED);
    if (!_renderer) {
        fprintf(stderr, "SDL_CreateRenderer Error: %s\n", SDL_GetError());
        SDL_DestroyWindow(*_window);
        SDL_Quit();
        return;
    }


    // needs some way to monitor the window quit action, at the moment it does not work.
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        fprintf(stderr, "SDL_Init Error: %s\n", SDL_GetError());
        return;
    }

    *_window_2 = SDL_CreateWindow(window_name_2, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1000, 1000, 0);
    if (!_window) {
        fprintf(stderr, "SDL_CreateWindow Error: %s\n", SDL_GetError());
        SDL_Quit();
        return;
    }

    *_renderer_2 = SDL_CreateRenderer(*_window_2, -1, SDL_RENDERER_ACCELERATED);
    if (!_renderer_2) {
        fprintf(stderr, "SDL_CreateRenderer Error: %s\n", SDL_GetError());
        SDL_DestroyWindow(*_window_2);
        SDL_Quit();
        return;
    }

    SDL_Event event;
    int quit = 0;

    while (!quit) {
        while (SDL_PollEvent(&event)) {
            SDL_Delay(10);
            if (event.type == SDL_QUIT) {
                // SDL_QUIT event is triggered when the window is closed
                
                quit = 1;
            }
        }
    }

    // Clean up and exit
    SDL_DestroyRenderer(*_renderer);
    SDL_DestroyWindow(*_window);
    SDL_DestroyRenderer(*_renderer_2);
    SDL_DestroyWindow(*_window_2);
    SDL_Quit();

}



int threadFunction(void* data) {
    const char* window_name = "test";
    SDL_Window** _window = (SDL_Window**)((void**)data)[1];
    SDL_Renderer** _renderer = (SDL_Renderer**)((void**)data)[2];
    
    init_windows(window_name, _window, _renderer, window_name, (SDL_Window**)((void**)data)[4], (SDL_Renderer**)((void**)data)[5]);

    return 0;
}
