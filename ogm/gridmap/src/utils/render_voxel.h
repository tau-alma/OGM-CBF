/**
 * @file render_voxel.h
 * @author Teemu Mökkönen (teemu.mokkonen@tuni.com)
 * @brief This file contains utilities for plotting voxel grid using The OpenGL Utility Toolkit
 * @version 0.1
 * @date 2024-02-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <GL/glut.h>
#include <stdio.h>
#include "math.h"    

float *** voxelGrid;
int size_x;
int size_y;
int size_z;

double camera_x = 50.0;
double camera_y = 50.0;
double camera_z = 100.0;

/**
 * @brief Display the saved instance of the voxel grid saved in to the global variables, this function is called automatically from the init loop
 * 
 */
void display() {
    
    // Clear the color and depth buffers
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Reset transformations
    glLoadIdentity();

   // Set the camera for top-down view
    gluLookAt(0.0, 0.0, 80.0, // eye position (x, y, z)
              0.0, 0.0, 0.0,  // look-at position (x, y, z)
              0.0, 1.0, 0.0); // up vector (x, y, z)

    // Rotate the grid to align with the top-down view
    glRotatef(-90, 1.0, 0.0, 0.0);


    
    // Draw the voxel grid
    for (int x = 0; x < size_x; x++) {
        for (int y = 0; y < size_y; y++) {
            for (int z = 0; z < size_z; z++) {
                float val = 1.0 - 1/exp(1 + voxelGrid[x][y][z]);
                if (0.4 > val && val > 0.2) {
                    glPushMatrix();
                    glColor4f(0.0, 1.0, 1.0, 0.2); // uncertain region
                    glTranslatef(x - size_x / 2.0, y - size_y / 2.0, z - size_z / 2.0);
                    glutSolidCube(0.5); // Draw a solid cube
                    glPopMatrix();
                }
                if (val <= 0.2) {
                    glPushMatrix();
                    if (z < 3) {
                        glColor4f(1.0, 0.0, 0.0, 0.5); // Red floor
                    }
                    else {
                        glColor4f(0.0, 0.0, 1.0, 0.5); // blue obs with 50% transparency
                    }
                    glTranslatef(x - size_x / 2.0, y - size_y / 2.0, z - size_z / 2.0);
                    glutSolidCube(0.5); // Draw a solid cube
                    glPopMatrix();
                }
                
            }
        }
    }

    // Flush buffer
    glFlush();
    // Swap the front and back buffers to display the rendered image
    glutSwapBuffers();
}


/**
 * @brief Function for listening the keyboard events for moving the viewpoint, GLUT is viewing this function, no need for user to use this.
 */
void keyboard(unsigned char key, int x, int y) {
    printf("got keyboard event");
    switch (key) {
        case 'w':
        case 'W':
            camera_z -= 1.0;
            break;
        case 's':
        case 'S':
            camera_z += 1.0;
            break;
        case 'a':
        case 'A':
            camera_x -= 1.0;
            break;
        case 'd':
        case 'D':
            camera_x += 1.0;
            break;
    }

    glutPostRedisplay();
}

/**
 * @brief Used for initializing the rendering loop for the voxel grid. 
 */
void render_init(int argc, char** argv) {
    // Initialize GLUT and create the window
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(1200, 1200);
    glutCreateWindow("Voxel Grid");

    // Enable depth testing for 3D
    glEnable(GL_DEPTH_TEST);

    // Set the projection matrix
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(80.0, 1.0, 0.1, 200.0);

    // Set the modelview matrix
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Set the clear color (background color)
    glClearColor(0.0, 0.0, 0.0, 1.0);
    
    glutKeyboardFunc(keyboard);
    // Register display function
    glutDisplayFunc(display);

}
