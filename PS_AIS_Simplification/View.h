#pragma once

/// Init the OpenGL
void init(void);

/// Display the Object
void display(void);

/// Reshape the Window
void reshape(int w, int h);

/// Mouse Messenge
void mouse(int button, int state, int x, int y);

/// Motion Function
void motion(int x, int y);

/// Keyboard Messenge
void keyboard(unsigned char key, int x, int y);

/// Idle Function
void idle(void);
