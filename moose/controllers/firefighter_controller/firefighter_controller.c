#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/lidar.h>
#include <webots/motor.h> 
#include <webots/supervisor.h> 

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#define TIME_STEP 64
#define CAMERA_NAME "camera"
#define LIDAR_NAME "lidar"
#define LEFT_MOTOR "left wheel"
#define RIGHT_MOTOR "right wheel"
#define MAX_SPEED 3.0
#define FIRE_R 251
#define FIRE_G 72
#define FIRE_B 15
#define COLOR_TOLERANCE 50
#define FIRE_SIZE_STOP 10000
#define SAFE_DISTANCE 0.8 

typedef enum {
  SEARCHING,
  ALIGNING,
  MOVING_FAST,
  PRE_AVOID,
  AVOIDING,
  STOPPED
} RobotState;

// --- Set Motor Speed Method ---
void set_speed(WbDeviceTag left_motor, WbDeviceTag right_motor, double left, double right) {
    if (left > MAX_SPEED) left = MAX_SPEED;
    if (right > MAX_SPEED) right = MAX_SPEED;
    if (left < -MAX_SPEED) left = -MAX_SPEED;
    if (right < -MAX_SPEED) right = -MAX_SPEED;
    
    wb_motor_set_velocity(left_motor, left);
    wb_motor_set_velocity(right_motor, right);
}

// --- Fire Detection Method ---
int detect_fire(WbDeviceTag camera, double *x_center, double *y_center, int *fire_size, double current_distance) {
  const unsigned char *image = wb_camera_get_image(camera);
  if (!image) return 0;

  int width = wb_camera_get_width(camera);
  int height = wb_camera_get_height(camera);
  int fire_pixels = 0;
  double sumX = 0, sumY = 0;

  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      
      // Ignore ground if far
      if (current_distance > 1.0 && y > height * 0.60) continue; 

      int r = wb_camera_image_get_red(image, width, x, y);
      int g = wb_camera_image_get_green(image, width, x, y);
      int b = wb_camera_image_get_blue(image, width, x, y);

      // Check Fire Color
      if (r > g + 40 &&
          abs(r - FIRE_R) < COLOR_TOLERANCE &&
          abs(g - FIRE_G) < COLOR_TOLERANCE &&
          abs(b - FIRE_B) < COLOR_TOLERANCE) {
        fire_pixels++;
        sumX += x;
        sumY += y;
      }
    }
  }

  if (fire_pixels == 0) return 0;
  *x_center = sumX / fire_pixels;
  *y_center = sumY / fire_pixels;
  *fire_size = fire_pixels;
  return 1;
}

// --- Lidar Distance Reading ---
double get_min_lidar_distance(WbDeviceTag lidar) {
  const float *ranges = wb_lidar_get_range_image(lidar);
  if (!ranges) return INFINITY;
  int width = wb_lidar_get_horizontal_resolution(lidar);
  double min_distance = INFINITY;
  
  int start = width * 0.3;
  int end = width * 0.7;
  
  for (int i = start; i < end; i++) {
    if (ranges[i] < min_distance && ranges[i] > 0.05) { 
      min_distance = ranges[i];
    }
  }
  return min_distance;
}

// --- Main Method ---
int main() {
  wb_robot_init();

  //Sensors
  WbDeviceTag camera = wb_robot_get_device(CAMERA_NAME);
  wb_camera_enable(camera, TIME_STEP);

  WbDeviceTag lidar = wb_robot_get_device(LIDAR_NAME);
  wb_lidar_enable(lidar, TIME_STEP);
  wb_lidar_enable_point_cloud(lidar);

  // Motors
  WbDeviceTag left_motor = wb_robot_get_device(LEFT_MOTOR);
  WbDeviceTag right_motor = wb_robot_get_device(RIGHT_MOTOR);
  
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // Fire Node
  WbNodeRef fire_node = wb_supervisor_node_get_from_def("FIRE_PLACE");
  if (fire_node == NULL) printf("⚠️ Warning: FIRE_PLACE not found!\n");

  RobotState state = SEARCHING;
  double fireX = 0, fireY = 0;
  int fireSize = 0;
  int width = wb_camera_get_width(camera);
  double centerX = width / 2.0;
  int avoid_timer = 0;

  printf("=== Pioneer Rescue: Precision Mode ===\n");

  // --- Main Loop ---
  while (wb_robot_step(TIME_STEP) != -1) {
    // Read Sensors
    double min_distance = get_min_lidar_distance(lidar);
    const unsigned char *image = wb_camera_get_image(camera);
    if (!image) continue;

    // Detect Fire Position
    int found = detect_fire(camera, &fireX, &fireY, &fireSize, min_distance);
    double offset = found ? (fireX - centerX) / centerX : 0;

    switch (state) {
      // --- State: Searching ---
      case SEARCHING:
        if (found) {
            printf("🔥 Fire found! Stabilizing...\n");
            set_speed(left_motor, right_motor, 0, 0); 
            state = ALIGNING;
        }
        else {
            set_speed(left_motor, right_motor, -1.0, 1.0); 
        }
        break;

      // --- State: Aligning ---
      case ALIGNING:
        if (!found) { state = SEARCHING; break; }
        
        if (fabs(offset) < 0.15) {
          set_speed(left_motor, right_motor, 0, 0);
          state = MOVING_FAST;
        } else if (offset < 0) {
           set_speed(left_motor, right_motor, -0.5, 0.5);
        } else {
           set_speed(left_motor, right_motor, 0.5, -0.5);
        }
        break;

      // --- State: Moving to Fire ---
      case MOVING_FAST:
        
        // Priority Stop (Too Close)
        if (min_distance < 0.6) {
             printf("✅ GOAL REACHED (Proximity Stop).\n");
             set_speed(left_motor, right_motor, 0, 0);
             state = STOPPED;
             break;
        }

        // --- Rabbit Detection Part ---
        if (min_distance < 1.2 && fireSize < 50) {
             printf("🐇 Obstacle! Short Avoidance...\n");
             set_speed(left_motor, right_motor, 0, 0); 
             state = PRE_AVOID; 
             avoid_timer = 0;
             break;
        }

        // --- Goal Reached Check ---
        if (min_distance < SAFE_DISTANCE) {
             printf("✅ GOAL REACHED (Visual Stop).\n");
             set_speed(left_motor, right_motor, 0, 0);
             state = STOPPED;
             break;
        }

        // if Lost Fire
        if (!found) { 
            if (min_distance < 0.9) {
                set_speed(left_motor, right_motor, MAX_SPEED, MAX_SPEED);
            } else {
                state = SEARCHING; 
            }
            break; 
        }
        
        // Correction
        if (offset < -0.2) {
             set_speed(left_motor, right_motor, MAX_SPEED * 0.8, MAX_SPEED); 
        } else if (offset > 0.2) {
             set_speed(left_motor, right_motor, MAX_SPEED, MAX_SPEED * 0.8); 
        } else {
             set_speed(left_motor, right_motor, MAX_SPEED, MAX_SPEED);
        }
        break;

      // --- State: Stabilize Before Turn ---
      case PRE_AVOID:
        avoid_timer++;
        set_speed(left_motor, right_motor, 0, 0);
        if (avoid_timer > 5) { 
             state = AVOIDING;
             avoid_timer = 0;
        }
        break;

      // --- State: Obstacle Avoidance ---
      case AVOIDING:
        avoid_timer++;
      
        // Turn Right
        if (avoid_timer < 12) { 
            set_speed(left_motor, right_motor, 2.0, -2.0); 
        } 
        // Move Forward
        else if (avoid_timer < 60) { 
            // Wall Protection
            if (min_distance < 0.5) {
                set_speed(left_motor, right_motor, 0, 0);
                state = SEARCHING;
            } else {
                set_speed(left_motor, right_motor, MAX_SPEED, MAX_SPEED);
            }
        }
        else {
            printf("Avoidance complete.\n");
            set_speed(left_motor, right_motor, 0, 0); 
            state = SEARCHING;
        }
        break;

      // --- State: Extinguish Fire ---
      case STOPPED:
        set_speed(left_motor, right_motor, 0, 0);
        
        // Remove Fire Object
        if (fire_node != NULL) {
            wb_supervisor_node_remove(fire_node);
            fire_node = NULL;
            printf("💦 Fire Extinguished!\n");
        }
        break;
        
      default:
        break;
    }
  }

  wb_robot_cleanup();
  return 0;
}






















