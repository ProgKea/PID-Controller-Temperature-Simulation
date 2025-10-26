#include <pthread.h>
#include <unistd.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include <stdint.h>

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

typedef int8_t i8;
typedef int16_t i16;
typedef int32_t i32;
typedef int64_t i64;

typedef float f32;
typedef double f64;

// helper macros
#define MAX(A, B) ((A) < (B) ? (B) : (A))
#define MIN(A, B) ((A) > (B) ? (B) : (A))
#define CLAMP(V, LO, HI) MIN(MAX(V, LO), HI)


#include <raylib.h>

// NOTE: All temperature values are in °C

#define SIMULATION_DT_SECS 10
#define SIMULATION_SAMPLE_CAPACITY 1500
#define SIMULATION_MAX_TEMPERATURE 50
#define FONT_SIZE 20

typedef struct {
  f32 temperature;
  f32 time;
  f32 P, I, D;
} PID_Sample;

typedef struct {
  PID_Sample data[SIMULATION_SAMPLE_CAPACITY];
  i32 start;
  i32 count;
} PID_Sample_Ring_Buffer;

typedef struct {
  // simulation state
  // {
  f32 target_temperature;
  f32 heating_power;
  f32 time;

  f32 outside_temperature;
  f32 temperature;
  f32 accumulated_error;
  f32 previous_error;

  f32 Kp;
  f32 Ki;
  f32 Kd;

  PID_Sample_Ring_Buffer samples;
  i32 plot_x_offset;
  bool paused;
  // }

  i32 window_width;
  i32 window_height;
} PID_State;

pthread_mutex_t simulation_mutex;
pthread_t simulation_thread;

static volatile PID_State state = {0};

static void
add_sample(volatile PID_Sample_Ring_Buffer *ring_buffer, PID_Sample item)
{
  if (ring_buffer->count < SIMULATION_SAMPLE_CAPACITY)
  {
    ring_buffer->data[ring_buffer->count++] = item;
  }
  else
  {
    ring_buffer->start %= SIMULATION_SAMPLE_CAPACITY;
    ring_buffer->data[ring_buffer->start++] = item;
  }
}

static void
copy_samples(PID_Sample *dst, PID_Sample_Ring_Buffer ring_buffer)
{
  if (ring_buffer.start == 0)
  {
    memcpy(dst, ring_buffer.data, sizeof(PID_Sample)*ring_buffer.count);
  }
  else
  {
    i32 count = ring_buffer.count - ring_buffer.start;
    memcpy(dst, ring_buffer.data + ring_buffer.start, sizeof(PID_Sample) * count);
    memcpy(dst + count, ring_buffer.data, sizeof(PID_Sample) * (ring_buffer.count - count));
  }
}

static f32
y_from_temperature(f32 height, f32 temperature)
{
  f32 y = height - (height * (temperature / SIMULATION_MAX_TEMPERATURE));
  return y;
}

void *
simulation_main(void *argument)
{
  (void) argument;

  for (;;)
  {
    pthread_mutex_lock(&simulation_mutex);
    if (!state.paused)
    {
      f32 error = state.target_temperature - state.temperature;
      f32 P = state.Kp * error;
      f32 I = state.Ki * state.accumulated_error;
      f32 D = state.Kd * ((error - state.previous_error) / SIMULATION_DT_SECS);
      printf("P = %f, I = %f, D = %f\n", P, I, D);
      state.heating_power = CLAMP(P + I + D, 0, 100);

      state.previous_error = error;
      state.accumulated_error += error * SIMULATION_DT_SECS;

      const f32 heat_loss_coefficient = 1.5f;
      const f32 thermal_mass = 500;
      f32 heat_loss = heat_loss_coefficient * (state.temperature - state.outside_temperature);
      f32 net_heat = state.heating_power - heat_loss;
      f32 temperature_change = (net_heat / thermal_mass) * SIMULATION_DT_SECS;
      state.temperature += temperature_change;

      state.time += SIMULATION_DT_SECS;

      PID_Sample sample = {
        .temperature = state.temperature,
        .time = state.time,
      };
      add_sample(&state.samples, sample);

      printf("temperature = %.2f\n", state.temperature);
      printf("target temperature = %.2f\n", state.target_temperature);
      printf("heating power = %.2f\n", state.heating_power);
      printf("temperature change = %f\n", temperature_change);
    }
    pthread_mutex_unlock(&simulation_mutex);

    usleep((SIMULATION_DT_SECS*1000*1000)/100);
  }

  return 0;
}

void
pid_plot(Rectangle rectangle)
{
  BeginScissorMode(rectangle.x, rectangle.y, rectangle.width, rectangle.height);

  static PID_Sample samples[SIMULATION_SAMPLE_CAPACITY];
  i32 sample_count = 0;
  pthread_mutex_lock(&simulation_mutex);
  f32 y_target = y_from_temperature(rectangle.height, state.target_temperature);
  sample_count = state.samples.count;
  copy_samples(samples, state.samples);
  pthread_mutex_unlock(&simulation_mutex);

  // draw labels
  f32 temperature_line_x = 0;
  i32 steps = SIMULATION_MAX_TEMPERATURE/10;
  for (i32 step = 0; step <= steps; ++step)
  {
    assert(step < 1000);
    i32 y = rectangle.height - (rectangle.height/steps*step) + rectangle.y;
    char buffer[4];
    sprintf(buffer, "%d", step*10);

    i32 y_centered_and_clamped = y-FONT_SIZE/2;
    if (y_centered_and_clamped < rectangle.y)
    {
      y_centered_and_clamped = rectangle.y;
    }
    else if (y_centered_and_clamped + FONT_SIZE/2 >= rectangle.y + rectangle.height)
    {
      y_centered_and_clamped = y-FONT_SIZE;
    }

    DrawText(buffer, rectangle.x, y_centered_and_clamped, FONT_SIZE, WHITE);
    temperature_line_x = MAX((f32) MeasureText(buffer, FONT_SIZE), temperature_line_x);
  }

  f32 x_start = temperature_line_x + 2.0f + rectangle.x;
  DrawLineV((Vector2) {x_start, rectangle.y}, (Vector2) {x_start, rectangle.height+rectangle.y}, WHITE);
  DrawLineV((Vector2) {rectangle.x, y_target + rectangle.y}, (Vector2) {rectangle.width + rectangle.x, y_target+rectangle.y}, GREEN);

  // calculate x offset
  f32 offset = 0.0f;
  f32 last_sample_x = state.time + x_start;
  f32 edge = rectangle.x + rectangle.width - 50.0f;
  if (last_sample_x > edge)
  {
    offset = last_sample_x - edge;
  }

  // samples -> points
  static Vector2 sample_points[SIMULATION_SAMPLE_CAPACITY];
  for (i32 i = 0; i < sample_count; ++i)
  {
    Vector2 *sample_point = sample_points + i;
    sample_point->x = samples[i].time + x_start - offset;
    sample_point->y = y_from_temperature(rectangle.height, samples[i].temperature) + rectangle.y;
    DrawCircleV(*sample_point, 4.0f, RED);
  }
  DrawSplineLinear(sample_points, sample_count, 2.0f, RED);

  if (sample_count > 0)
  {
    char buffer[10];
    i32 i = sample_count-1;
    sprintf(buffer, "%.02f", samples[i].temperature);

    i32 width = MeasureText(buffer, FONT_SIZE);

    Vector2 position = sample_points[i];
    position.x -= width/2;
    position.y -= FONT_SIZE;
    DrawText(buffer, position.x, position.y, FONT_SIZE, WHITE);
  }

  EndScissorMode();
}

int
main(void)
{
  state.target_temperature = 22;
  state.outside_temperature = 9;
  state.temperature = 15;

  PID_Sample sample = {
    .temperature = state.temperature,
    .time = 0.0f,
  };
  add_sample(&state.samples, sample);

  state.Kp = 5.0f;
  state.Ki = 0.05f;
  state.Kd = 1.0f;

  assert(pthread_mutex_init(&simulation_mutex, 0) == 0);
  pthread_create(&simulation_thread, 0, simulation_main, 0);

  const i32 FACTOR = 80;
  SetConfigFlags(FLAG_VSYNC_HINT|FLAG_MSAA_4X_HINT|FLAG_WINDOW_RESIZABLE);
  InitWindow(16*FACTOR, 9*FACTOR, "PID Controller Simulation");

  while (!WindowShouldClose())
  {
    if (state.window_width == 0 || IsWindowResized())
    {
      state.window_width = GetScreenWidth();
      state.window_height = GetScreenHeight();
    }

    BeginDrawing();
    ClearBackground(GetColor(0x181818FF));

    f32 padding = 10.0f;
    Rectangle plot_rectangle = {
      .x = padding,
      .y = padding,
      .width = state.window_width - padding*2,
      .height = state.window_height - padding*2,
    };
    pid_plot(plot_rectangle);

    if (IsKeyPressed(KEY_SPACE))
    {
      pthread_mutex_lock(&simulation_mutex);
      state.paused = !state.paused;
      pthread_mutex_unlock(&simulation_mutex);
    }

    Vector2 mouse = GetMousePosition();
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && CheckCollisionPointRec(mouse, plot_rectangle))
    {
      f32 temperature = SIMULATION_MAX_TEMPERATURE * ((plot_rectangle.height - (mouse.y - plot_rectangle.y)) / plot_rectangle.height);
      pthread_mutex_lock(&simulation_mutex);
      state.target_temperature = temperature;
      pthread_mutex_unlock(&simulation_mutex);
    }

    EndDrawing();
  }
  CloseWindow();
  pthread_mutex_destroy(&simulation_mutex);

  return 0;
}
