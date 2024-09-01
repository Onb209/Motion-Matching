extern "C"
{
#include "raylib.h"
#include "raymath.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
}
#if defined(PLATFORM_WEB)
#include <emscripten/emscripten.h>
#endif

#include "common.h"
#include "vec.h"
#include "quat.h"
#include "spring.h"
#include "array.h"
#include "character.h"
#include "database.h"
#include "nnet.h"
#include "lmm.h"
#define RLIGHTS_IMPLEMENTATION
#include "rlights.h"
#include <initializer_list>
#include <functional>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <chrono>
using namespace std;
#define GLSL_VERSION 330

//--------------------------------------

static inline Vector3 to_Vector3(vec3 v)
{
    return (Vector3){ v.x, v.y, v.z };
}

//--------------------------------------


int world = 1;
 
float x_max = 0.0f;
float z_max = 0.0f;
 
int x_index_max = 0;
int z_index_max = 0;
 
void setTerrainParameters(int world) {
    if (world == 1)
    {
        // rough terrain
        x_max = 30.43f;
        z_max = 13.44f;
        x_index_max = 1332;
        z_index_max = 593;
    }
    else if (world == 2)
    {
        // urban terrain
        x_max = 23.61f;
        z_max = 23.45f;
        x_index_max = 520;
        z_index_max = 516;
    }
}
 
int mapToRange(float value, float minInput, float maxInput, int minOutput, int maxOutput) {
    float normalizedValue = (value - minInput) / (maxInput - minInput);
    
    float mappedValue = normalizedValue * (maxOutput - minOutput) + minOutput;
    
    int roundedValue = std::round(mappedValue);
    
    if (roundedValue < minOutput) {
        return minOutput;
    } else if (roundedValue > maxOutput) {
        return maxOutput;
    }
    
    return roundedValue;
}
 
float findClosestY(const vector<float> terrain_y, float position_x, float position_z) {
    int index_x = mapToRange(position_x, -x_max, x_max, 0, x_index_max - 1);
    int index_z = mapToRange(position_z, -z_max, z_max, 0, z_index_max - 1);
    
    int index_y = index_x + index_z * x_index_max;
 
    return terrain_y[index_y];
}



// Perform linear blend skinning and copy 
// result into mesh data. Update and upload 
// deformed vertex positions and normals to GPU
void deform_character_mesh(
  Mesh& mesh, 
  const character& c,
  const slice1d<vec3> bone_anim_positions,
  const slice1d<quat> bone_anim_rotations,
  const slice1d<int> bone_parents)
{
    linear_blend_skinning_positions(
        slice1d<vec3>(mesh.vertexCount, (vec3*)mesh.vertices),
        c.positions,
        c.bone_weights,
        c.bone_indices,
        c.bone_rest_positions,
        c.bone_rest_rotations,
        bone_anim_positions,
        bone_anim_rotations);
    
    linear_blend_skinning_normals(
        slice1d<vec3>(mesh.vertexCount, (vec3*)mesh.normals),
        c.normals,
        c.bone_weights,
        c.bone_indices,
        c.bone_rest_rotations,
        bone_anim_rotations);
    
    UpdateMeshBuffer(mesh, 0, mesh.vertices, mesh.vertexCount * 3 * sizeof(float), 0);
    UpdateMeshBuffer(mesh, 2, mesh.normals, mesh.vertexCount * 3 * sizeof(float), 0);
}

Mesh make_character_mesh(character& c)
{
    Mesh mesh = { 0 };
    
    mesh.vertexCount = c.positions.size;
    mesh.triangleCount = c.triangles.size / 3;
    mesh.vertices = (float*)MemAlloc(c.positions.size * 3 * sizeof(float));
    mesh.texcoords = (float*)MemAlloc(c.texcoords.size * 2 * sizeof(float));
    mesh.normals = (float*)MemAlloc(c.normals.size * 3 * sizeof(float));
    mesh.indices = (unsigned short*)MemAlloc(c.triangles.size * sizeof(unsigned short));
    
    memcpy(mesh.vertices, c.positions.data, c.positions.size * 3 * sizeof(float));
    memcpy(mesh.texcoords, c.texcoords.data, c.texcoords.size * 2 * sizeof(float));
    memcpy(mesh.normals, c.normals.data, c.normals.size * 3 * sizeof(float));
    memcpy(mesh.indices, c.triangles.data, c.triangles.size * sizeof(unsigned short));
    
    UploadMesh(&mesh, true);
    
    return mesh;
}

//--------------------------------------

// Basic functionality to get gamepad input including deadzone and 
// squaring of the stick location to increase sensitivity. To make 
// all the other code that uses this easier, we assume stick is 
// oriented on floor (i.e. y-axis is zero)

enum
{
    GAMEPAD_PLAYER = 0,
};

enum
{
    GAMEPAD_STICK_LEFT,
    GAMEPAD_STICK_RIGHT,
};

vec3 gamepad_get_stick(int stick, const float deadzone = 0.2f)
{
// I've replaced all the original content of this function with the following:
vec3 pad(0,0,0);float am = 1.f;
if (IsKeyDown(KEY_RIGHT_SHIFT) || IsKeyDown(KEY_LEFT_SHIFT)) am*=0.5f;  // use SHIFT to slow down/walk
if (IsKeyDown(KEY_RIGHT_CONTROL) || IsKeyDown(KEY_LEFT_CONTROL)) am*=2.f; // use CTRL to speed up
if (IsKeyDown(KEY_A)) am*=0.25f;
if (stick==GAMEPAD_STICK_LEFT)  {
        // use ARROW keys to move the character
    if (IsKeyDown(KEY_RIGHT)) pad.x+=am;
    if (IsKeyDown(KEY_LEFT)) pad.x-=am; 
    if (IsKeyDown(KEY_UP)) pad.z-=am;
    if (IsKeyDown(KEY_DOWN)) pad.z+=am;
}
// else if (stick==GAMEPAD_STICK_RIGHT)    {
//         // use the number pad to move the camera
//     if (IsKeyDown(KEY_Q)) pad.x-=am;
//     if (IsKeyDown(KEY_E)) pad.x+=am; 
//     if (IsKeyDown(KEY_W)) pad.z+=am;
//     if (IsKeyDown(KEY_S)) pad.z-=am;
// }
else if (stick==GAMEPAD_STICK_RIGHT)	{
            // use the number pad to move the camera
        if (IsKeyDown(KEY_KP_6)) pad.x-=am;
        if (IsKeyDown(KEY_KP_4)) pad.x+=am; 
        if (IsKeyDown(KEY_KP_8)) pad.z+=am;
        if (IsKeyDown(KEY_KP_2)) pad.z-=am;
    }
return pad;
}

int keboard_motion_index() {
    
    // int motion_idx = 0;

    if (IsKeyDown(KEY_Q)) return 1;
    if (IsKeyDown(KEY_W)) return 2;
    if (IsKeyDown(KEY_E)) return 3;
    // if (IsKeyDown(KEY_R)) return 4;

}
// vec3 gamepad_get_stick(int stick, const float deadzone = 0.2f)
// {
//     float gamepadx = GetGamepadAxisMovement(GAMEPAD_PLAYER, stick == GAMEPAD_STICK_LEFT ? GAMEPAD_AXIS_LEFT_X : GAMEPAD_AXIS_RIGHT_X);
//     float gamepady = GetGamepadAxisMovement(GAMEPAD_PLAYER, stick == GAMEPAD_STICK_LEFT ? GAMEPAD_AXIS_LEFT_Y : GAMEPAD_AXIS_RIGHT_Y);
//     float gamepadmag = sqrtf(gamepadx*gamepadx + gamepady*gamepady);
    
//     if (gamepadmag > deadzone)
//     {
//         float gamepaddirx = gamepadx / gamepadmag;
//         float gamepaddiry = gamepady / gamepadmag;
//         float gamepadclippedmag = gamepadmag > 1.0f ? 1.0f : gamepadmag*gamepadmag;
//         gamepadx = gamepaddirx * gamepadclippedmag;
//         gamepady = gamepaddiry * gamepadclippedmag;
//     }
//     else
//     {
//         gamepadx = 0.0f;
//         gamepady = 0.0f;
//     }
    
//     return vec3(gamepadx, 0.0f, gamepady);
// }

//--------------------------------------

float orbit_camera_update_azimuth(
    const float azimuth, 
    const vec3 gamepadstick_right,
    const bool desired_strafe,
    const float dt)
{
    vec3 gamepadaxis = desired_strafe ? vec3() : gamepadstick_right;
    return azimuth + 2.0f * dt * -gamepadaxis.x;
}

float orbit_camera_update_altitude(
    const float altitude, 
    const vec3 gamepadstick_right,
    const bool desired_strafe,
    const float dt)
{
    vec3 gamepadaxis = desired_strafe ? vec3() : gamepadstick_right;
    return clampf(altitude + 2.0f * dt * gamepadaxis.z, 0.0, 0.4f * PIf);
}

float orbit_camera_update_distance(
    const float distance, 
    const float dt)
{
    float gamepadzoom = 
        IsGamepadButtonDown(GAMEPAD_PLAYER, GAMEPAD_BUTTON_LEFT_TRIGGER_1)  ? +1.0f :
        IsGamepadButtonDown(GAMEPAD_PLAYER, GAMEPAD_BUTTON_RIGHT_TRIGGER_1) ? -1.0f : 0.0f;
        
    return clampf(distance +  10.0f * dt * gamepadzoom, 0.1f, 100.0f);
}

// Updates the camera using the orbit cam controls
void orbit_camera_update(
    Camera3D& cam, 
    float& camera_azimuth,
    float& camera_altitude,
    float& camera_distance,
    const vec3 target,
    const vec3 gamepadstick_right,
    const bool desired_strafe,
    const float dt)
{
    camera_azimuth = orbit_camera_update_azimuth(camera_azimuth, gamepadstick_right, desired_strafe, dt);
    camera_altitude = orbit_camera_update_altitude(camera_altitude, gamepadstick_right, desired_strafe, dt);
    camera_distance = orbit_camera_update_distance(camera_distance, dt);
    
    quat rotation_azimuth = quat_from_angle_axis(camera_azimuth, vec3(0, 1, 0));
    vec3 position = quat_mul_vec3(rotation_azimuth, vec3(0, 0, camera_distance));
    vec3 axis = normalize(cross(position, vec3(0, 1, 0)));
    
    quat rotation_altitude = quat_from_angle_axis(camera_altitude, axis);
    
    vec3 eye = target + quat_mul_vec3(rotation_altitude, position);

    cam.target = (Vector3){ target.x, target.y, target.z };
    cam.position = (Vector3){ eye.x, eye.y, eye.z };
}

//--------------------------------------

// bool desired_strafe_update()
// {
//     return IsGamepadButtonDown(GAMEPAD_PLAYER, GAMEPAD_BUTTON_LEFT_TRIGGER_2) > 0.5f;
// }
bool desired_strafe_update()
{
    //return IsGamepadButtonDown(GAMEPAD_PLAYER, GAMEPAD_BUTTON_LEFT_TRIGGER_2) > 0.5f;
    return IsKeyDown(KEY_LEFT_ALT); // use left ALT to strafe
}


void desired_gait_update(
    float& desired_gait, 
    float& desired_gait_velocity,
    const float dt,
    const float gait_change_halflife = 0.1f)
{
    simple_spring_damper_exact(
        desired_gait, 
        desired_gait_velocity,
        IsGamepadButtonDown(GAMEPAD_PLAYER, GAMEPAD_BUTTON_RIGHT_FACE_DOWN) ? 1.0f : 0.0f,
        gait_change_halflife,
        dt);
}

vec3 desired_velocity_update(
    const vec3 gamepadstick_left,
    const float camera_azimuth,
    const quat simulation_rotation,
    const float fwrd_speed,
    const float side_speed,
    const float back_speed)
{
    // Find stick position in world space by rotating using camera azimuth
    vec3 global_stick_direction = quat_mul_vec3(
        quat_from_angle_axis(camera_azimuth, vec3(0, 1, 0)), gamepadstick_left);
    
    // Find stick position local to current facing direction
    vec3 local_stick_direction = quat_inv_mul_vec3(
        simulation_rotation, global_stick_direction);
    
    // Scale stick by forward, sideways and backwards speeds
    vec3 local_desired_velocity = local_stick_direction.z > 0.0 ?
        vec3(side_speed, 0.0f, fwrd_speed) * local_stick_direction :
        vec3(side_speed, 0.0f, back_speed) * local_stick_direction;
    
    // Re-orientate into the world space
    return quat_mul_vec3(simulation_rotation, local_desired_velocity);
}

quat desired_rotation_update(
    const quat desired_rotation,
    const vec3 gamepadstick_left,
    const vec3 gamepadstick_right,
    const float camera_azimuth,
    const bool desired_strafe,
    const vec3 desired_velocity)
{
    quat desired_rotation_curr = desired_rotation;
    
    // If strafe is active then desired direction is coming from right
    // stick as long as that stick is being used, otherwise we assume
    // forward facing
    if (desired_strafe)
    {
        vec3 desired_direction = quat_mul_vec3(quat_from_angle_axis(camera_azimuth, vec3(0, 1, 0)), vec3(0, 0, -1));

        if (length(gamepadstick_right) > 0.01f)
        {
            desired_direction = quat_mul_vec3(quat_from_angle_axis(camera_azimuth, vec3(0, 1, 0)), normalize(gamepadstick_right));
        }
        
        return quat_from_angle_axis(atan2f(desired_direction.x, desired_direction.z), vec3(0, 1, 0));            
    }
    
    // If strafe is not active the desired direction comes from the left 
    // stick as long as that stick is being used
    else if (length(gamepadstick_left) > 0.01f)
    {
        
        vec3 desired_direction = normalize(desired_velocity);
        return quat_from_angle_axis(atan2f(desired_direction.x, desired_direction.z), vec3(0, 1, 0));
    }
    
    // Otherwise desired direction remains the same
    else
    {
        return desired_rotation_curr;
    }
}

//--------------------------------------

// Moving the root is a little bit difficult when we have the
// inertializer set up in the way we do. Essentially we need
// to also make sure to adjust all of the locations where 
// we are transforming the data to and from as well as the 
// offsets being blended out
void inertialize_root_adjust(
    vec3& offset_position,
    vec3& transition_src_position,
    quat& transition_src_rotation,
    vec3& transition_dst_position,
    quat& transition_dst_rotation,
    vec3& position,
    quat& rotation,
    const vec3 input_position,
    const quat input_rotation)
{
    // Find the position difference and add it to the state and transition location
    vec3 position_difference = input_position - position;
    position = position_difference + position;
    transition_dst_position = position_difference + transition_dst_position;
    
    // Find the point at which we want to now transition from in the src data
    transition_src_position = transition_src_position + quat_mul_vec3(transition_src_rotation,
        quat_inv_mul_vec3(transition_dst_rotation, position - offset_position - transition_dst_position));
    transition_dst_position = position;
    offset_position = vec3();
    
    // Find the rotation difference. We need to normalize here or some error can accumulate 
    // over time during adjustment.
    quat rotation_difference = quat_normalize(quat_mul_inv(input_rotation, rotation));
    
    // Apply the rotation difference to the current rotation and transition location
    rotation = quat_mul(rotation_difference, rotation);
    transition_dst_rotation = quat_mul(rotation_difference, transition_dst_rotation);
}

void inertialize_pose_reset(
    slice1d<vec3> bone_offset_positions,
    slice1d<vec3> bone_offset_velocities,
    slice1d<quat> bone_offset_rotations,
    slice1d<vec3> bone_offset_angular_velocities,
    vec3& transition_src_position,
    quat& transition_src_rotation,
    vec3& transition_dst_position,
    quat& transition_dst_rotation,
    const vec3 root_position,
    const quat root_rotation)
{
    bone_offset_positions.zero();
    bone_offset_velocities.zero();
    bone_offset_rotations.set(quat());
    bone_offset_angular_velocities.zero();
    
    transition_src_position = root_position;
    transition_src_rotation = root_rotation;
    transition_dst_position = vec3();
    transition_dst_rotation = quat();
    // transition_dst_position = root_position;
    // transition_dst_rotation = root_rotation;
}

void inertialize_pose_reset2(
    slice1d<vec3> bone_offset_positions,
    slice1d<vec3> bone_offset_velocities,
    slice1d<quat> bone_offset_rotations,
    slice1d<vec3> bone_offset_angular_velocities,
    vec3& transition_src_position,
    quat& transition_src_rotation,
    vec3& transition_dst_position,
    quat& transition_dst_rotation,
    const vec3 root_position,
    const quat root_rotation)
{
    bone_offset_positions.zero();
    bone_offset_velocities.zero();
    bone_offset_rotations.set(quat());
    bone_offset_angular_velocities.zero();
    
    transition_src_position = root_position;
    transition_src_rotation = root_rotation;
    // transition_dst_position = vec3();
    // transition_dst_rotation = quat();
    transition_dst_position = root_position;
    transition_dst_rotation = root_rotation;
}


// This function transitions the inertializer for 
// the full character. It takes as input the current 
// offsets, as well as the root transition locations,
// current root state, and the full pose information 
// for the pose being transitioned from (src) as well 
// as the pose being transitioned to (dst) in their
// own animation spaces.
void inertialize_pose_transition(
    slice1d<vec3> bone_offset_positions,
    slice1d<vec3> bone_offset_velocities,
    slice1d<quat> bone_offset_rotations,
    slice1d<vec3> bone_offset_angular_velocities,
    vec3& transition_src_position,
    quat& transition_src_rotation,
    vec3& transition_dst_position,
    quat& transition_dst_rotation,
    const vec3 root_position,
    const vec3 root_velocity,
    const quat root_rotation,
    const vec3 root_angular_velocity,
    const slice1d<vec3> bone_src_positions,
    const slice1d<vec3> bone_src_velocities,
    const slice1d<quat> bone_src_rotations,
    const slice1d<vec3> bone_src_angular_velocities,
    const slice1d<vec3> bone_dst_positions,
    const slice1d<vec3> bone_dst_velocities,
    const slice1d<quat> bone_dst_rotations,
    const slice1d<vec3> bone_dst_angular_velocities)
{
    // First we record the root position and rotation
    // in the animation data for the source and destination
    // animation
    transition_dst_position = root_position;
    transition_dst_rotation = root_rotation;
    transition_src_position = bone_dst_positions(0);
    transition_src_rotation = bone_dst_rotations(0);
    
    // We then find the velocities so we can transition the 
    // root inertiaizers
    vec3 world_space_dst_velocity = quat_mul_vec3(transition_dst_rotation, 
        quat_inv_mul_vec3(transition_src_rotation, bone_dst_velocities(0)));
    
    vec3 world_space_dst_angular_velocity = quat_mul_vec3(transition_dst_rotation, 
        quat_inv_mul_vec3(transition_src_rotation, bone_dst_angular_velocities(0)));
    
    // Transition inertializers recording the offsets for 
    // the root joint
    inertialize_transition(
        bone_offset_positions(0),
        bone_offset_velocities(0),
        root_position,
        root_velocity,
        root_position,
        world_space_dst_velocity);
        
    inertialize_transition(
        bone_offset_rotations(0),
        bone_offset_angular_velocities(0),
        root_rotation,
        root_angular_velocity,
        root_rotation,
        world_space_dst_angular_velocity);
    
    // Transition all the inertializers for each other bone
    for (int i = 1; i < bone_offset_positions.size; i++)
    {
        inertialize_transition(
            bone_offset_positions(i),
            bone_offset_velocities(i),
            bone_src_positions(i),
            bone_src_velocities(i),
            bone_dst_positions(i),
            bone_dst_velocities(i));
            
        inertialize_transition(
            bone_offset_rotations(i),
            bone_offset_angular_velocities(i),
            bone_src_rotations(i),
            bone_src_angular_velocities(i),
            bone_dst_rotations(i),
            bone_dst_angular_velocities(i));
    }
}

// This function updates the inertializer states. Here 
// it outputs the smoothed animation (input plus offset) 
// as well as updating the offsets themselves. It takes 
// as input the current playing animation as well as the 
// root transition locations, a halflife, and a dt
void inertialize_pose_update(
    slice1d<vec3> bone_positions,
    slice1d<vec3> bone_velocities,
    slice1d<quat> bone_rotations,
    slice1d<vec3> bone_angular_velocities,
    slice1d<vec3> bone_offset_positions,
    slice1d<vec3> bone_offset_velocities,
    slice1d<quat> bone_offset_rotations,
    slice1d<vec3> bone_offset_angular_velocities,
    const slice1d<vec3> bone_input_positions,
    const slice1d<vec3> bone_input_velocities,
    const slice1d<quat> bone_input_rotations,
    const slice1d<vec3> bone_input_angular_velocities,
    const vec3 transition_src_position,
    const quat transition_src_rotation,
    const vec3 transition_dst_position,
    const quat transition_dst_rotation,
    const float halflife,
    const float dt)
{
    // First we find the next root position, velocity, rotation
    // and rotational velocity in the world space by transforming 
    // the input animation from it's animation space into the 
    // space of the currently playing animation.
    vec3 world_space_position = quat_mul_vec3(transition_dst_rotation, 
        quat_inv_mul_vec3(transition_src_rotation, 
            bone_input_positions(0) - transition_src_position)) + transition_dst_position;
    
    vec3 world_space_velocity = quat_mul_vec3(transition_dst_rotation, 
        quat_inv_mul_vec3(transition_src_rotation, bone_input_velocities(0)));
    
    // Normalize here because quat inv mul can sometimes produce 
    // unstable returns when the two rotations are very close.
    quat world_space_rotation = quat_normalize(quat_mul(transition_dst_rotation, 
        quat_inv_mul(transition_src_rotation, bone_input_rotations(0))));
    
    vec3 world_space_angular_velocity = quat_mul_vec3(transition_dst_rotation, 
        quat_inv_mul_vec3(transition_src_rotation, bone_input_angular_velocities(0)));
    
    // Then we update these two inertializers with these new world space inputs
    inertialize_update(
        bone_positions(0),
        bone_velocities(0),
        bone_offset_positions(0),
        bone_offset_velocities(0),
        world_space_position,
        world_space_velocity,
        halflife,
        dt);
        
    inertialize_update(
        bone_rotations(0),
        bone_angular_velocities(0),
        bone_offset_rotations(0),
        bone_offset_angular_velocities(0),
        world_space_rotation,
        world_space_angular_velocity,
        halflife,
        dt);        
    
    // Then we update the inertializers for the rest of the bones
    for (int i = 1; i < bone_positions.size; i++)
    {
        inertialize_update(
            bone_positions(i),
            bone_velocities(i),
            bone_offset_positions(i),
            bone_offset_velocities(i),
            bone_input_positions(i),
            bone_input_velocities(i),
            halflife,
            dt);
            
        inertialize_update(
            bone_rotations(i),
            bone_angular_velocities(i),
            bone_offset_rotations(i),
            bone_offset_angular_velocities(i),
            bone_input_rotations(i),
            bone_input_angular_velocities(i),
            halflife,
            dt);
    }
}

void inertialize_pose_update2(
    slice1d<vec3> bone_positions,
    slice1d<vec3> bone_velocities,
    slice1d<quat> bone_rotations,
    slice1d<vec3> bone_angular_velocities,
    slice1d<vec3> bone_offset_positions,
    slice1d<vec3> bone_offset_velocities,
    slice1d<quat> bone_offset_rotations,
    slice1d<vec3> bone_offset_angular_velocities,
    const slice1d<vec3> bone_input_positions,
    const slice1d<vec3> bone_input_velocities,
    const slice1d<quat> bone_input_rotations,
    const slice1d<vec3> bone_input_angular_velocities,
    const vec3 transition_src_position,
    const quat transition_src_rotation,
    const vec3 transition_dst_position,
    const quat transition_dst_rotation,
    const float halflife,
    const float dt,
    const vector<float> terrain_y)
{
    // First we find the next root position, velocity, rotation
    // and rotational velocity in the world space by transforming 
    // the input animation from it's animation space into the 
    // space of the currently playing animation.
    vec3 world_space_position = quat_mul_vec3(transition_dst_rotation, 
        quat_inv_mul_vec3(transition_src_rotation, 
            bone_input_positions(0) - transition_src_position)) + transition_dst_position;

    world_space_position.y = findClosestY(terrain_y, world_space_position.x, world_space_position.z);
    
    vec3 world_space_velocity = quat_mul_vec3(transition_dst_rotation, 
        quat_inv_mul_vec3(transition_src_rotation, bone_input_velocities(0)));
    
    // Normalize here because quat inv mul can sometimes produce 
    // unstable returns when the two rotations are very close.
    quat world_space_rotation = quat_normalize(quat_mul(transition_dst_rotation, 
        quat_inv_mul(transition_src_rotation, bone_input_rotations(0))));
    
    vec3 world_space_angular_velocity = quat_mul_vec3(transition_dst_rotation, 
        quat_inv_mul_vec3(transition_src_rotation, bone_input_angular_velocities(0)));
    
    // Then we update these two inertializers with these new world space inputs
    inertialize_update(
        bone_positions(0),
        bone_velocities(0),
        bone_offset_positions(0),
        bone_offset_velocities(0),
        world_space_position,
        world_space_velocity,
        halflife,
        dt);
        
    inertialize_update(
        bone_rotations(0),
        bone_angular_velocities(0),
        bone_offset_rotations(0),
        bone_offset_angular_velocities(0),
        world_space_rotation,
        world_space_angular_velocity,
        halflife,
        dt);        
    
    // Then we update the inertializers for the rest of the bones
    for (int i = 1; i < bone_positions.size; i++)
    {
        inertialize_update(
            bone_positions(i),
            bone_velocities(i),
            bone_offset_positions(i),
            bone_offset_velocities(i),
            bone_input_positions(i),
            bone_input_velocities(i),
            halflife,
            dt);
            
        inertialize_update(
            bone_rotations(i),
            bone_angular_velocities(i),
            bone_offset_rotations(i),
            bone_offset_angular_velocities(i),
            bone_input_rotations(i),
            bone_input_angular_velocities(i),
            halflife,
            dt);
    }
}

//--------------------------------------
void query_compute_motion_index(
    slice1d<float> query, 
    int& offset,
    bool human_close,
    bool push_animation,
    int human_num)
{
    // if (push_animation){
    //     query(offset + 0) = 0;
    // } else if (human_close){
    //     query(offset + 0) = 1;
    // } 
    // if(!push_animation && !human_close){
    //     if(human_num == 0) query(offset + 0) = 2;
    //     else if(human_num == 1) query(offset + 0) = 3;
    //     else if(human_num == 2) query(offset + 0) = 4;
    //     else if(human_num == 3) query(offset + 0) = 5;
    //     else if(human_num == 4) query(offset + 0) = 6;
    // }

    if (push_animation){
        query(offset + 0) = 0;
    }
    else{
        query(offset + 0) = 1;
    }

    offset += 1;
}
// Copy a part of a feature vector from the 
// matching database into the query feature vector
void query_copy_denormalized_feature(
    slice1d<float> query, 
    int& offset, 
    const int size, 
    const slice1d<float> features,
    const slice1d<float> features_offset,
    const slice1d<float> features_scale)
{
    for (int i = 0; i < size; i++)
    {
        query(offset + i) = features(offset + i) * features_scale(offset + i) + features_offset(offset + i);
    }
    
    offset += size;
}

// void query_copy_denormalized_feature2(
//     slice1d<float> query, 
//     int& offset, 
//     const int size, 
//     const slice1d<float> features,
//     const slice1d<float> features_offset,
//     const slice1d<float> features_scale,
//     bool chair_close,
//     bool push_animation,
//     const vector<float> terrain_y)
// {
//     if (chair_close || push_animation){
//         query(offset + 0) = features(offset + 0) * features_scale(offset + 0) + features_offset(offset + 0);
        
//         query(offset + 1) = ( findClosestY(terrain_y, features(offset + 0), features(offset + 2)) +0.02 ) * features_scale(offset + 1) + features_offset(offset + 1); //
        
//         query(offset + 2) = features(offset + 2) * features_scale(offset + 2) + features_offset(offset + 2);
        
//         query(offset + 3) = features(offset + 3) * features_scale(offset + 3) + features_offset(offset + 3);
        
//         query(offset + 4) = (findClosestY(terrain_y, features(offset + 3), features(offset + 5)) +0.02 ) * features_scale(offset + 4) + features_offset(offset + 4); //
        
//         query(offset + 5) = features(offset + 5) * features_scale(offset + 5) + features_offset(offset + 5);

//         query(offset + 6) = features(offset + 6) * features_scale(offset + 6) + features_offset(offset + 6);
        
//         query(offset + 7) = (findClosestY(terrain_y, features(offset + 7), features(offset + 7)) +0.9) * features_scale(offset + 7) + features_offset(offset + 7); //

        
//         query(offset + 8) = features(offset + 8) * features_scale(offset + 8) + features_offset(offset + 8);

//     }
//     else{
//         for (int i = 0; i < 9; i++)
//         {
//             query(offset + i) = features(offset + i) * features_scale(offset + i) + features_offset(offset + i);
//         }
//     }

//     for (int i = 9; i < size; i++)
//     {
//         query(offset + i) = features(offset + i) * features_scale(offset + i) + features_offset(offset + i);
//     }
    
//     offset += size;
// }
// Compute the query feature vector for the current 
// trajectory controlled by the gamepad.
void query_compute_trajectory_position_feature(
    slice1d<float> query, 
    int& offset, 
    const vec3 root_position, 
    const vec3 root_body_position, 
    const quat root_rotation, 
    const slice1d<vec3> trajectory_positions,
    const vector<float> terrain_y)
{
    vec3 traj0 = quat_inv_mul_vec3(root_rotation, trajectory_positions(1) - root_position);
    vec3 traj1 = quat_inv_mul_vec3(root_rotation, trajectory_positions(2) - root_position);
    vec3 traj2 = quat_inv_mul_vec3(root_rotation, trajectory_positions(3) - root_position);
    
    float root_terrain_height = findClosestY(terrain_y, root_position.x, root_position.z);
    // root_terrain_height *= 0.0;
    traj0.y = findClosestY(terrain_y, trajectory_positions(1).x, trajectory_positions(1).z) +0.9 - root_position.y-root_body_position.y;
    traj1.y = findClosestY(terrain_y, trajectory_positions(2).x, trajectory_positions(2).z) +0.9 - root_position.y-root_body_position.y;
    traj2.y = findClosestY(terrain_y, trajectory_positions(3).x, trajectory_positions(3).z) +0.9 - root_position.y-root_body_position.y;

    // printf("traj_y: %f, %f, %f", traj0.y, traj1.y, traj2.y);
    query(offset + 0) = traj0.x;
    query(offset + 1) = traj0.y;
    query(offset + 2) = traj0.z;
    query(offset + 3) = traj1.x;
    query(offset + 4) = traj1.y;
    query(offset + 5) = traj1.z;
    query(offset + 6) = traj2.x;
    query(offset + 7) = traj2.y;
    query(offset + 8) = traj2.z;
    

    offset += 9;

}

// Same but for the trajectory direction
void query_compute_trajectory_direction_feature(
    slice1d<float> query, 
    int& offset, 
    const quat root_rotation, 
    const slice1d<quat> trajectory_rotations)
{
    vec3 traj0 = quat_inv_mul_vec3(root_rotation, quat_mul_vec3(trajectory_rotations(1), vec3(0, 0, 1)));
    vec3 traj1 = quat_inv_mul_vec3(root_rotation, quat_mul_vec3(trajectory_rotations(2), vec3(0, 0, 1)));
    vec3 traj2 = quat_inv_mul_vec3(root_rotation, quat_mul_vec3(trajectory_rotations(3), vec3(0, 0, 1)));
    
    query(offset + 0) = traj0.x;
    query(offset + 1) = traj0.z;
    query(offset + 2) = traj1.x;
    query(offset + 3) = traj1.z;
    query(offset + 4) = traj2.x;
    query(offset + 5) = traj2.z;
    
    
    offset += 6;
}

void query_compute_chair_position_feature(
    slice1d<float> query, 
    int& offset, 
    const vec3 root_position, 
    const quat root_rotation,
    const vec3 chair_position,
    bool chair_close)
{
    if (chair_close){
        vec3 pos = quat_inv_mul_vec3(root_rotation, chair_position - root_position);
    
        query(offset + 0) = pos.x;
        query(offset + 1) = pos.z;
    }
    else{
        query(offset + 0) = 0.0;
        query(offset + 1) = 0.0;
    }
    
    
    offset += 2;
}

// Same but for the trajectory direction
void query_compute_chair_direction_feature(
    slice1d<float> query, 
    int& offset, 
    const vec3 root_position, 
    const quat root_rotation,
    const quat chair_direction,
    bool chair_close)
{
    if (chair_close){
        vec3 relative_chair_dir = quat_inv_mul_vec3(root_rotation, quat_mul_vec3(chair_direction, vec3(0, 0, 1)));
    
        query(offset + 0) = relative_chair_dir.x;
        query(offset + 1) = relative_chair_dir.z;
    
    } else{
        query(offset + 0) = 0.0;
        query(offset + 1) = 0.0;
    }
    
    offset += 2;

}

void query_compute_other_position_feature(
    slice1d<float> query, 
    int& offset, 
    const vec3 root_position, 
    const quat root_rotation, 
    const vec3 root_position2,
    bool human_close)
    // const vec3 left_hand_position,
    // const vec3 right_hand_position)
{
    
    if (human_close) {
        vec3 pos = quat_inv_mul_vec3(root_rotation, root_position2 - root_position);
        
        query(offset + 0) = pos.x;
        query(offset + 1) = pos.z;
    } else {
        
        query(offset + 0) = 0.0;
        query(offset + 1) = 0.0;
    }
    
    offset += 2;

}

void query_compute_hip_velocity(
    slice1d<float> query, 
    int& offset, 
    const vec3 root_velocity,
    bool human_close
)
{
    if (human_close){
        query(offset + 0) = root_velocity.x;
        query(offset + 1) = root_velocity.y;
        query(offset + 2) = root_velocity.z;
    }
    else{
        query(offset + 0) = 0.0;
        query(offset + 1) = 0.0;
        query(offset + 2) = 0.0;
    }

    offset += 3;
}

void query_compute_other_hand_position_feature(
    slice1d<float> query, 
    int& offset, 
    const vec3 root_position, 
    const quat root_rotation, 
    const vec3 left_hand_position,
    const vec3 right_hand_position)
{
    vec3 left_pos = quat_inv_mul_vec3(root_rotation, left_hand_position - root_position);
    vec3 right_pos = quat_inv_mul_vec3(root_rotation, right_hand_position - root_position);

    query(offset + 0) = left_pos.x;
    query(offset + 1) = left_pos.z;
    query(offset + 0) = right_pos.x;
    query(offset + 1) = right_pos.z;
    offset += 4;

}

void query_compute_other_rotation_feature(
    slice1d<float> query, 
    int& offset, 
    const vec3 root_position,
    const vec3 root_position2,
    const quat root_rotation, 
    const quat root_rotation2,
    bool human_close)
{
    if (human_close){
        vec3 rot = quat_inv_mul_vec3(root_rotation, quat_mul_vec3(root_rotation2, vec3(0, 0, 1)));

        query(offset + 0) = rot.x;
        query(offset + 1) = rot.z;
    }else {
        query(offset + 0) = 0.0;
        query(offset + 1) = 0.0;
    }
    
    
    offset += 2;
}

void query_compute_other_velocity_feature(
    slice1d<float> query, 
    int& offset, 
    const quat root_rotation,
    // const vec3 root_position,
    vec3& velocity)
    // const vec3 left_hand_position,
    // const vec3 right_hand_position)
{
    vec3 vel = quat_inv_mul_vec3(root_rotation, velocity);
    
    // vec3 left_hand = left_hand_position;
    // left_hand.y = 0.0f;
    // vec3 right_hand = right_hand_position;
    // right_hand.y = 0.0f;

    // float pushing_threshold = 0.25f;
    // if(length(left_hand - root_position) < pushing_threshold || length(right_hand - root_position) < pushing_threshold){
    //     query(offset + 0) = vel.x;
    //     query(offset + 1) = vel.z;
    // }

    query(offset + 0) = vel.x;
    query(offset + 1) = vel.z;
    
    offset += 2;
}

void query_compute_other_hand_velocity_feature(
    slice1d<float> query, 
    int& offset, 
    const quat root_rotation,
    vec3& left_hand_velocity,
    vec3& right_hand_velocity)
{
    vec3 left_vel = quat_inv_mul_vec3(root_rotation, left_hand_velocity);
    vec3 right_vel = quat_inv_mul_vec3(root_rotation, right_hand_velocity);

    query(offset + 0) = left_vel.x;
    query(offset + 1) = left_vel.z;
    query(offset + 2) = right_vel.x;
    query(offset + 3) = right_vel.z;
    
    offset += 4;
}

// void query_compute_other_velocity_feature(
//     slice1d<float> query, 
//     int& offset, 
//     const quat root_rotation,
//     const quat root_rotation2,
//     const vec3 
//     vec3& velocity)
// {
//     vec3 vel = quat_inv_mul_vec3(root_rotation, velocity);

//     query(offset + 0) = vel.x;
//     query(offset + 1) = vel.z;
    
//     offset += 2;
// }

// void query_compute_foot_contact_feature(
//     slice1d<float> query,
//     int& offset,
//     const array1d<bool> contact_states,
//     const vec3 l_contact_positions,
//     const vec3 r_contact_positions,
//     const slice1d<float> features,
//     const vector<float> terrain_y)
// {
//     query(offset + 0) = contact_states(0);
//     query(offset + 1) = contact_states(1);

//     // query(offset + 0) = false;
//     // query(offset + 1) = false;
 
//     // vec3 l_foot_pos = {l_contact_positions.x, l_contact_positions.y, l_contact_positions.z};
//     // vec3 r_foot_pos = {r_contact_positions.x, r_contact_positions.y, r_contact_positions.z};
 
//     // float height_l = findClosestY(terrain_x, terrain_z, terrain_y, l_foot_pos.x, l_foot_pos.z);
//     // float height_r = findClosestY(terrain_x, terrain_z, terrain_y, r_foot_pos.x, r_foot_pos.z);
 
//     // if (height_l <= (l_foot_pos.y + 0.02f) && height_l + 0.5f > l_foot_pos.y){
//     //     query(offset + 0) = true;
//     //     // cout << "True" << endl;
//     // }

//     // if (height_r <= (r_foot_pos.y + 0.02f) && height_r + 0.5f > r_foot_pos.y){
//     //     query(offset + 1) = true;
//     // }
 
//     offset += 2;
// }

void query_compute_pushing_feature(
    slice1d<float> query, 
    int& offset,
    const slice1d<float> features,
    const slice1d<float> features_offset,
    const slice1d<float> features_scale,
    slice1d<float> query_pushed,
    const slice1d<float> features_pushed,
    const slice1d<float> features_pushed_offset,
    const slice1d<float> features_pushed_scale,
    const slice1d<vec3> root_position,
    const slice1d<quat> root_rotation,
    const slice1d<vec3> root_velocity,
    const vec3 human_position,
    const quat human_rotation,
    vec3& human_velocity,
    const slice1d<int> db_bone_parents,
    std::chrono::steady_clock::time_point& pushing_state_changed_time,
    const int pushing_milliseconds)
{
    vec3 human_relative_position = quat_inv_mul_vec3(root_rotation(0), human_position - root_position(0));
    vec3 human_relative_direction = quat_inv_mul_vec3(root_rotation(0), quat_mul_vec3(human_rotation, vec3(0, 0, 1)));
    vec3 human_relative_velocity = quat_inv_mul_vec3(root_rotation(0), human_velocity);


    query(offset + 0) = human_relative_position.x;
    query(offset + 1) = human_relative_position.z;
    query(offset + 2) = human_relative_direction.x;
    query(offset + 3) = human_relative_direction.z;


    array1d<vec3> g_bone_positions(db_bone_parents.size);
    array1d<quat> g_bone_rotations(db_bone_parents.size);

    forward_kinematics_full(
        g_bone_positions,
        g_bone_rotations,
        root_position,
        root_rotation,
        db_bone_parents);

    g_bone_positions(Bone_LeftHand).y = 0.0f;
    g_bone_positions(Bone_RightHand).y = 0.0f;

    float pushing_threshold = 0.5f;

    if (length(human_relative_position) < pushing_threshold || length(g_bone_positions(Bone_LeftHand) - human_position) < pushing_threshold / 2 || length(g_bone_positions(Bone_RightHand) - human_position) < pushing_threshold / 2)
    {
        pushing_state_changed_time = std::chrono::steady_clock::now();
        query(offset + 4) = human_relative_velocity.x;
        query(offset + 5) = human_relative_velocity.z;
        // query(offset + 4) = features(offset + 4) * features_scale(offset + 4) + features_offset(offset + 4);
        // query(offset + 5) = features(offset + 5) * features_scale(offset + 5) + features_offset(offset + 5);
    }

    else
    {
        query(offset + 4) = 0.0f;
        query(offset + 5) = 0.0f;
    }
        

    vec3 relative_position_pushed = quat_inv_mul_vec3(human_rotation, root_position(0) - human_position);
    vec3 relative_direction_pushed = quat_inv_mul_vec3(human_rotation, quat_mul_vec3(root_rotation(0), vec3(0, 0, 1)));
    vec3 relative_velocity_pushed = quat_inv_mul_vec3(human_rotation, root_velocity(0));


    query_pushed(0) = relative_position_pushed.x;
    query_pushed(1) = relative_position_pushed.z;
    query_pushed(2) = relative_direction_pushed.x;
    query_pushed(3) = relative_direction_pushed.z;
    query_pushed(4) = relative_velocity_pushed.x;
    query_pushed(5) = relative_velocity_pushed.z;

    human_relative_velocity.x = query(offset + 4);
    human_relative_velocity.z = query(offset + 5);



    
    vec3 pushed_velocity = quat_inv_mul_vec3(human_rotation, quat_mul_vec3(root_rotation(0), human_relative_velocity));

    query_pushed(6) = pushed_velocity.x;
    query_pushed(7) = pushed_velocity.z;

    human_velocity.x = query_pushed(6);
    human_velocity.z = query_pushed(7);

    human_velocity = quat_mul_vec3(human_rotation, human_velocity);

    offset += 6;
}
//--------------------------------------

// Collide against the obscales which are
// essentially bounding boxes of a given size
// vec3 simulation_collide_obstacles(
//     const vec3 prev_pos,
//     const vec3 next_pos,
//     const slice1d<vec3> obstacles_positions,
//     const slice1d<vec3> obstacles_scales,
//     const float radius = 0.6f)
// {
//     vec3 dx = next_pos - prev_pos;
//     vec3 proj_pos = prev_pos;
    
//     // Substep because I'm too lazy to implement CCD
//     int substeps = 1 + (int)(length(dx) * 5.0f);
    
//     for (int j = 0; j < substeps; j++)
//     {
//         proj_pos = proj_pos + dx / substeps;
        
//         // for (int i = 0; i < obstacles_positions.size; i++)
//         // {
//         //     // Find nearest point inside obscale and push out
//         //     vec3 nearest = clamp(proj_pos, 
//         //       obstacles_positions(i) - 0.5f * obstacles_scales(i),
//         //       obstacles_positions(i) + 0.5f * obstacles_scales(i));

//         //     if (length(nearest - proj_pos) < radius)
//         //     {
//         //         proj_pos = radius * normalize(proj_pos - nearest) + nearest;
//         //     }
//         // }
//     } 
    
//     return proj_pos;
// }

vec3 simulation_collide_obstacles(
    const vec3 prev_pos,
    const vec3 next_pos,
    const slice1d<vec3> obstacles_positions,
    const slice1d<vec3> obstacles_scales,
    const slice1d<vec3> human_positions,
    const slice1d<vec3> human_scales,
    const float radius = 0.3f)
{
    vec3 dx = next_pos - prev_pos;
    vec3 proj_pos = prev_pos;
    
    // Substep because I'm too lazy to implement CCD
    int substeps = 1 + (int)(length(dx) * 5.0f);
    
    for (int j = 0; j < substeps; j++)
    {
        proj_pos = proj_pos + dx / substeps;

        for (int i = 0; i < human_positions.size; i++)
        {
            vec3 nearest = clamp(proj_pos, 
                human_positions(i) - 0.5f * human_scales(0),
                human_positions(i) + 0.5f * human_scales(0));

            if (length(nearest - proj_pos) < radius)
            {
                proj_pos = radius * normalize(proj_pos - nearest) + nearest;
            }
        }
    } 

    return proj_pos;
}

vec3 simulation_terrain(
    const vec3 prev_pos,
    const vec3 next_pos,
    const std::vector<float>& terrain_y,
    const float radius = 0.7f)
{
    vec3 dx = next_pos - prev_pos;
    vec3 proj_pos = prev_pos;
    
    // Substep because I'm too lazy to implement CCD
    int substeps = 1 + (int)(length(dx) * 5.0f);
    
    for (int j = 0; j < substeps; j++)
    {
        proj_pos = proj_pos + dx / substeps; 
        float height = abs(findClosestY(terrain_y, proj_pos.x, proj_pos.z) - prev_pos.y);     

        if (height > 1.5)
        {
            // printf("to high");
            // proj_pos = radius * normalize(proj_pos - nearest) + nearest;
            // proj_pos =  prev_pos;
        }

    } 
    
    return proj_pos;
}

// Taken from https://theorangeduck.com/page/spring-roll-call#controllers
void simulation_positions_update(
    vec3& position, 
    vec3& velocity, 
    vec3& acceleration, 
    const vec3 desired_velocity, 
    const float halflife, 
    const float dt,
    const slice1d<vec3> obstacles_positions,
    const slice1d<vec3> obstacles_scales,
    const slice1d<vec3> human_positions,
    const slice1d<vec3> human_scales,
    vector<float> terrain_y)
{
    float y = halflife_to_damping(halflife) / 2.0f; 
    vec3 j0 = velocity - desired_velocity;
    vec3 j1 = acceleration + j0*y;
    float eydt = fast_negexpf(y*dt);
    
    vec3 position_prev = position;

    float map_y = findClosestY(terrain_y, position.x, position.z);
    float gravity = 5.0f;
    if (position.y > map_y) acceleration.y -= gravity;

    position = eydt*(((-j1)/(y*y)) + ((-j0 - j1*dt)/y)) + 
        (j1/(y*y)) + j0/y + desired_velocity * dt + position_prev;
    velocity = eydt*(j0 + j1*dt) + desired_velocity;
    acceleration = eydt*(acceleration - j1*y*dt);
    
    // float map_y = findClosestY(terrain_y, position.x, position.z);

    // position.y = map_y;

    // position = simulation_terrain(
    //     position_prev, 
    //     position,
    //     terrain_y);

    position = simulation_collide_obstacles(
        position_prev, 
        position,
        obstacles_positions,
        obstacles_scales,
        human_positions,
        human_scales);
}

void simulation_positions_update_trajectory(
    vec3& position, 
    vec3& velocity, 
    vec3& acceleration, 
    const vec3 desired_velocity, 
    const float halflife, 
    const float dt,
    const slice1d<vec3> obstacles_positions,
    const slice1d<vec3> obstacles_scales,
    const slice1d<vec3> human_positions,
    const slice1d<vec3> human_scales,
    vector<float> terrain_y)
{
    float y = halflife_to_damping(halflife) / 2.0f; 
    vec3 j0 = velocity - desired_velocity;
    vec3 j1 = acceleration + j0*y;
    float eydt = fast_negexpf(y*dt);
    
    vec3 position_prev = position;

    position = eydt*(((-j1)/(y*y)) + ((-j0 - j1*dt)/y)) + 
        (j1/(y*y)) + j0/y + desired_velocity * dt + position_prev;
    velocity = eydt*(j0 + j1*dt) + desired_velocity;
    acceleration = eydt*(acceleration - j1*y*dt);
    
    float map_y = findClosestY(terrain_y, position.x, position.z);

    position.y = map_y;

    // position = simulation_terrain(
    //     position_prev, 
    //     position,
    //     terrain_y);
    position = simulation_collide_obstacles(
        position_prev, 
        position,
        obstacles_positions,
        obstacles_scales,
        human_positions,
        human_scales);
}

void simulation_rotations_update(
    quat& rotation, 
    vec3& angular_velocity, 
    const quat desired_rotation, 
    const float halflife, 
    const float dt)
{
    simple_spring_damper_exact(
        rotation, 
        angular_velocity, 
        desired_rotation, 
        halflife, dt);
}

// Predict what the desired velocity will be in the 
// future. Here we need to use the future trajectory 
// rotation as well as predicted future camera 
// position to find an accurate desired velocity in 
// the world space
void trajectory_desired_velocities_predict(
  slice1d<vec3> desired_velocities,
  const slice1d<quat> trajectory_rotations,
  const vec3 desired_velocity,
  const float camera_azimuth,
  const vec3 gamepadstick_left,
  const vec3 gamepadstick_right,
  const bool desired_strafe,
  const float fwrd_speed,
  const float side_speed,
  const float back_speed,
  const float dt)
{
    desired_velocities(0) = desired_velocity;
    
    for (int i = 1; i < desired_velocities.size; i++)
    {
        desired_velocities(i) = desired_velocity_update(
            gamepadstick_left,
            orbit_camera_update_azimuth(
                camera_azimuth, gamepadstick_right, desired_strafe, i * dt),
            trajectory_rotations(i),
            fwrd_speed,
            side_speed,
            back_speed);
    }
}

void trajectory_positions_predict(
    slice1d<vec3> positions, 
    slice1d<vec3> velocities, 
    slice1d<vec3> accelerations, 
    const vec3 position, 
    const vec3 velocity, 
    const vec3 acceleration, 
    const slice1d<vec3> desired_velocities, 
    const float halflife,
    const float dt,
    const slice1d<vec3> obstacles_positions,
    const slice1d<vec3> obstacles_scales,
    const slice1d<vec3> human_positions,
    const slice1d<vec3> human_scales,
    const vector<float> terrain_y)
{
    positions(0) = position;
    velocities(0) = velocity;
    accelerations(0) = acceleration;
    
    for (int i = 1; i < positions.size; i++)
    {
        positions(i) = positions(i-1);
        velocities(i) = velocities(i-1);
        accelerations(i) = accelerations(i-1);
        
        simulation_positions_update_trajectory(
            positions(i), 
            velocities(i), 
            accelerations(i), 
            desired_velocities(i), 
            halflife, 
            dt, 
            obstacles_positions, 
            obstacles_scales,
            human_positions,
            human_scales,
            terrain_y);
    }
}

// Predict desired rotations given the estimated future 
// camera rotation and other parameters
void trajectory_desired_rotations_predict(
  slice1d<quat> desired_rotations,
  const slice1d<vec3> desired_velocities,
  const quat desired_rotation,
  const float camera_azimuth,
  const vec3 gamepadstick_left,
  const vec3 gamepadstick_right,
  const bool desired_strafe,
  const float dt)
{
    desired_rotations(0) = desired_rotation;
    
    for (int i = 1; i < desired_rotations.size; i++)
    {
        desired_rotations(i) = desired_rotation_update(
            desired_rotations(i-1),
            gamepadstick_left,
            gamepadstick_right,
            orbit_camera_update_azimuth(
                camera_azimuth, gamepadstick_right, desired_strafe, i * dt),
            desired_strafe,
            desired_velocities(i));
    }
}

void trajectory_rotations_predict(
    slice1d<quat> rotations, 
    slice1d<vec3> angular_velocities, 
    const quat rotation, 
    const vec3 angular_velocity, 
    const slice1d<quat> desired_rotations, 
    const float halflife,
    const float dt)
{
    rotations.set(rotation);
    angular_velocities.set(angular_velocity);
    
    for (int i = 1; i < rotations.size; i++)
    {
        simulation_rotations_update(
            rotations(i), 
            angular_velocities(i), 
            desired_rotations(i), 
            halflife, 
            i * dt);
    }
}

//--------------------------------------

void contact_reset(
    bool& contact_state,
    bool& contact_lock,
    vec3& contact_position,
    vec3& contact_velocity,
    vec3& contact_point,
    vec3& contact_target,
    vec3& contact_offset_position,
    vec3& contact_offset_velocity,
    const vec3 input_contact_position,
    const vec3 input_contact_velocity,
    const bool input_contact_state)
{
    contact_state = false;
    contact_lock = false;
    contact_position = input_contact_position;
    contact_velocity = input_contact_velocity;
    contact_point = input_contact_position;
    contact_target = input_contact_position;
    contact_offset_position = vec3();
    contact_offset_velocity = vec3();
}

void contact_update(
    bool& contact_state,
    bool& contact_lock,
    vec3& contact_position,
    vec3& contact_velocity,
    vec3& contact_point,
    vec3& contact_target,
    vec3& contact_offset_position,
    vec3& contact_offset_velocity,
    const vec3 input_contact_position,
    const bool input_contact_state,
    const float unlock_radius,
    const float foot_height,
    const float halflife,
    const float dt,
    const vector<float> terrain_y,
    const float eps=1e-8)
{
    // First compute the input contact position velocity via finite difference
    vec3 input_contact_velocity = 
        (input_contact_position - contact_target) / (dt + eps);    
    contact_target = input_contact_position;
    
    // Update the inertializer to tick forward in time
    inertialize_update(
        contact_position,
        contact_velocity,
        contact_offset_position,
        contact_offset_velocity,
        // If locked we feed the contact point and zero velocity, 
        // otherwise we feed the input from the animation
        contact_lock ? contact_point : input_contact_position,
        contact_lock ?        vec3() : input_contact_velocity,
        halflife,
        dt);
    
    // If the contact point is too far from the current input position 
    // then we need to unlock the contact
    bool unlock_contact = contact_lock && (
        length(contact_point - input_contact_position) > unlock_radius);
    
    // If the contact was previously inactive but is now active we 
    // need to transition to the locked contact state
    if (!contact_state && input_contact_state)
    {
        // Contact point is given by the current position of 
        // the foot projected onto the ground plus foot height
        contact_lock = true;
        contact_point = contact_position;
        // contact_point.y = foot_height;
        contact_point.y = findClosestY(terrain_y, contact_point.x, contact_point.z);
        
        inertialize_transition(
            contact_offset_position,
            contact_offset_velocity,
            input_contact_position,
            input_contact_velocity,
            contact_point,
            vec3());
    }
    
    // Otherwise if we need to unlock or we were previously in 
    // contact but are no longer we transition to just taking 
    // the input position as-is
    else if ((contact_lock && contact_state && !input_contact_state) 
         || unlock_contact)
    {
        contact_lock = false;
        
        inertialize_transition(
            contact_offset_position,
            contact_offset_velocity,
            contact_point,
            vec3(),
            input_contact_position,
            input_contact_velocity);
    }
    
    // Update contact state
    contact_state = input_contact_state;
}
/*
void contact_update(
    bool& contact_state,
    bool& contact_lock,
    vec3& contact_position,
    vec3& contact_velocity,
    vec3& contact_point,
    vec3& contact_target,
    vec3& contact_offset_position,
    vec3& contact_offset_velocity,
    vec3 input_contact_position,
    const bool input_contact_state,
    const float unlock_radius,
    const float foot_height,
    const float halflife,
    const float dt,
    const vector<float> terrain_y,
    const float eps=1e-8)
{
    // Update contact position height
    input_contact_position.y = maxf(input_contact_position.y, findClosestY(terrain_y, input_contact_position.x, input_contact_position.z) + foot_height);
    contact_target.y = maxf(contact_target.y, findClosestY(terrain_y, contact_target.x, contact_target.z) + foot_height);

    // First compute the input contact position velocity via finite difference
    vec3 input_contact_velocity = 
        (input_contact_position - contact_target) / (dt + eps);    
    contact_target = input_contact_position;
    
    // Update the inertializer to tick forward in time
    inertialize_update(
        contact_position,
        contact_velocity,
        contact_offset_position,
        contact_offset_velocity,
        // If locked we feed the contact point and zero velocity, 
        // otherwise we feed the input from the animation
        contact_lock ? contact_point : input_contact_position,
        contact_lock ?        vec3() : input_contact_velocity,
        halflife,
        dt);
    
    // If the contact point is too far from the current input position 
    // then we need to unlock the contact
    bool unlock_contact = contact_lock && (
        length(contact_point - input_contact_position) > unlock_radius);
    
    // If the contact was previously inactive but is now active we 
    // need to transition to the locked contact state
    if (!contact_state && input_contact_state)
    {
        // Contact point is given by the current position of 
        // the foot projected onto the ground plus foot height
        contact_lock = true;
        contact_point = contact_position;
        // contact_point.y = foot_height;
        contact_point.y = maxf(contact_point.y, findClosestY(terrain_y, contact_point.x, contact_point.z) + foot_height);

        inertialize_transition(
            contact_offset_position,
            contact_offset_velocity,
            input_contact_position,
            input_contact_velocity,
            contact_point,
            vec3());
    }
    
    // Otherwise if we need to unlock or we were previously in 
    // contact but are no longer we transition to just taking 
    // the input position as-is
    else if ((contact_lock && contact_state && !input_contact_state) 
         || unlock_contact)
    {
        contact_lock = false;
        contact_point.y = maxf(contact_point.y, findClosestY(terrain_y, contact_point.x, contact_point.z) + foot_height);

        inertialize_transition(
            contact_offset_position,
            contact_offset_velocity,
            contact_point,
            vec3(),
            input_contact_position,
            input_contact_velocity);
    }
    
    // Update contact state
    contact_state = input_contact_state;
}*/

//--------------------------------------

// Rotate a joint to look toward some 
// given target position
void ik_look_at(
    quat& bone_rotation,
    const quat global_parent_rotation,
    const quat global_rotation,
    const vec3 global_position,
    const vec3 child_position,
    const vec3 target_position,
    const float eps = 1e-5f)
{
    vec3 curr_dir = normalize(child_position - global_position);
    vec3 targ_dir = normalize(target_position - global_position);

    if (fabs(1.0f - dot(curr_dir, targ_dir) > eps))
    {
        bone_rotation = quat_inv_mul(global_parent_rotation, 
            quat_mul(quat_between(curr_dir, targ_dir), global_rotation));
    }
}

// Basic two-joint IK in the style of https://theorangeduck.com/page/simple-two-joint
// Here I add a basic "forward vector" which acts like a kind of pole-vetor
// to control the bending direction
void ik_two_bone(
    quat& bone_root_lr, 
    quat& bone_mid_lr,
    const vec3 bone_root, 
    const vec3 bone_mid, 
    const vec3 bone_end, 
    const vec3 target, 
    const vec3 fwd,
    const quat bone_root_gr, 
    const quat bone_mid_gr,
    const quat bone_par_gr,
    const float max_length_buffer) {
    
    float max_extension = 
        length(bone_root - bone_mid) + 
        length(bone_mid - bone_end) - 
        max_length_buffer;
    
    vec3 target_clamp = target;
    if (length(target - bone_root) > max_extension)
    {
        target_clamp = bone_root + max_extension * normalize(target - bone_root);
    }
    
    vec3 axis_dwn = normalize(bone_end - bone_root);
    vec3 axis_rot = normalize(cross(axis_dwn, fwd));

    vec3 a = bone_root;
    vec3 b = bone_mid;
    vec3 c = bone_end;
    vec3 t = target_clamp;
    
    float lab = length(b - a);
    float lcb = length(b - c);
    float lat = length(t - a);

    float ac_ab_0 = acosf(clampf(dot(normalize(c - a), normalize(b - a)), -1.0f, 1.0f));
    float ba_bc_0 = acosf(clampf(dot(normalize(a - b), normalize(c - b)), -1.0f, 1.0f));

    float ac_ab_1 = acosf(clampf((lab * lab + lat * lat - lcb * lcb) / (2.0f * lab * lat), -1.0f, 1.0f));
    float ba_bc_1 = acosf(clampf((lab * lab + lcb * lcb - lat * lat) / (2.0f * lab * lcb), -1.0f, 1.0f));

    quat r0 = quat_from_angle_axis(ac_ab_1 - ac_ab_0, axis_rot);
    quat r1 = quat_from_angle_axis(ba_bc_1 - ba_bc_0, axis_rot);

    vec3 c_a = normalize(bone_end - bone_root);
    vec3 t_a = normalize(target_clamp - bone_root);

    quat r2 = quat_from_angle_axis(
        acosf(clampf(dot(c_a, t_a), -1.0f, 1.0f)),
        normalize(cross(c_a, t_a)));
    
    bone_root_lr = quat_inv_mul(bone_par_gr, quat_mul(r2, quat_mul(r0, bone_root_gr)));
    bone_mid_lr = quat_inv_mul(bone_root_gr, quat_mul(r1, bone_mid_gr));
}

//--------------------------------------

void draw_axis(const vec3 pos, const quat rot, const float scale = 1.0f)
{
    vec3 axis0 = pos + quat_mul_vec3(rot, scale * vec3(1.0f, 0.0f, 0.0f));
    vec3 axis1 = pos + quat_mul_vec3(rot, scale * vec3(0.0f, 1.0f, 0.0f));
    vec3 axis2 = pos + quat_mul_vec3(rot, scale * vec3(0.0f, 0.0f, 1.0f));
    
    DrawLine3D(to_Vector3(pos), to_Vector3(axis0), RED);
    DrawLine3D(to_Vector3(pos), to_Vector3(axis1), GREEN);
    DrawLine3D(to_Vector3(pos), to_Vector3(axis2), BLUE);
}
void draw_features(const slice1d<float> features, const vec3 pos, const quat rot, const Color color)
{
    vec3 lfoot_pos = quat_mul_vec3(rot, vec3(features( 0), features( 1), features( 2))) + pos;
    vec3 rfoot_pos = quat_mul_vec3(rot, vec3(features( 3), features( 4), features( 5))) + pos;
    vec3 lfoot_vel = quat_mul_vec3(rot, vec3(features( 6), features( 7), features( 8)));
    vec3 rfoot_vel = quat_mul_vec3(rot, vec3(features( 9), features(10), features(11)));
    
    vec3 traj0_pos = quat_mul_vec3(rot, vec3(features(15), features(16), features(17))) + pos;
    vec3 traj1_pos = quat_mul_vec3(rot, vec3(features(18), features(19), features(20))) + pos;
    vec3 traj2_pos = quat_mul_vec3(rot, vec3(features(21), features(22), features(23))) + pos;
    vec3 traj0_dir = quat_mul_vec3(rot, vec3(features(24),         0.0f, features(25)));
    vec3 traj1_dir = quat_mul_vec3(rot, vec3(features(26),         0.0f, features(27)));
    vec3 traj2_dir = quat_mul_vec3(rot, vec3(features(28),         0.0f, features(29)));

    vec3 chair_pos = quat_mul_vec3(rot, vec3(features(30),         0.0f, features(31))) + pos;
    vec3 chair_dir = quat_mul_vec3(rot, vec3(features(32),         0.0f, features(33)));

    vec3 other_pos = quat_mul_vec3(rot, vec3(features(34),         0.0f, features(35))) + pos;
    vec3 other_dir = quat_mul_vec3(rot, vec3(features(36),         0.0f, features(37)));

    DrawSphereWires(to_Vector3(lfoot_pos), 0.05f, 4, 10, color);
    DrawSphereWires(to_Vector3(rfoot_pos), 0.05f, 4, 10, color);
    DrawSphereWires(to_Vector3(traj0_pos), 0.05f, 4, 10, color);
    DrawSphereWires(to_Vector3(traj1_pos), 0.05f, 4, 10, color);
    DrawSphereWires(to_Vector3(traj2_pos), 0.05f, 4, 10, color);
    DrawSphereWires(to_Vector3(chair_pos), 0.05f, 4, 10, PINK);
    DrawSphereWires(to_Vector3(other_pos), 0.05f, 4, 10, GREEN);
    
    DrawLine3D(to_Vector3(lfoot_pos), to_Vector3(lfoot_pos + 0.1f * lfoot_vel), color);
    DrawLine3D(to_Vector3(rfoot_pos), to_Vector3(rfoot_pos + 0.1f * rfoot_vel), color);
    DrawLine3D(to_Vector3(traj0_pos), to_Vector3(traj0_pos + 0.25f * traj0_dir), color);
    DrawLine3D(to_Vector3(traj1_pos), to_Vector3(traj1_pos + 0.25f * traj1_dir), color);
    DrawLine3D(to_Vector3(traj2_pos), to_Vector3(traj2_pos + 0.25f * traj2_dir), color); 
    DrawLine3D(to_Vector3(chair_pos), to_Vector3(chair_pos + 0.25f * chair_dir), PINK);
    DrawLine3D(to_Vector3(other_pos), to_Vector3(other_pos + 0.25f * other_dir), GREEN);
}


// void draw_features(const slice1d<float> features, const vec3 pos, const quat rot, const Color color)
// {
//     vec3 lfoot_pos = quat_mul_vec3(rot, vec3(features( 0), features( 1), features( 2))) + pos;
//     vec3 rfoot_pos = quat_mul_vec3(rot, vec3(features( 3), features( 4), features( 5))) + pos;
//     // vec3 hip_pos = quat_mul_vec3(rot, vec3(features( 6), features( 7), features( 8))) + pos;
//     vec3 lfoot_vel = quat_mul_vec3(rot, vec3(features( 9), features(10), features(11)));
//     vec3 rfoot_vel = quat_mul_vec3(rot, vec3(features( 12), features(13), features(14)));
//     // vec3 hip_vel = quat_mul_vec3(rot, vec3(features( 15), features(16), features(17)));
    
//     vec3 traj0_pos = quat_mul_vec3(rot, vec3(features(18), features(19), features(20))) + pos;
//     vec3 traj1_pos = quat_mul_vec3(rot, vec3(features(21), features(22), features(23))) + pos;
//     vec3 traj2_pos = quat_mul_vec3(rot, vec3(features(24), features(25), features(26))) + pos;
//     vec3 traj0_dir = quat_mul_vec3(rot, vec3(features(27),         0.0f, features(28)));
//     vec3 traj1_dir = quat_mul_vec3(rot, vec3(features(29),         0.0f, features(30)));
//     vec3 traj2_dir = quat_mul_vec3(rot, vec3(features(31),         0.0f, features(32)));

//     vec3 chair_pos = quat_mul_vec3(rot, vec3(features(33),         0.0f, features(34))) + pos;
//     vec3 chair_dir = quat_mul_vec3(rot, vec3(features(35),         0.0f, features(36)));

//     vec3 other_pos = quat_mul_vec3(rot, vec3(features(37),         0.0f, features(38))) + pos;
//     vec3 other_dir = quat_mul_vec3(rot, vec3(features(39),         0.0f, features(40)));

//     DrawSphereWires(to_Vector3(lfoot_pos), 0.05f, 4, 10, color);
//     DrawSphereWires(to_Vector3(rfoot_pos), 0.05f, 4, 10, color);
//     DrawSphereWires(to_Vector3(hip_pos), 0.1f, 4, 10, color);
//     DrawSphereWires(to_Vector3(traj0_pos), 0.05f, 4, 10, RED);
//     DrawSphereWires(to_Vector3(traj1_pos), 0.05f, 4, 10, GREEN);
//     DrawSphereWires(to_Vector3(traj2_pos), 0.05f, 4, 10, BLUE);
//     DrawSphereWires(to_Vector3(chair_pos), 0.05f, 4, 10, PINK);
//     DrawSphereWires(to_Vector3(other_pos), 0.05f, 4, 10, GREEN);
    
//     DrawLine3D(to_Vector3(lfoot_pos), to_Vector3(lfoot_pos + 0.1f * lfoot_vel), color);
//     DrawLine3D(to_Vector3(rfoot_pos), to_Vector3(rfoot_pos + 0.1f * rfoot_vel), color);
//     DrawLine3D(to_Vector3(hip_pos), to_Vector3(hip_pos + 0.1f * hip_vel), color);
//     DrawLine3D(to_Vector3(traj0_pos), to_Vector3(traj0_pos + 0.25f * traj0_dir), RED);
//     DrawLine3D(to_Vector3(traj1_pos), to_Vector3(traj1_pos + 0.25f * traj1_dir), GREEN);
//     DrawLine3D(to_Vector3(traj2_pos), to_Vector3(traj2_pos + 0.25f * traj2_dir), BLUE); 
//     DrawLine3D(to_Vector3(chair_pos), to_Vector3(chair_pos + 0.25f * chair_dir), PINK);
//     DrawLine3D(to_Vector3(other_pos), to_Vector3(other_pos + 0.25f * other_dir), GREEN);
// }

void draw_features_pushed(const slice1d<float> features, const vec3 pos, const quat rot, const Color color)
{
    vec3 other_pos = quat_mul_vec3(rot, vec3(features(0),         0.0f, features(1))) + pos;
    vec3 other_dir = quat_mul_vec3(rot, vec3(features(2),         0.0f, features(3)));

    DrawSphereWires(to_Vector3(other_pos), 0.05f, 4, 10, BLUE);
    DrawLine3D(to_Vector3(other_pos), to_Vector3(other_pos + 0.25f * other_dir), BLUE); 
}

void draw_trajectory(
    const slice1d<vec3> trajectory_positions, 
    const slice1d<quat> trajectory_rotations, 
    const Color color)
{
    for (int i = 1; i < trajectory_positions.size; i++)
    {
        DrawSphereWires(to_Vector3(trajectory_positions(i)), 0.05f, 4, 10, color);
        DrawLine3D(to_Vector3(trajectory_positions(i)), to_Vector3(
            trajectory_positions(i) + 0.6f * quat_mul_vec3(trajectory_rotations(i), vec3(0, 0, 1.0f))), color);
        DrawLine3D(to_Vector3(trajectory_positions(i-1)), to_Vector3(trajectory_positions(i)), color);
    }
}

void draw_obstacles(
    const slice1d<vec3> obstacles_positions,
    const slice1d<vec3> obstacles_scales)
{
    for (int i = 0; i < obstacles_positions.size; i++)
    {
        vec3 position = vec3(
            obstacles_positions(i).x, 
            obstacles_positions(i).y + 0.5f * obstacles_scales(i).y + 0.01f, 
            obstacles_positions(i).z);
      
        DrawCube(
            to_Vector3(position),
            obstacles_scales(i).x, 
            obstacles_scales(i).y, 
            obstacles_scales(i).z,
            LIGHTGRAY);
            
        DrawCubeWires(
            to_Vector3(position),
            obstacles_scales(i).x, 
            obstacles_scales(i).y, 
            obstacles_scales(i).z,
            GRAY);
    }
}

//--------------------------------------

vec3 adjust_character_position(
    const vec3 character_position,
    const vec3 simulation_position,
    const float halflife,
    const float dt)
{
    // Find the difference in positioning
    vec3 difference_position = simulation_position - character_position;
    
    // Damp that difference using the given halflife and dt
    vec3 adjustment_position = damp_adjustment_exact(
        difference_position,
        halflife,
        dt);
    
    // Add the damped difference to move the character toward the sim
    return adjustment_position + character_position;
}

quat adjust_character_rotation(
    const quat character_rotation,
    const quat simulation_rotation,
    const float halflife,
    const float dt)
{
    // Find the difference in rotation (from character to simulation).
    // Here `quat_abs` forces the quaternion to take the shortest 
    // path and normalization is required as sometimes taking 
    // the difference between two very similar rotations can 
    // introduce numerical instability
    quat difference_rotation = quat_abs(quat_normalize(
        quat_mul_inv(simulation_rotation, character_rotation)));
    
    // Damp that difference using the given halflife and dt
    quat adjustment_rotation = damp_adjustment_exact(
        difference_rotation,
        halflife,
        dt);
    
    // Apply the damped adjustment to the character
    return quat_mul(adjustment_rotation, character_rotation);
}

vec3 adjust_character_position_by_velocity(
    const vec3 character_position,
    const vec3 character_velocity,
    const vec3 simulation_position,
    const float max_adjustment_ratio,
    const float halflife,
    const float dt)
{
    // Find and damp the desired adjustment
    vec3 adjustment_position = damp_adjustment_exact(
        simulation_position - character_position,
        halflife,
        dt);
    
    // If the length of the adjustment is greater than the character velocity 
    // multiplied by the ratio then we need to clamp it to that length
    float max_length = max_adjustment_ratio * length(character_velocity) * dt;
    
    if (length(adjustment_position) > max_length)
    {
        adjustment_position = max_length * normalize(adjustment_position);
    }
    
    // Apply the adjustment
    return adjustment_position + character_position;
}

quat adjust_character_rotation_by_velocity(
    const quat character_rotation,
    const vec3 character_angular_velocity,
    const quat simulation_rotation,
    const float max_adjustment_ratio,
    const float halflife,
    const float dt)
{
    // Find and damp the desired rotational adjustment
    quat adjustment_rotation = damp_adjustment_exact(
        quat_abs(quat_normalize(quat_mul_inv(
            simulation_rotation, character_rotation))),
        halflife,
        dt);
    
    // If the length of the adjustment is greater than the angular velocity 
    // multiplied by the ratio then we need to clamp this adjustment
    float max_length = max_adjustment_ratio *
        length(character_angular_velocity) * dt;
    
    if (length(quat_to_scaled_angle_axis(adjustment_rotation)) > max_length)
    {
        // To clamp can convert to scaled angle axis, rescale, and convert back
        adjustment_rotation = quat_from_scaled_angle_axis(max_length * 
            normalize(quat_to_scaled_angle_axis(adjustment_rotation)));
    }
    
    // Apply the adjustment
    return quat_mul(adjustment_rotation, character_rotation);
}

//--------------------------------------

vec3 clamp_character_position(
    const vec3 character_position,
    const vec3 simulation_position,
    const float max_distance)
{
    // If the character deviates too far from the simulation 
    // position we need to clamp it to within the max distance
    if (length(character_position - simulation_position) > max_distance)
    {
        return max_distance * 
            normalize(character_position - simulation_position) + 
            simulation_position;
    }
    else
    {
        return character_position;
    }
}
  
quat clamp_character_rotation(
    const quat character_rotation,
    const quat simulation_rotation,
    const float max_angle)
{
    // If the angle between the character rotation and simulation 
    // rotation exceeds the threshold we need to clamp it back
    if (quat_angle_between(character_rotation, simulation_rotation) > max_angle)
    {
        // First, find the rotational difference between the two
        quat diff = quat_abs(quat_mul_inv(
            character_rotation, simulation_rotation));
        
        // We can then decompose it into angle and axis
        float diff_angle; vec3 diff_axis;
        quat_to_angle_axis(diff, diff_angle, diff_axis);
        
        // We then clamp the angle to within our bounds
        diff_angle = clampf(diff_angle, -max_angle, max_angle);
        
        // And apply back the clamped rotation
        return quat_mul(
          quat_from_angle_axis(diff_angle, diff_axis), simulation_rotation);
    }
    else
    {
        return character_rotation;
    }
}

//--------------------------------------

void update_callback(void* args)
{
    ((std::function<void()>*)args)->operator()();
}


int main(void)
{   
    setTerrainParameters(world);
   
    bool matching_blue = false;

    // Init Window
    
    // const int screen_width = 1280;
    // const int screen_height = 720;

    const int screen_width = 1920;
    const int screen_height = 1080;
    
    SetConfigFlags(FLAG_VSYNC_HINT);
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(screen_width, screen_height, "Motion Matching");
    SetTargetFPS(60);
    
    // Camera

    Camera3D camera = { 0 };
    camera.position = (Vector3){ 0.0f, 10.0f, 10.0f };
    camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    float camera_azimuth = 0.0f;
    float camera_altitude = 0.4f;
    float camera_distance = 4.0f;

    Camera3D camera_shadow_map = { 0 };
    camera_shadow_map.fovy = 45.0f;
    camera_shadow_map.target = (Vector3){0.0f, 0.0f, 0.0f};
    camera_shadow_map.position = (Vector3){0.0f, 10.0f, 0.0f};
    camera_shadow_map.up = (Vector3){0.0f, 0.0f, -1.0f};
    camera_shadow_map.projection = CAMERA_PERSPECTIVE;
    
    // Scene Obstacles
    
    array1d<vec3> obstacles_positions(0);
    array1d<vec3> obstacles_scales(0);
    
    // obstacles_positions(0) = vec3(5.0f, 0.0f, 6.0f);
    // obstacles_positions(1) = vec3(-3.0f, 0.0f, -5.0f);
    // obstacles_positions(2) = vec3(-8.0f, 0.0f, 3.0f);
    
    // obstacles_scales(0) = vec3(2.0f, 1.0f, 5.0f);
    // obstacles_scales(1) = vec3(4.0f, 1.0f, 4.0f);
    // obstacles_scales(2) = vec3(2.0f, 1.0f, 2.0f);
    


    ifstream file("resources/PFNN_terrain_data.txt");
    if (!file.is_open()) {
        cerr << "Failed to open file." << endl;
        return 1;
    }

    vector<float> terrain_y;
    string line1;
    while (getline(file, line1)) {
        istringstream iss(line1);
        float height;
        char comma;
        if (!(iss >> height)) {
            cerr << "Error reading line." << endl;
            return 1;
        }
        terrain_y.push_back(height);
    }

    float max_height = *max_element(terrain_y.begin(), terrain_y.end());
    float min_height = *min_element(terrain_y.begin(), terrain_y.end());

    Image image = LoadImage("./resources/PFNN_heightmap.png");     // Load heightmap image (RAM)
    Texture2D texture = LoadTextureFromImage(image);        // Convert image to texture (VRAM)
    // setTerrainParameters(1);

    Shader ground_plane_shader = LoadShader("./resources/checkerboard.vs", "./resources/checkerboard.fs");
    Mesh mesh = GenMeshHeightmap(image, (Vector3){ x_max *2, (max_height - min_height), z_max*2 }); // Generate heightmap mesh (RAM and VRAM)
    Model model = LoadModelFromMesh(mesh);                  // Load model from generated mesh
    model.materials[0].shader = ground_plane_shader;

    Vector3 mapPosition = { -x_max, min_height, -z_max };

    UnloadImage(image);             // Unload heightmap image from RAM, already uploaded to VRAM

    // Ground Plane
    
    // Shader ground_plane_shader = LoadShader("./resources/checkerboard.vs", "./resources/checkerboard.fs");
    // Mesh ground_plane_mesh = GenMeshPlane(20.0f, 20.0f, 10, 10);
    // Model ground_plane_model = LoadModelFromMesh(ground_plane_mesh);
    // ground_plane_model.materials[0].shader = ground_plane_shader;

    
    // Character
    
    character character_data;
    character_load(character_data, "./resources/character.bin");
    // character_load(character_data2, "./resources/character.bin");
    
    Shader character_shader = LoadShader("./resources/character.vs", "./resources/character.fs");
    Mesh character_mesh = make_character_mesh(character_data);
    // Mesh character_mesh2 = make_character_mesh(character_data2);
    Model character_model = LoadModelFromMesh(character_mesh);
    
    // Model character_model1 = LoadModelFromMesh(character_mesh2);
    // Model character_model2 = LoadModelFromMesh(character_mesh2);
    // Model character_model3 = LoadModelFromMesh(character_mesh2);
    // Model character_model4 = LoadModelFromMesh(character_mesh2);
    // Model character_model5 = LoadModelFromMesh(character_mesh2);

    character_model.materials[0].shader = character_shader;
    
    // character_model1.materials[0].shader = character_shader;
    // character_model2.materials[0].shader = character_shader;
    // character_model3.materials[0].shader = character_shader;
    // character_model4.materials[0].shader = character_shader;
    // character_model5.materials[0].shader = character_shader;

    // Chair
    Model chair = LoadModel("resources/chair_01.obj");
    chair.materials[0].shader = character_shader;
    
    
    
    // Light

    Shader shader = LoadShader(TextFormat("resources/shaders/glsl%i/lighting.vs", GLSL_VERSION),
                               TextFormat("resources/shaders/glsl%i/lighting.fs", GLSL_VERSION));
    // shader.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocation(shader, "matModel");
    shader.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(shader, "viewPos");

    int ambientLoc = GetShaderLocation(shader, "ambient");
    float ambientColor[4] = {0.1f, 0.1f, 0.1f, 1.0f };
    SetShaderValue(shader, ambientLoc, ambientColor, SHADER_UNIFORM_VEC4);

    
    // SetShaderValue(shader, ambientLoc, (float[4]){ 0.1f, 0.1f, 0.1f, 1.0f }, SHADER_UNIFORM_VEC4);

    // Create lights
    Light lights[MAX_LIGHTS] = { 0 };
    lights[0] = CreateLight(LIGHT_POINT, (Vector3){ 0.0f, 10.0f, 5.0f }, Vector3Zero(), WHITE, shader);
    // Light light_1 = CreateLight(LIGHT_POINT, (Vector3){ 0, 15.f, 15.f}, Vector3Zero(), WHITE, shader);
    // UpdateLightValues(shader, light_1);

    RenderTexture2D shadow_render_texture = LoadRenderTexture(screen_width, screen_height);

    // Assign out lighting shader to model
    // model.materials[0].shader = shader;
    // character_model.materials[0].shader = shader;
    // model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = shadow_render_texture.texture;

    

    // Load Animation Data and build Matching Database

    // float feature_weight_foot_position = 2.5f;
    // float feature_weight_foot_velocity = 2.5f;
    // float feature_weight_hip_velocity = 1.5f;
    // float feature_weight_trajectory_positions = 2.0f;
    // float feature_weight_trajectory_directions = 0.5f;

    // float feature_weight_chair_positions = 5.0f;
    // float feature_weight_chair_directions = 5.0f;

    // float feature_weight_other = 5.0f;

    // float feature_weight_foot_position = 2.5f;
    // float feature_weight_foot_velocity = 2.5f;
    // float feature_weight_hip_velocity = 1.5f;
    // float feature_weight_trajectory_positions = 2.0f;
    // float feature_weight_trajectory_directions = 0.5f;

    // float feature_weight_chair_positions = 5.0f;
    // float feature_weight_chair_directions = 5.0f;

    // float feature_weight_other = 5.0f;

    float feature_weight_foot_position = 2.5f;
    float feature_weight_foot_velocity = 2.5f;
    float feature_weight_hip_velocity = 1.5f;
    float feature_weight_trajectory_positions = 5.0f;
    float feature_weight_trajectory_directions = 2.5f;

    float feature_weight_chair_positions = 4.0f;
    float feature_weight_chair_directions = 6.0f;


    float feature_weight_other = 4.0f;
    
    float feature_motion_idx = 2.0f;
    
    database db;
    database_load(db, "./resources/database_all.bin");

    // database db_terrain_pushing;
    // database_load(db_terrain_pushing, "./resources/database_terrain_pushing.bin");

    // database db_chair;
    // database_load(db_chair, "./resources/database_chair.bin");

    database db2;
    database_load(db2, "./resources/database_pushed.bin");
    
    database_build_matching_features(
        db,
        db2,
        feature_weight_foot_position,
        feature_weight_foot_velocity,
        feature_weight_hip_velocity,
        feature_weight_trajectory_positions,
        feature_weight_trajectory_directions,
        feature_weight_chair_positions,
        feature_weight_chair_directions,
        feature_weight_other);
    
    
    // terrain_database_build_matching_features(
    //     db,
    //     feature_weight_foot_position,
    //     feature_weight_foot_velocity,
    //     feature_weight_hip_velocity,
    //     feature_weight_trajectory_positions,
    //     feature_weight_trajectory_directions);

        
    database_save_matching_features(db, "./resources/features.bin");

    // database_build_matching_features(
    //     db_terrain_pushing,
    //     db2,
    //     feature_weight_foot_position,
    //     feature_weight_foot_velocity,
    //     feature_weight_hip_velocity,
    //     feature_weight_trajectory_positions,
    //     feature_weight_trajectory_directions,
    //     0.0f,
    //     0.0f,
    //     feature_weight_other);
        
    // database_save_matching_features(db_terrain_pushing, "./resources/features_terrain_pushing.bin");

    // database_build_matching_features(
    //     db_chair,
    //     db_chair,
    //     feature_weight_foot_position,
    //     feature_weight_foot_velocity,
    //     feature_weight_hip_velocity,
    //     0.0f,
    //     0.0f,
    //     feature_weight_chair_positions,
    //     feature_weight_chair_directions,
    //     0.0f);
        
    // database_save_matching_features(db_chair, "./resources/features_chair.bin");

    pushed_database_build_matching_features(
        db2,
        db,
        feature_weight_other,
        feature_motion_idx);

    database_save_matching_features(db2, "./resources/features_pushed.bin");
   
    // Pose & Inertializer Data
    
    int frame_index = db.range_starts(0);
    float inertialize_blending_halflife = 0.2f;
    // float inertialize_blending_halflife = 0.03f;

    array1d<vec3> curr_bone_positions = db.bone_positions(frame_index);
    array1d<vec3> curr_bone_velocities = db.bone_velocities(frame_index);
    array1d<quat> curr_bone_rotations = db.bone_rotations(frame_index);
    array1d<vec3> curr_bone_angular_velocities = db.bone_angular_velocities(frame_index);
    array1d<bool> curr_bone_contacts = db.contact_states(frame_index);

    array1d<vec3> trns_bone_positions = db.bone_positions(frame_index);
    array1d<vec3> trns_bone_velocities = db.bone_velocities(frame_index);
    array1d<quat> trns_bone_rotations = db.bone_rotations(frame_index);
    array1d<vec3> trns_bone_angular_velocities = db.bone_angular_velocities(frame_index);
    array1d<bool> trns_bone_contacts = db.contact_states(frame_index);

    array1d<vec3> bone_positions = db.bone_positions(frame_index);
    array1d<vec3> bone_velocities = db.bone_velocities(frame_index);
    array1d<quat> bone_rotations = db.bone_rotations(frame_index);
    array1d<vec3> bone_angular_velocities = db.bone_angular_velocities(frame_index);
    
    array1d<vec3> bone_offset_positions(db.nbones());
    array1d<vec3> bone_offset_velocities(db.nbones());
    array1d<quat> bone_offset_rotations(db.nbones());
    array1d<vec3> bone_offset_angular_velocities(db.nbones());
    
    array1d<vec3> global_bone_positions(db.nbones());
    array1d<vec3> global_bone_velocities(db.nbones());
    array1d<quat> global_bone_rotations(db.nbones());
    array1d<vec3> global_bone_angular_velocities(db.nbones());
    array1d<bool> global_bone_computed(db.nbones());
    
    vec3 transition_src_position;
    quat transition_src_rotation;
    vec3 transition_dst_position;
    quat transition_dst_rotation;

    inertialize_pose_reset(
        bone_offset_positions,
        bone_offset_velocities,
        bone_offset_rotations,
        bone_offset_angular_velocities,
        transition_src_position,
        transition_src_rotation,
        transition_dst_position,
        transition_dst_rotation,
        bone_positions(0),
        bone_rotations(0));
    
    inertialize_pose_update(
        bone_positions,
        bone_velocities,
        bone_rotations,
        bone_angular_velocities,
        bone_offset_positions,
        bone_offset_velocities,
        bone_offset_rotations,
        bone_offset_angular_velocities,
        db.bone_positions(frame_index),
        db.bone_velocities(frame_index),
        db.bone_rotations(frame_index),
        db.bone_angular_velocities(frame_index),
        transition_src_position,
        transition_src_rotation,
        transition_dst_position,
        transition_dst_rotation,
        inertialize_blending_halflife,
        0.0f);

    int frame_index2 = db2.range_starts(0);

    array1d<vec3> curr_bone_positions2 = db2.bone_positions(frame_index2);
    array1d<vec3> curr_bone_velocities2 = db2.bone_velocities(frame_index2);
    array1d<quat> curr_bone_rotations2 = db2.bone_rotations(frame_index2);
    array1d<vec3> curr_bone_angular_velocities2 = db2.bone_angular_velocities(frame_index2);
    array1d<bool> curr_bone_contacts2 = db2.contact_states(frame_index2);

    array1d<vec3> trns_bone_positions2 = db2.bone_positions(frame_index2);
    array1d<vec3> trns_bone_velocities2 = db2.bone_velocities(frame_index2);
    array1d<quat> trns_bone_rotations2 = db2.bone_rotations(frame_index2);
    array1d<vec3> trns_bone_angular_velocities2 = db2.bone_angular_velocities(frame_index2);
    array1d<bool> trns_bone_contacts2 = db2.contact_states(frame_index2);

    array1d<vec3> bone_positions2 = db2.bone_positions(frame_index2);    
    array1d<vec3> bone_velocities2 = db2.bone_velocities(frame_index2);
    array1d<quat> bone_rotations2 = db2.bone_rotations(frame_index2);
    array1d<vec3> bone_angular_velocities2 = db2.bone_angular_velocities(frame_index2);

    array1d<vec3> init_bone_positions = db2.bone_positions(0);
    array1d<quat> init_bone_rotations = db2.bone_rotations(0);

    array1d<vec3> last_bone_positions = db2.bone_positions(0);
    array1d<quat> last_bone_rotations = db2.bone_rotations(0);

    array1d<vec3> syn_bone_positions = db2.bone_positions(0);
    array1d<quat> syn_bone_rotations = db2.bone_rotations(0);
    
    array1d<vec3> bone_offset_positions2(db2.nbones());
    array1d<vec3> bone_offset_velocities2(db2.nbones());
    array1d<quat> bone_offset_rotations2(db2.nbones());
    array1d<vec3> bone_offset_angular_velocities2(db2.nbones());
    
    array1d<vec3> global_bone_positions2(db2.nbones());
    array1d<vec3> global_bone_velocities2(db2.nbones());
    array1d<quat> global_bone_rotations2(db2.nbones());
    array1d<vec3> global_bone_angular_velocities2(db2.nbones());
    array1d<bool> global_bone_computed2(db2.nbones());

    int pushing_milliseconds = 800;

    std::chrono::steady_clock::time_point pushing_state_changed_time = std::chrono::steady_clock::now() - std::chrono::milliseconds(pushing_milliseconds);

    array1d<vec3> human_positions(5);
    array1d<quat> human_rotations(5);
    array1d<vec3> human_velocities(5);
    array1d<vec3> human_scales(1);
    vec3 syn_velocity = vec3(0.0f, 0.0f, 0.0f);

    human_positions(0) = vec3(-5.0f, findClosestY(terrain_y, -5.0f, -10.0f), -10.0f);
    human_rotations(0) = quat();
    human_velocities(0) = vec3(0.0f, 0.0f, 0.0f);

    human_positions(1) = vec3(-18.0f, findClosestY(terrain_y, -18.0f, -2.3f), -2.3f);
    human_rotations(1) = quat();
    human_velocities(1) = vec3(0.0f, 0.0f, 0.0f);

    human_positions(2) = vec3(-25.3f, findClosestY(terrain_y, -25.3f, 6.8f), 6.8f);
    human_rotations(2) = quat();
    human_velocities(2) = vec3(0.0f, 0.0f, 0.0f);

    human_positions(3) = vec3(-9.0f, findClosestY(terrain_y, -9.0f, 2.0f), 2.0f);
    human_rotations(3) = quat();
    human_velocities(3) = vec3(0.0f, 0.0f, 0.0f);

    human_positions(4) = vec3(12.0f, findClosestY(terrain_y, 12.0f, -4.5f), -4.5f);
    human_rotations(4) = quat();
    human_velocities(4) = vec3(0.0f, 0.0f, 0.0f);
    human_scales(0) = vec3(0.6f, 1.7f, 0.3f);


    array1d<vec3> chair_positions(3);
    array1d<float> chair_rotation_angel(3);
    array1d<Color> chair_color(3);

    chair_positions(0) = vec3(-10.0f, findClosestY(terrain_y, -10.0f, -7.6f), -7.6f);
    chair_positions(1) = vec3(-13.3f, findClosestY(terrain_y, -13.3f, -3.0f), -3.0f);
    chair_positions(2) = vec3(11.3f, findClosestY(terrain_y, 11.3f, 3.3f), 3.3f);

    chair_rotation_angel(0) = -45.0;
    chair_rotation_angel(1) = 45.0;
    chair_rotation_angel(2) = 90.0;

    chair_color(0) = ORANGE;
    chair_color(1) = ORANGE;
    chair_color(2) = ORANGE;

    vec3 transition_src_position2;
    quat transition_src_rotation2;
    vec3 transition_dst_position2;
    quat transition_dst_rotation2;
    
    inertialize_pose_reset(
        bone_offset_positions2,
        bone_offset_velocities2,
        bone_offset_rotations2,
        bone_offset_angular_velocities2,
        transition_src_position2,
        transition_src_rotation2,
        transition_dst_position2,
        transition_dst_rotation2,
        bone_positions2(0),
        bone_rotations2(0));
    
    inertialize_pose_update2(
        bone_positions2,
        bone_velocities2,
        bone_rotations2,
        bone_angular_velocities2,
        bone_offset_positions2,
        bone_offset_velocities2,
        bone_offset_rotations2,
        bone_offset_angular_velocities2,
        db2.bone_positions(frame_index2),
        db2.bone_velocities(frame_index2),
        db2.bone_rotations(frame_index2),
        db2.bone_angular_velocities(frame_index2),
        transition_src_position2,
        transition_src_rotation2,
        transition_dst_position2,
        transition_dst_rotation2,
        inertialize_blending_halflife,
        0.0f,
        terrain_y);
        
    // Trajectory & Gameplay Data
    
    float search_time = 0.1f;
    float search_timer = search_time;
    float force_search_timer = search_time;

    float search_time_pushed = 0.1f;
    float search_timer_pushed = search_time_pushed;
    float force_search_timer_pushed = search_time_pushed;
    
    vec3 desired_velocity;
    vec3 desired_velocity_change_curr;
    vec3 desired_velocity_change_prev;
    float desired_velocity_change_threshold = 50.0;
    
    quat desired_rotation;
    vec3 desired_rotation_change_curr;
    vec3 desired_rotation_change_prev;
    float desired_rotation_change_threshold = 50.0;
    
    float desired_gait = 0.0f;
    float desired_gait_velocity = 0.0f;
    
    vec3 simulation_position;
    // simulation_position.y =findClosestY(terrain_y, bone_positions(0).x, bone_positions(0).z);
    // simulation_position = bone_positions(0);
    vec3 simulation_velocity;
    vec3 simulation_acceleration;
    quat simulation_rotation;
    vec3 simulation_angular_velocity;
    
    float simulation_velocity_halflife = 0.27f;
    float simulation_rotation_halflife = 0.27f;

    // // All speeds in m/s
    // float simulation_run_fwrd_speed = 1.75f;
    // float simulation_run_side_speed = 1.75f;
    // float simulation_run_back_speed = 1.75f;
    
    // // float simulation_walk_fwrd_speed = 1.75f;
    // // float simulation_walk_side_speed = 1.5f;
    // // float simulation_walk_back_speed = 1.25f;
    // float simulation_walk_fwrd_speed = 1.0f;
    // float simulation_walk_side_speed = 1.0f;
    // float simulation_walk_back_speed = 1.0f;

    float simulation_run_fwrd_speed = 4.0f;
    float simulation_run_side_speed = 3.0f;
    float simulation_run_back_speed = 2.5f;
    
    float simulation_walk_fwrd_speed = 1.75f;
    float simulation_walk_side_speed = 1.5f;
    float simulation_walk_back_speed = 1.25f;
    
    array1d<vec3> trajectory_desired_velocities(4);
    array1d<quat> trajectory_desired_rotations(4);
    array1d<vec3> trajectory_positions(4);
    array1d<vec3> trajectory_velocities(4);
    array1d<vec3> trajectory_accelerations(4);
    array1d<quat> trajectory_rotations(4);
    array1d<vec3> trajectory_angular_velocities(4);
    
    // Synchronization
    
    bool synchronization_enabled =true;;
    float synchronization_data_factor = 0.8f;
    
    // Adjustment
    
    bool adjustment_enabled = true;
    bool adjustment_by_velocity_enabled = true;
    float adjustment_position_halflife = 0.1f;
    float adjustment_rotation_halflife = 0.2f;
    float adjustment_position_max_ratio = 0.5f;
    float adjustment_rotation_max_ratio = 0.5f;
    
    // Clamping
    
    bool clamping_enabled = true;
    float clamping_max_distance = 0.15f;
    float clamping_max_angle = 0.5f * PIf;
    
    // IK
    
    bool ik_enabled = true;
    float ik_max_length_buffer = 0.015f;
    float ik_foot_height = 0.02f;
    float ik_toe_length = 0.15f;
    float ik_unlock_radius = 0.1f;
    float ik_blending_halflife = 0.1f;
    
    // Contact and Foot Locking data
    
    array1d<int> contact_bones(2);
    contact_bones(0) = Bone_LeftToe;
    contact_bones(1) = Bone_RightToe;
    
    array1d<bool> contact_states(contact_bones.size);
    array1d<bool> contact_locks(contact_bones.size);
    array1d<vec3> contact_positions(contact_bones.size);
    array1d<vec3> contact_velocities(contact_bones.size);
    array1d<vec3> contact_points(contact_bones.size);
    array1d<vec3> contact_targets(contact_bones.size);
    array1d<vec3> contact_offset_positions(contact_bones.size);
    array1d<vec3> contact_offset_velocities(contact_bones.size);

    array1d<bool> contact_states2(contact_bones.size);
    array1d<bool> contact_locks2(contact_bones.size);
    array1d<vec3> contact_positions2(contact_bones.size);
    array1d<vec3> contact_velocities2(contact_bones.size);
    array1d<vec3> contact_points2(contact_bones.size);
    array1d<vec3> contact_targets2(contact_bones.size);
    array1d<vec3> contact_offset_positions2(contact_bones.size);
    array1d<vec3> contact_offset_velocities2(contact_bones.size);
    
    for (int i = 0; i < contact_bones.size; i++)
    {
        vec3 bone_position;
        vec3 bone_velocity;
        quat bone_rotation;
        vec3 bone_angular_velocity;
        
        forward_kinematics_velocity(
            bone_position,
            bone_velocity,
            bone_rotation,
            bone_angular_velocity,
            bone_positions,
            bone_velocities,
            bone_rotations,
            bone_angular_velocities,
            db.bone_parents,
            contact_bones(i));
        
        contact_reset(
            contact_states(i),
            contact_locks(i),
            contact_positions(i),  
            contact_velocities(i),
            contact_points(i),
            contact_targets(i),
            contact_offset_positions(i),
            contact_offset_velocities(i),
            bone_position,
            bone_velocity,
            false);

        vec3 bone_position2;
        vec3 bone_velocity2;
        quat bone_rotation2;
        vec3 bone_angular_velocity2;
        
        forward_kinematics_velocity(
            bone_position2,
            bone_velocity2,
            bone_rotation2,
            bone_angular_velocity2,
            bone_positions2,
            bone_velocities2,
            bone_rotations2,
            bone_angular_velocities2,
            db2.bone_parents,
            contact_bones(i));
        
        contact_reset(
            contact_states2(i),
            contact_locks2(i),
            contact_positions2(i),  
            contact_velocities2(i),
            contact_points2(i),
            contact_targets2(i),
            contact_offset_positions2(i),
            contact_offset_velocities2(i),
            bone_position2,
            bone_velocity2,
            false);
    }
    
    array1d<vec3> adjusted_bone_positions = bone_positions;
    array1d<quat> adjusted_bone_rotations = bone_rotations;

    array1d<vec3> adjusted_bone_positions2 = bone_positions2;
    array1d<quat> adjusted_bone_rotations2 = bone_rotations2;
    
    // Learned Motion Matching
    
    bool lmm_enabled = false;
    
    nnet decompressor, stepper, projector;    
    nnet_load(decompressor, "./resources/decompressor.bin");
    nnet_load(stepper, "./resources/stepper.bin");
    nnet_load(projector, "./resources/projector.bin");

    nnet decompressor2, stepper2, projector2;    
    nnet_load(decompressor2, "./resources/decompressor.bin");
    nnet_load(stepper2, "./resources/stepper.bin");
    nnet_load(projector2, "./resources/projector.bin");
    // nnet_load(decompressor2, "./resources/decompressor_pushed.bin");
    // nnet_load(stepper2, "./resources/stepper_pushed.bin");
    // nnet_load(projector2, "./resources/projector_pushed.bin");

    nnet_evaluation decompressor_evaluation, stepper_evaluation, projector_evaluation;
    decompressor_evaluation.resize(decompressor);
    stepper_evaluation.resize(stepper);
    projector_evaluation.resize(projector);

    nnet_evaluation decompressor_evaluation2, stepper_evaluation2, projector_evaluation2;
    decompressor_evaluation2.resize(decompressor2);
    stepper_evaluation2.resize(stepper2);
    projector_evaluation2.resize(projector2);

    array1d<float> features_proj = db.features(frame_index);
    array1d<float> features_curr = db.features(frame_index);
    array1d<float> latent_proj(32); latent_proj.zero();
    array1d<float> latent_curr(32); latent_curr.zero();

    array1d<float> features_proj2 = db2.features(frame_index2);
    array1d<float> features_curr2 = db2.features(frame_index2);
    array1d<float> latent_proj2(32); latent_proj2.zero();
    array1d<float> latent_curr2(32); latent_curr2.zero();

    bool chair_close = false;
    bool human_close = false;
    bool end_of_animation2 = false;
    bool end_of_anim = false;
    bool push_animation = false;
    bool chair_animation = false;
    int chair_num = 0;
    int chair_num2 = 0;
    int human_num = 0;
    int human_num2 = 0;

    const vector<int> start_frame = {0, 53,  115,  189,  290,  395,  485,  568,  687,  745,  825, 917, 1034, 1236};
    const vector<int> stop_frame = {53,  115,  189,  290,  395,  485,  568,  687,  745,  825, 917, 1034, 1236, 1310};
    
    // Go

    float dt = 1.0f / 60.0f;
    float phaseLight = .0f;

    auto update_func = [&]()
    {
        // Get gamepad stick states
        vec3 gamepadstick_left = gamepad_get_stick(GAMEPAD_STICK_LEFT);
        vec3 gamepadstick_right = gamepad_get_stick(GAMEPAD_STICK_RIGHT);

        if (IsKeyDown(KEY_Q)) camera_distance += 0.2;
        if (IsKeyDown(KEY_W)) camera_distance -= 0.2;
        if ((push_animation || chair_animation) && IsKeyDown(KEY_X)) {
            push_animation = false;
            chair_animation = false;
            synchronization_data_factor = 0.8;
        }
        
        // Get if strafe is desired
        bool desired_strafe = desired_strafe_update();
        
        // Get the desired gait (walk / run)
        desired_gait_update(
            desired_gait,
            desired_gait_velocity,
            dt);
        
        // Get the desired simulation speeds based on the gait
        float simulation_fwrd_speed = lerpf(simulation_run_fwrd_speed, simulation_walk_fwrd_speed, desired_gait);
        float simulation_side_speed = lerpf(simulation_run_side_speed, simulation_walk_side_speed, desired_gait);
        float simulation_back_speed = lerpf(simulation_run_back_speed, simulation_walk_back_speed, desired_gait);
        
        // Get the desired velocity
        vec3 desired_velocity_curr = desired_velocity_update(
            gamepadstick_left,
            camera_azimuth,
            simulation_rotation,
            simulation_fwrd_speed,
            simulation_side_speed,
            simulation_back_speed);
            
        // Get the desired rotation/direction
        quat desired_rotation_curr = desired_rotation_update(
            desired_rotation,
            gamepadstick_left,
            gamepadstick_right,
            camera_azimuth,
            desired_strafe,
            desired_velocity_curr);
        
        // Check if we should force a search because input changed quickly
        desired_velocity_change_prev = desired_velocity_change_curr;
        desired_velocity_change_curr =  (desired_velocity_curr - desired_velocity) / dt;
        desired_velocity = desired_velocity_curr;
        
        desired_rotation_change_prev = desired_rotation_change_curr;
        desired_rotation_change_curr = quat_to_scaled_angle_axis(quat_abs(quat_mul_inv(desired_rotation_curr, desired_rotation))) / dt;
        desired_rotation =  desired_rotation_curr;
        
        bool force_search = false;
        bool force_search_pushed = false;
        // bool blue_changed = false;
        // synchronization_data_factor = 0.8;
        

        if (std::find(stop_frame.begin(), stop_frame.end(), frame_index2) != stop_frame.end()) {
            end_of_animation2 = true;
            push_animation = false;
            force_search = true;
            force_search_pushed = true;
            // human_num = (human_num + 1) % 5;
            human_num2++;
        }
        
        if(chair_animation && end_of_anim){
            chair_animation = false;
            chair_color(chair_num) = GREEN;
            chair_num2++;
            force_search = true;
            synchronization_data_factor = 0.8;
        }
        
        if (length(bone_positions(0) - chair_positions(0)) < 2.5f){
            // synchronization_data_factor = 1.0;
            chair_num = 0;
            chair_close = true;
            
        } else if (length(bone_positions(0) - chair_positions(1)) < 2.5f){
            chair_num = 1;
            chair_close = true;
            
        }else if (length(bone_positions(0) - chair_positions(2)) < 2.5f){
            chair_num = 2;
            chair_close = true;
            
        }
        else {
            // synchronization_data_factor = 0.8;
            chair_close = false;
        }
        

        if (length(bone_positions(0) - human_positions(0)) < 2.0f){
            // synchronization_data_factor = 1.0;
            human_num = 0;
            human_close = true;
            
        } else if (length(bone_positions(0) - human_positions(1)) < 2.0f){
            // synchronization_data_factor = 1.0;
            human_num = 1;
            human_close = true;
            
        } else if (length(bone_positions(0) - human_positions(2)) < 2.0f){
            // synchronization_data_factor = 1.0;
            human_num = 2;
            human_close = true;
            
        } else if (length(bone_positions(0) - human_positions(3)) < 2.0f){
            // synchronization_data_factor = 1.0;
            human_num = 3;
            human_close = true;
            
        } else if (length(bone_positions(0) - human_positions(4)) < 2.0f){
            // synchronization_data_factor = 1.0;
            human_num = 4;
            human_close = true;
            
        }
        else {
            // synchronization_data_factor = 0.8;
            human_close = false;
        }

        if(human_close && IsKeyDown(KEY_S)) push_animation = true;
        if(chair_close && IsKeyDown(KEY_S)) {
            synchronization_data_factor = 0.9;
            chair_animation = true;
        }
        // if(!human_close && end_of_anim2) blue_changed = true;
        // else blue_changed = false;


        if (force_search_timer <= 0.0f && (
            (length(desired_velocity_change_prev) >= desired_velocity_change_threshold && 
             length(desired_velocity_change_curr)  < desired_velocity_change_threshold)
        ||  (length(desired_rotation_change_prev) >= desired_rotation_change_threshold && 
             length(desired_rotation_change_curr)  < desired_rotation_change_threshold)))
        {
            force_search = true;
            force_search_timer = search_time;
        }
        else if (force_search_timer > 0)
        {
            force_search_timer -= dt;
        }

        if(force_search_timer_pushed <= 0.0f ){ // && !end_of_animation2
            force_search_pushed = true;
            force_search_timer_pushed = search_time_pushed;
        }
        else if(force_search_timer_pushed > 0){
            force_search_timer_pushed -= dt;
        }

        // force_search_timer_pushed -= dt;
        
        // Predict Future Trajectory
        
        trajectory_desired_rotations_predict(
          trajectory_desired_rotations,
          trajectory_desired_velocities,
          desired_rotation,
          camera_azimuth,
          gamepadstick_left,
          gamepadstick_right,
          desired_strafe,
          15.0f * dt);
        
        trajectory_rotations_predict(
            trajectory_rotations,
            trajectory_angular_velocities,
            simulation_rotation,
            simulation_angular_velocity,
            trajectory_desired_rotations,
            simulation_rotation_halflife,
            15.0f * dt);
        
        trajectory_desired_velocities_predict(
          trajectory_desired_velocities,
          trajectory_rotations,
          desired_velocity,
          camera_azimuth,
          gamepadstick_left,
          gamepadstick_right,
          desired_strafe,
          simulation_fwrd_speed,
          simulation_side_speed,
          simulation_back_speed,
          15.0f * dt);
        
        trajectory_positions_predict(
            trajectory_positions,
            trajectory_velocities,
            trajectory_accelerations,
            simulation_position,
            simulation_velocity,
            simulation_acceleration,
            trajectory_desired_velocities,
            simulation_velocity_halflife,
            15.0f * dt,
            obstacles_positions,
            obstacles_scales,
            human_positions,
            human_scales,
            terrain_y);
        
        

        // if (chair_close){
        //     synchronization_data_factor = 1.0f;
        // }
        // if(IsKeyDown(KEY_S)){
        //     synchronization_data_factor = 0.75f;
        // }

        // if (!chair_close) {
        //     if (db_num!=1) {
        //         db_num = 1;
        //         frame_index = db.range_starts(0);
        //         // synchronization_enabled = false;
        //         synchronization_data_factor = 0.75f;
        //         db = db_terrain_pushing;
        //         if (force_search_timer <= 0.0f && (
        //             (length(desired_velocity_change_prev) >= desired_velocity_change_threshold && 
        //             length(desired_velocity_change_curr)  < desired_velocity_change_threshold)
        //         ||  (length(desired_rotation_change_prev) >= desired_rotation_change_threshold && 
        //             length(desired_rotation_change_curr)  < desired_rotation_change_threshold)))
        //         {
        //             force_search = true;
        //             force_search_timer = search_time;
        //         }
        //         else if (force_search_timer > 0)
        //         {
        //             force_search_timer -= dt;
        //         }
        //     }       
        // }
        // else { 
        //     if (db_num != 2) {
        //         db_num = 2;
        //         frame_index = db_chair.range_starts(0);
        //         // synchronization_enabled = true;
        //         synchronization_data_factor = 1.0f;
        //         db = db_chair;
        //         if (force_search_timer <= 0.0f){
        //             force_search = true;
        //             force_search_timer = search_time;
        //         }
        //         else if (force_search_timer > 0){
        //             force_search_timer -= dt;
        //         }
        //     }            
        // }
    
           
        // Make query vector for search.
        // In theory this only needs to be done when a search is 
        // actually required however for visualization purposes it
        // can be nice to do it every frame
        array1d<float> query(db.nfeatures());
        slice1d<float> query_features = lmm_enabled ? slice1d<float>(features_curr) : db.features(frame_index);

        int offset = 0;
        query_copy_denormalized_feature(query, offset, 3, query_features, db.features_offset, db.features_scale); // Left Foot Position
        query_copy_denormalized_feature(query, offset, 3, query_features, db.features_offset, db.features_scale); // Right Foot Position
        // query_copy_denormalized_feature(query, offset, 3, query_features, db.features_offset, db.features_scale); // Hips Position
        query_copy_denormalized_feature(query, offset, 3, query_features, db.features_offset, db.features_scale); // Left Foot Velocity
        query_copy_denormalized_feature(query, offset, 3, query_features, db.features_offset, db.features_scale); // Right Foot Velocity
        query_copy_denormalized_feature(query, offset, 3, query_features, db.features_offset, db.features_scale); // Hip Velocity

        // query_copy_denormalized_feature2(query, offset, 18, query_features, db.features_offset, db.features_scale, chair_close, push_animation, terrain_y); 

        query_compute_trajectory_position_feature(query, offset, bone_positions(0), bone_positions(1), bone_rotations(0), trajectory_positions, terrain_y);
        query_compute_trajectory_direction_feature(query, offset, bone_rotations(0), trajectory_rotations);
    
        quat chair_direction = quat_from_angle_axis(chair_rotation_angel(chair_num), vec3(0,1,0));
        vec3 chair_position = chair_positions(chair_num);
        printf("chair_num: %d\n", chair_num);
        query_compute_chair_position_feature(query, offset, bone_positions(0), bone_rotations(0), chair_position, chair_animation);
        query_compute_chair_direction_feature(query, offset, bone_positions(0), bone_rotations(0), chair_direction, chair_animation);
        printf("human_num: %d\n", human_num);
        query_compute_other_position_feature(query, offset, bone_positions(0), bone_rotations(0), human_positions(human_num), push_animation);
        query_compute_other_rotation_feature(query, offset, bone_positions(0), human_positions(human_num), bone_rotations(0), human_rotations(human_num), push_animation);


        assert(offset == db.nfeatures());
        // Check if we reached the end of the current anim
        end_of_anim = database_trajectory_index_clamp(db, frame_index, 1) == frame_index;
        
        array1d<float> query2(db2.nfeatures());

        slice1d<float> query_features2 = lmm_enabled ? slice1d<float>(features_curr2) : db.features(frame_index2);

        int offset2 = 0;
        query_compute_other_position_feature(query2, offset2, human_positions(human_num), human_rotations(human_num), bone_positions(0), push_animation);
        query_compute_other_rotation_feature(query2, offset2, human_positions(human_num), bone_positions(0), human_rotations(human_num), bone_rotations(0), push_animation);
        query_compute_motion_index(query2, offset2, human_close, push_animation, human_num);

        assert(offset2 == db2.nfeatures());

        // Check if we reached the end of the current anim
        // vector<int> stop_frame = {53,  115,  189,  290,  395,  485,  568,  687,  745,  825, 917, 1034, 1236, 1310};
        bool end_of_anim2 = database_trajectory_index_clamp(db2, frame_index2, 1) == frame_index2;

        // end_of_anim2 = database_pushed_index_clamp(frame_index2, 1) == frame_index2;
        // printf("end_of_anim2: %s",end_of_anim2);
        
        // Do we need to search?
        if (force_search || search_timer <= 0.0f || end_of_anim)
        {
            if (lmm_enabled)
            {
                // Project query onto nearest feature vector
                
                float best_cost = FLT_MAX;
                bool transition = false;
                
                projector_evaluate(
                    transition,
                    best_cost,
                    features_proj,
                    latent_proj,
                    projector_evaluation,
                    query,
                    db.features_offset,
                    db.features_scale,
                    features_curr,
                    projector);
                
                // If projection is sufficiently different from current
                if (transition)
                {   
                    // Evaluate pose for projected features
                    decompressor_evaluate(
                        trns_bone_positions,
                        trns_bone_velocities,
                        trns_bone_rotations,
                        trns_bone_angular_velocities,
                        trns_bone_contacts,
                        decompressor_evaluation,
                        features_proj,
                        latent_proj,
                        curr_bone_positions(0),
                        curr_bone_rotations(0),
                        decompressor,
                        dt);
                    
                    // Transition inertializer to this pose
                    inertialize_pose_transition(
                        bone_offset_positions,
                        bone_offset_velocities,
                        bone_offset_rotations,
                        bone_offset_angular_velocities,
                        transition_src_position,
                        transition_src_rotation,
                        transition_dst_position,
                        transition_dst_rotation,
                        bone_positions(0),
                        bone_velocities(0),
                        bone_rotations(0),
                        bone_angular_velocities(0),
                        curr_bone_positions,
                        curr_bone_velocities,
                        curr_bone_rotations,
                        curr_bone_angular_velocities,
                        trns_bone_positions,
                        trns_bone_velocities,
                        trns_bone_rotations,
                        trns_bone_angular_velocities);
                    
                    // Update current features and latents
                    features_curr = features_proj;
                    latent_curr = latent_proj;
                }
            }
            else
            {
                // Search
                // if(synchronization_enabled){
                //     if(IsKeyDown(KEY_S)) {
                //         synchronization_enabled =  false;
                //     }
                // }
                
                int best_index = end_of_anim ? -1 : frame_index;
                float best_cost = FLT_MAX;
                
                database_search(
                    best_index,
                    best_cost,
                    db,
                    query);
                
                // Transition if better frame found
                
                if (best_index != frame_index || push_animation)
                {
                    trns_bone_positions = db.bone_positions(best_index);
                    trns_bone_velocities = db.bone_velocities(best_index);
                    trns_bone_rotations = db.bone_rotations(best_index);
                    trns_bone_angular_velocities = db.bone_angular_velocities(best_index);
                    
                    inertialize_pose_transition(
                        bone_offset_positions,
                        bone_offset_velocities,
                        bone_offset_rotations,
                        bone_offset_angular_velocities,
                        transition_src_position,
                        transition_src_rotation,
                        transition_dst_position,
                        transition_dst_rotation,
                        bone_positions(0),
                        bone_velocities(0),
                        bone_rotations(0),
                        bone_angular_velocities(0),
                        curr_bone_positions,
                        curr_bone_velocities,
                        curr_bone_rotations,
                        curr_bone_angular_velocities,
                        trns_bone_positions,
                        trns_bone_velocities,
                        trns_bone_rotations,
                        trns_bone_angular_velocities);
                    
                    frame_index = best_index;
                }
            }

            // Reset search timer
            search_timer = search_time;
        }
        
        // if (human_close){
        if (force_search_pushed || search_timer_pushed <= 0.0f || end_of_anim2){ 
            if (lmm_enabled)
            {
                // Project query onto nearest feature vector
                
                float best_cost2 = FLT_MAX;
                bool transition2 = false;
                
                projector_evaluate(
                    transition2,
                    best_cost2,
                    features_proj2,
                    latent_proj2,
                    projector_evaluation2,
                    query2,
                    db2.features_offset,
                    db2.features_scale,
                    features_curr2,
                    projector2);
                
                // If projection is sufficiently different from current
                if (transition2)
                {   
                    // Evaluate pose for projected features
                    decompressor_evaluate(
                        trns_bone_positions2,
                        trns_bone_velocities2,
                        trns_bone_rotations2,
                        trns_bone_angular_velocities2,
                        trns_bone_contacts2,
                        decompressor_evaluation2,
                        features_proj2,
                        latent_proj2,
                        curr_bone_positions2(0),
                        curr_bone_rotations2(0),
                        decompressor2,
                        dt);
                    
                    // Transition inertializer to this pose
                    inertialize_pose_transition(
                        bone_offset_positions2,
                        bone_offset_velocities2,
                        bone_offset_rotations2,
                        bone_offset_angular_velocities2,
                        transition_src_position2,
                        transition_src_rotation2,
                        transition_dst_position2,
                        transition_dst_rotation2,
                        bone_positions2(0),
                        bone_velocities2(0),
                        bone_rotations2(0),
                        bone_angular_velocities2(0),
                        curr_bone_positions2,
                        curr_bone_velocities2,
                        curr_bone_rotations2,
                        curr_bone_angular_velocities2,
                        trns_bone_positions2,
                        trns_bone_velocities2,
                        trns_bone_rotations2,
                        trns_bone_angular_velocities2);
                    
                    // Update current features and latents
                    features_curr2 = features_proj2;
                    latent_curr2 = latent_proj2;
                }
            }
            else
            {
                int best_index2 = end_of_anim2 ? -1 : frame_index2;
                float best_cost2 = FLT_MAX;

                printf("database_search (%d) \n", frame_index2);
                
                database_search(
                    best_index2,
                    best_cost2,
                    db2,
                    query2);
                
                // Transition if better frame found
                
                if (best_index2 != frame_index2)
                {
                    trns_bone_positions2 = db2.bone_positions(best_index2);
                    trns_bone_velocities2 = db2.bone_velocities(best_index2);
                    trns_bone_rotations2 = db2.bone_rotations(best_index2);
                    trns_bone_angular_velocities2 = db2.bone_angular_velocities(best_index2);
                    
                    inertialize_pose_transition(
                        bone_offset_positions2,
                        bone_offset_velocities2,
                        bone_offset_rotations2,
                        bone_offset_angular_velocities2,
                        transition_src_position2,
                        transition_src_rotation2,
                        transition_dst_position2,
                        transition_dst_rotation2,
                        bone_positions2(0),
                        bone_velocities2(0),
                        bone_rotations2(0),
                        bone_angular_velocities2(0),
                        curr_bone_positions2,
                        curr_bone_velocities2,
                        curr_bone_rotations2,
                        curr_bone_angular_velocities2,
                        trns_bone_positions2,
                        trns_bone_velocities2,
                        trns_bone_rotations2,
                        trns_bone_angular_velocities2);
                    
                    frame_index2 = best_index2;
                } 
            }
            search_timer_pushed = search_time_pushed;
        }
        
        // Tick down search timer
        search_timer -= dt;
        search_timer_pushed -= dt; 
        // if(end_of_animation2){
        //     search_timer_pushed -= dt; 
        //     end_of_animation2 = false;
        // }

        if (lmm_enabled)
        {
            // Update features and latents
            stepper_evaluate(
                features_curr,
                latent_curr,
                stepper_evaluation,
                stepper,
                dt);
            
            // Decompress next pose
            decompressor_evaluate(
                curr_bone_positions,
                curr_bone_velocities,
                curr_bone_rotations,
                curr_bone_angular_velocities,
                curr_bone_contacts,
                decompressor_evaluation,
                features_curr,
                latent_curr,
                curr_bone_positions(0),
                curr_bone_rotations(0),
                decompressor,
                dt);
            
            // Update features and latents
            stepper_evaluate(
                features_curr2,
                latent_curr2,
                stepper_evaluation2,
                stepper2,
                dt);
            
            // Decompress next pose
            decompressor_evaluate(
                curr_bone_positions2,
                curr_bone_velocities2,
                curr_bone_rotations2,
                curr_bone_angular_velocities2,
                curr_bone_contacts2,
                decompressor_evaluation2,
                features_curr2,
                latent_curr2,
                curr_bone_positions2(0),
                curr_bone_rotations2(0),
                decompressor2,
                dt);

        }
        else
        {              
            // Look-up Next Pose
            curr_bone_positions = db.bone_positions(frame_index);
            curr_bone_velocities = db.bone_velocities(frame_index);
            curr_bone_rotations = db.bone_rotations(frame_index);
            curr_bone_angular_velocities = db.bone_angular_velocities(frame_index);
            curr_bone_contacts = db.contact_states(frame_index);

            // Tick frame
            frame_index++; // Assumes dt is fixed to 60fps


            curr_bone_positions2 = db2.bone_positions(frame_index2);
            curr_bone_velocities2 = db2.bone_velocities(frame_index2);
            curr_bone_rotations2 = db2.bone_rotations(frame_index2);
            curr_bone_angular_velocities2 = db2.bone_angular_velocities(frame_index2);
            curr_bone_contacts2 = db2.contact_states(frame_index2);

            frame_index2++; // Assumes dt is fixed to 60fps
            
        }
        
        // Update inertializer
        
        inertialize_pose_update(
            bone_positions,
            bone_velocities,
            bone_rotations,
            bone_angular_velocities,
            bone_offset_positions,
            bone_offset_velocities,
            bone_offset_rotations,
            bone_offset_angular_velocities,
            curr_bone_positions,
            curr_bone_velocities,
            curr_bone_rotations,
            curr_bone_angular_velocities,
            transition_src_position,
            transition_src_rotation,
            transition_dst_position,
            transition_dst_rotation,
            inertialize_blending_halflife,
            dt);
        

        // if (!human_close){
        //     curr_bone_positions2 = init_bone_positions2;
        //     curr_bone_rotations2 = init_bone_rotations2;
        // }

        inertialize_pose_update2(
            bone_positions2,
            bone_velocities2,
            bone_rotations2,
            bone_angular_velocities2,
            bone_offset_positions2,
            bone_offset_velocities2,
            bone_offset_rotations2,
            bone_offset_angular_velocities2,
            curr_bone_positions2,
            curr_bone_velocities2,
            curr_bone_rotations2,
            curr_bone_angular_velocities2,
            transition_src_position2,
            transition_src_rotation2,
            transition_dst_position2,
            transition_dst_rotation2,
            inertialize_blending_halflife,
            dt,
            terrain_y);

        
        // Update Simulation
        
        vec3 simulation_position_prev = simulation_position;
        
        simulation_positions_update(
            simulation_position, 
            simulation_velocity, 
            simulation_acceleration,
            desired_velocity,
            simulation_velocity_halflife,
            dt,
            obstacles_positions,
            obstacles_scales,
            human_positions,
            human_scales,
            terrain_y);
            
        simulation_rotations_update(
            simulation_rotation, 
            simulation_angular_velocity, 
            desired_rotation,
            simulation_rotation_halflife,
            dt);
        
        // Synchronization 
        
        if (synchronization_enabled)
        {
            vec3 synchronized_position = lerp(
                simulation_position, 
                bone_positions(0),
                synchronization_data_factor);
                
            quat synchronized_rotation = quat_nlerp_shortest(
                simulation_rotation,
                bone_rotations(0), 
                synchronization_data_factor);
          
            // synchronized_position = simulation_terrain(
            //     simulation_position_prev,
            //     synchronized_position,
            //     terrain_y);
            synchronized_position = simulation_collide_obstacles(
                simulation_position_prev,
                synchronized_position,
                obstacles_positions,
                obstacles_scales,
                human_positions,
                human_scales);
            
            simulation_position = synchronized_position;
            simulation_rotation = synchronized_rotation;
            
            inertialize_root_adjust(
                bone_offset_positions(0),
                transition_src_position,
                transition_src_rotation,
                transition_dst_position,
                transition_dst_rotation,
                bone_positions(0),
                bone_rotations(0),
                synchronized_position,
                synchronized_rotation);

            // inertialize_root_adjust(
            //     bone_offset_positions2(0),
            //     transition_src_position2,
            //     transition_src_rotation2,
            //     transition_dst_position2,
            //     transition_dst_rotation2,
            //     bone_positions2(0),
            //     bone_rotations2(0),
            //     bone_positions2(0),
            //     bone_rotations2(0));
        }
        
        // Adjustment 
        
        if (!synchronization_enabled && adjustment_enabled)
        {   
            vec3 adjusted_position = bone_positions(0);
            quat adjusted_rotation = bone_rotations(0);
            
            if (adjustment_by_velocity_enabled)
            {
                adjusted_position = adjust_character_position_by_velocity(
                    bone_positions(0),
                    bone_velocities(0),
                    simulation_position,
                    adjustment_position_max_ratio,
                    adjustment_position_halflife,
                    dt);
                
                adjusted_rotation = adjust_character_rotation_by_velocity(
                    bone_rotations(0),
                    bone_angular_velocities(0),
                    simulation_rotation,
                    adjustment_rotation_max_ratio,
                    adjustment_rotation_halflife,
                    dt);
            }
            else
            {
                adjusted_position = adjust_character_position(
                    bone_positions(0),
                    simulation_position,
                    adjustment_position_halflife,
                    dt);
                
                adjusted_rotation = adjust_character_rotation(
                    bone_rotations(0),
                    simulation_rotation,
                    adjustment_rotation_halflife,
                    dt);
            }
      
            inertialize_root_adjust(
                bone_offset_positions(0),
                transition_src_position,
                transition_src_rotation,
                transition_dst_position,
                transition_dst_rotation,
                bone_positions(0),
                bone_rotations(0),
                adjusted_position,
                adjusted_rotation);
        }
        
        // Clamping
        
        if (!synchronization_enabled && clamping_enabled)
        {
            vec3 adjusted_position = bone_positions(0);
            quat adjusted_rotation = bone_rotations(0);
            
            adjusted_position = clamp_character_position(
                adjusted_position,
                simulation_position,
                clamping_max_distance);
            
            adjusted_rotation = clamp_character_rotation(
                adjusted_rotation,
                simulation_rotation,
                clamping_max_angle);
            
            inertialize_root_adjust(
                bone_offset_positions(0),
                transition_src_position,
                transition_src_rotation,
                transition_dst_position,
                transition_dst_rotation,
                bone_positions(0),
                bone_rotations(0),
                adjusted_position,
                adjusted_rotation);
        }
        
        // Contact fixup with foot locking and IK

        adjusted_bone_positions = bone_positions;
        adjusted_bone_rotations = bone_rotations;

        adjusted_bone_positions2 = bone_positions2;
        adjusted_bone_rotations2 = bone_rotations2;

        if (ik_enabled)
        {
            for (int i = 0; i < contact_bones.size; i++)
            {
                // Find all the relevant bone indices
                int toe_bone = contact_bones(i);
                int heel_bone = db.bone_parents(toe_bone);
                int knee_bone = db.bone_parents(heel_bone);
                int hip_bone = db.bone_parents(knee_bone);
                int root_bone = db.bone_parents(hip_bone);
                int sim_bone = db.bone_parents(root_bone);
                
                // Compute the world space position for the toe
                global_bone_computed.zero();
                
                forward_kinematics_partial(
                    global_bone_positions,
                    global_bone_rotations,
                    global_bone_computed,
                    bone_positions,
                    bone_rotations,
                    db.bone_parents,
                    toe_bone);
                
                global_bone_computed2.zero();
                
                forward_kinematics_partial(
                    global_bone_positions2,
                    global_bone_rotations2,
                    global_bone_computed2,
                    bone_positions2,
                    bone_rotations2,
                    db2.bone_parents,
                    toe_bone);
                
                // Update the contact state
                contact_update(
                    contact_states(i),
                    contact_locks(i),
                    contact_positions(i),  
                    contact_velocities(i),
                    contact_points(i),
                    contact_targets(i),
                    contact_offset_positions(i),
                    contact_offset_velocities(i),
                    global_bone_positions(toe_bone),
                    curr_bone_contacts(i),
                    ik_unlock_radius,
                    ik_foot_height,
                    ik_blending_halflife,
                    dt,
                    terrain_y);
                
                contact_update(
                    contact_states2(i),
                    contact_locks2(i),
                    contact_positions2(i),  
                    contact_velocities2(i),
                    contact_points2(i),
                    contact_targets2(i),
                    contact_offset_positions2(i),
                    contact_offset_velocities2(i),
                    global_bone_positions2(toe_bone),
                    curr_bone_contacts2(i),
                    ik_unlock_radius,
                    ik_foot_height,
                    ik_blending_halflife,
                    dt,
                    terrain_y);
                
                // Ensure contact position never goes through floor
                // vec3 contact_position_clamp = contact_positions(i);
                // contact_position_clamp.y = maxf(contact_position_clamp.y, ik_foot_height);
                vec3 contact_position_clamp = contact_positions(i);
                // ik_foot_height = findClosestY(terrain_y, contact_positions(i).x, contact_positions(i).z);
                contact_position_clamp.y = maxf(contact_position_clamp.y, ik_foot_height);
                contact_position_clamp.y = maxf(contact_position_clamp.y, findClosestY(terrain_y, contact_position_clamp.x, contact_position_clamp.z));
                // contact_position_clamp.y = maxf(contact_position_clamp.y, findClosestY(terrain_y, contact_position_clamp.x, contact_position_clamp.z) + ik_foot_height);
                
                vec3 contact_position_clamp2 = contact_positions2(i);
                contact_position_clamp2.y = maxf(contact_position_clamp2.y, ik_foot_height);
                // contact_position_clamp2.y = maxf(contact_position_clamp2.y, findClosestY(terrain_y, contact_positions2(i).x, contact_positions2(i).z));
                
                // Re-compute toe, heel, knee, hip, and root bone positions
                for (int bone : {heel_bone, knee_bone, hip_bone, root_bone, sim_bone})
                {
                    forward_kinematics_partial(
                        global_bone_positions,
                        global_bone_rotations,
                        global_bone_computed,
                        bone_positions,
                        bone_rotations,
                        db.bone_parents,
                        bone);

                    forward_kinematics_partial(
                        global_bone_positions2,
                        global_bone_rotations2,
                        global_bone_computed2,
                        bone_positions2,
                        bone_rotations2,
                        db2.bone_parents,
                        bone);
                }
                
                // Perform simple two-joint IK to place heel
                ik_two_bone(
                    adjusted_bone_rotations(hip_bone),
                    adjusted_bone_rotations(knee_bone),
                    global_bone_positions(hip_bone),
                    global_bone_positions(knee_bone),
                    global_bone_positions(heel_bone),
                    contact_position_clamp + (global_bone_positions(heel_bone) - global_bone_positions(toe_bone)),
                    quat_mul_vec3(global_bone_rotations(knee_bone), vec3(0.0f, 1.0f, 0.0f)),
                    global_bone_rotations(hip_bone),
                    global_bone_rotations(knee_bone),
                    global_bone_rotations(root_bone),
                    ik_max_length_buffer);
                
                ik_two_bone(
                    adjusted_bone_rotations2(hip_bone),
                    adjusted_bone_rotations2(knee_bone),
                    global_bone_positions2(hip_bone),
                    global_bone_positions2(knee_bone),
                    global_bone_positions2(heel_bone),
                    contact_position_clamp2 + (global_bone_positions2(heel_bone) - global_bone_positions2(toe_bone)),
                    quat_mul_vec3(global_bone_rotations2(knee_bone), vec3(0.0f, 1.0f, 0.0f)),
                    global_bone_rotations2(hip_bone),
                    global_bone_rotations2(knee_bone),
                    global_bone_rotations2(root_bone),
                    ik_max_length_buffer);
                
                // Re-compute toe, heel, and knee positions 
                global_bone_computed.zero();
                global_bone_computed2.zero();

                for (int bone : {toe_bone, heel_bone, knee_bone})
                {
                    forward_kinematics_partial(
                        global_bone_positions,
                        global_bone_rotations,
                        global_bone_computed,
                        adjusted_bone_positions,
                        adjusted_bone_rotations,
                        db.bone_parents,
                        bone);

                    forward_kinematics_partial(
                        global_bone_positions2,
                        global_bone_rotations2,
                        global_bone_computed2,
                        adjusted_bone_positions2,
                        adjusted_bone_rotations2,
                        db2.bone_parents,
                        bone);
                }
                
                // Rotate heel so toe is facing toward contact point
                ik_look_at(
                    adjusted_bone_rotations(heel_bone),
                    global_bone_rotations(knee_bone),
                    global_bone_rotations(heel_bone),
                    global_bone_positions(heel_bone),
                    global_bone_positions(toe_bone),
                    contact_position_clamp);
                    
                ik_look_at(
                    adjusted_bone_rotations2(heel_bone),
                    global_bone_rotations2(knee_bone),
                    global_bone_rotations2(heel_bone),
                    global_bone_positions2(heel_bone),
                    global_bone_positions2(toe_bone),
                    contact_position_clamp2);
                
                // Re-compute toe and heel positions
                global_bone_computed.zero();
                global_bone_computed2.zero();
                
                for (int bone : {toe_bone, heel_bone})
                {
                    forward_kinematics_partial(
                        global_bone_positions,
                        global_bone_rotations,
                        global_bone_computed,
                        adjusted_bone_positions,
                        adjusted_bone_rotations,
                        db.bone_parents,
                        bone);
                    forward_kinematics_partial(
                        global_bone_positions2,
                        global_bone_rotations2,
                        global_bone_computed2,
                        adjusted_bone_positions2,
                        adjusted_bone_rotations2,
                        db2.bone_parents,
                        bone);
                }
                
                // Rotate toe bone so that the end of the toe 
                // does not intersect with the ground
                vec3 toe_end_curr = quat_mul_vec3(
                    global_bone_rotations(toe_bone), vec3(ik_toe_length, 0.0f, 0.0f)) + 
                    global_bone_positions(toe_bone);
                vec3 toe_end_curr2 = quat_mul_vec3(
                    global_bone_rotations2(toe_bone), vec3(ik_toe_length, 0.0f, 0.0f)) + 
                    global_bone_positions2(toe_bone);
                    
                vec3 toe_end_targ = toe_end_curr;
                // ik_foot_height = findClosestY(terrain_y, toe_end_curr.x, toe_end_curr.z);
                // toe_end_targ.y = maxf(toe_end_targ.y, ik_foot_height);

                toe_end_targ.y = maxf(toe_end_targ.y, ik_foot_height);
                toe_end_targ.y = maxf(toe_end_targ.y, findClosestY(terrain_y, toe_end_targ.x, toe_end_targ.z) + ik_foot_height);
                

                vec3 toe_end_targ2 = toe_end_curr2;
                // ik_foot_height = findClosestY(terrain_y, toe_end_curr2.x, toe_end_curr2.z);
                // toe_end_targ2.y = maxf(toe_end_targ2.y, ik_foot_height);
                toe_end_targ2.y = maxf(toe_end_targ2.y, ik_foot_height);
                toe_end_targ2.y = maxf(toe_end_targ2.y, findClosestY(terrain_y, toe_end_targ2.x, toe_end_targ2.z) + ik_foot_height);
                
                ik_look_at(
                    adjusted_bone_rotations(toe_bone),
                    global_bone_rotations(heel_bone),
                    global_bone_rotations(toe_bone),
                    global_bone_positions(toe_bone),
                    toe_end_curr,
                    toe_end_targ);

                ik_look_at(
                    adjusted_bone_rotations2(toe_bone),
                    global_bone_rotations2(heel_bone),
                    global_bone_rotations2(toe_bone),
                    global_bone_positions2(toe_bone),
                    toe_end_curr2,
                    toe_end_targ2);
            }

            
        }
        

        
        


        // Full pass of forward kinematics to compute 
        // all bone positions and rotations in the world
        // space ready for rendering

        // if ((std::chrono::steady_clock::now() - pushing_state_changed_time > std::chrono::milliseconds(pushing_milliseconds)))
        // if(!human_close)
        // {
        //     bone_positions2(0) = human_positions(0);
        //     bone_rotations2(0) = human_rotations(0);
        //     bone_velocities2(0) = human_velocities(0);

        //     for (int j = 1; j < db.bone_parents.size; j++)
        //     {
        //         syn_bone_positions(j) = lerp(
        //             init_bone_positions(j), 
        //             last_bone_positions(j),
        //             0.9f);
                
        //         syn_bone_rotations(j) = quat_nlerp_shortest(
        //             init_bone_rotations(j),
        //             last_bone_rotations(j), 
        //             0.9f);

        //         last_bone_positions(j) = syn_bone_positions(j);
        //         last_bone_rotations(j) = syn_bone_rotations(j);

        //         bone_positions2(0) = last_bone_positions(0);
        //         bone_rotations2(0) = last_bone_rotations(0);
        //         bone_velocities2(0) = vec3();
        //     }
        // }

        // else
        // {
        //     human_positions(0) = bone_positions2(0);
        //     human_rotations(0) = bone_rotations2(0);
        //     human_velocities(0) = bone_velocities2(0);
 
        //     for (int j = 1; j < db.bone_parents.size; j++)
        //     {
        //         syn_bone_positions(j) = lerp(
        //             bone_positions2(j),
        //             last_bone_positions(j),
        //             0.9);
                
        //         syn_bone_rotations(j) = quat_nlerp_shortest(
        //             bone_rotations2(j),
        //             last_bone_rotations(j),
        //             0.9);
 
        //         last_bone_positions(j) = syn_bone_positions(j);
        //         last_bone_rotations(j) = syn_bone_rotations(j);

        //         bone_positions2(j) = last_bone_positions(j);
        //         bone_rotations2(j) = last_bone_rotations(j);
        //         bone_velocities2(j) = vec3();
        //     }

        // } 
        
        forward_kinematics_full(
            global_bone_positions,
            global_bone_rotations,
            adjusted_bone_positions,
            adjusted_bone_rotations,
            db.bone_parents);   
        
        forward_kinematics_full(
            global_bone_positions2,
            global_bone_rotations2,
            bone_positions2,
            bone_rotations2,
            db2.bone_parents);   
        

        // phaseLight += 0.05f;
        // camera_shadow_map.position.x = light_1.position.x = sinf(phaseLight) * 4.0f;
        // Update camera
        
        orbit_camera_update(
            camera, 
            camera_azimuth,
            camera_altitude,
            camera_distance,
            bone_positions(0) + vec3(0, 1, 0),
            // vec3(0, 1, 0),
            gamepadstick_right,
            desired_strafe,
            dt);

        // Render
        // BeginTextureMode(shadow_render_texture);
        // {
        // ClearBackground(RAYWHITE);
        //     BeginMode3D(camera_shadow_map);
        //     {
        //         // Draw ground plane
        //         // DrawModel(model, Vector3Zero(), 1.0f, WHITE);
                
        //         // Draw character model (as shadow)
        //         DrawModel(character_model, to_Vector3(bone_positions(0)), 0.3f, DARKGRAY);
        //         // DrawSphere(to_Vector3(contact_positions(0)), 0.05f,DARKGRAY);
        //         // DrawSphere(to_Vector3(contact_positions(1)), 0.05f, DARKGRAY);
        //     }
        //     EndMode3D();
        // }
        // EndTextureMode();
        // // model.materials[0].shader = shader;
        // character_model.materials[0].shader = shader;

        BeginDrawing();
        ClearBackground(RAYWHITE);
        
        BeginMode3D(camera);
        DrawModel(model, mapPosition, 1.0f, RAYWHITE);
        // Draw Simulation Object
        
        DrawCylinderWires(to_Vector3(simulation_position), 0.6f, 0.6f, 0.001f, 17, ORANGE);
        DrawSphereWires(to_Vector3(simulation_position), 0.05f, 4, 10, ORANGE);
        DrawLine3D(to_Vector3(simulation_position), to_Vector3(
            simulation_position + 0.6f * quat_mul_vec3(simulation_rotation, vec3(0.0f, 0.0f, 1.0f))), ORANGE);
        
        // Draw Clamping Radius/Angles
        
        if (clamping_enabled)
        {
            DrawCylinderWires(
                to_Vector3(simulation_position), 
                clamping_max_distance, 
                clamping_max_distance, 
                0.001f, 17, SKYBLUE);
            
            quat rotation_clamp_0 = quat_mul(quat_from_angle_axis(+clamping_max_angle, vec3(0.0f, 1.0f, 0.0f)), simulation_rotation);
            quat rotation_clamp_1 = quat_mul(quat_from_angle_axis(-clamping_max_angle, vec3(0.0f, 1.0f, 0.0f)), simulation_rotation);
            
            vec3 rotation_clamp_0_dir = simulation_position + 0.6f * quat_mul_vec3(rotation_clamp_0, vec3(0.0f, 0.0f, 1.0f));
            vec3 rotation_clamp_1_dir = simulation_position + 0.6f * quat_mul_vec3(rotation_clamp_1, vec3(0.0f, 0.0f, 1.0f));

            DrawLine3D(to_Vector3(simulation_position), to_Vector3(rotation_clamp_0_dir), SKYBLUE);
            DrawLine3D(to_Vector3(simulation_position), to_Vector3(rotation_clamp_1_dir), SKYBLUE);
        }
        
        // Draw IK foot lock positions
        
        if (ik_enabled)
        {
            for (int i = 0; i <  contact_positions.size; i++)
            {
                if (contact_locks(i))
                {
                    DrawSphereWires(to_Vector3(contact_positions(i)), 0.05f, 4, 10, PINK);
                    DrawSphereWires(to_Vector3(contact_positions2(i)), 0.05f, 4, 10, MAGENTA);
                }
            }
        }
        
        draw_trajectory(
            trajectory_positions,
            trajectory_rotations,
            ORANGE);
        
        draw_obstacles(
            obstacles_positions,
            obstacles_scales);
        
        deform_character_mesh(
            character_mesh, 
            character_data, 
            global_bone_positions, 
            global_bone_rotations,
            db.bone_parents);

        
        
        // printf("bone_positions: %f, %f \n", bone_positions(0).x, bone_positions(0).z);
        
        DrawModel(character_model, (Vector3){0.0f, 0.0f, 0.0f}, 1.0f, RAYWHITE);

        if (push_animation){
            human_positions(human_num) = bone_positions2(0);
            human_rotations(human_num) = bone_rotations2(0);
            human_velocities(human_num) = bone_velocities2(0);
        }
        
        bone_positions2(0) = human_positions(human_num);
        bone_rotations2(0) = human_rotations(human_num);
        bone_velocities2(0) = human_velocities(human_num);
        deform_character_mesh(
            character_mesh,
            character_data, 
            global_bone_positions2,
            global_bone_rotations2,
            db2.bone_parents);
        DrawModel(character_model, (Vector3){0.0f, 0.0f, 0.0f}, 1.0f, BLUE);

        // bone_positions2(0) = human_positions(1);
        // bone_rotations2(0) = human_rotations(1);
        // bone_velocities2(0) = human_velocities(1);
        // deform_character_mesh(
        //     character_mesh,
        //     character_data, 
        //     global_bone_positions2,
        //     global_bone_rotations2,
        //     db2.bone_parents);
        // DrawModel(character_model, (Vector3){0.0f, 0.0f, 0.0f}, 1.0f, BLUE);

        // bone_positions2(0) = human_positions(2);
        // bone_rotations2(0) = human_rotations(2);
        // bone_velocities2(0) = human_velocities(2);
        // deform_character_mesh(
        //     character_mesh,
        //     character_data, 
        //     global_bone_positions2,
        //     global_bone_rotations2,
        //     db2.bone_parents);
        // DrawModel(character_model, (Vector3){0.0f, 0.0f, 0.0f}, 1.0f, BLUE);

        // bone_positions2(0) = human_positions(3);
        // bone_rotations2(0) = human_rotations(3);
        // bone_velocities2(0) = human_velocities(3);
        // deform_character_mesh(
        //     character_mesh,
        //     character_data, 
        //     global_bone_positions2,
        //     global_bone_rotations2,
        //     db2.bone_parents);
        // DrawModel(character_model, (Vector3){0.0f, 0.0f, 0.0f}, 1.0f, BLUE);

        // bone_positions2(0) = human_positions(4);
        // bone_rotations2(0) = human_rotations(4);
        // bone_velocities2(0) = human_velocities(4);
        // deform_character_mesh(
        //     character_mesh,
        //     character_data, 
        //     global_bone_positions2,
        //     global_bone_rotations2,
        //     db2.bone_parents);
        // DrawModel(character_model, (Vector3){0.0f, 0.0f, 0.0f}, 1.0f, BLUE);

        // for (int i = 0; i < human_positions.size; i++){
        //     string bone_pos_str = "bone_positions2_" + std::to_string(i)

        //     bone_positions2(0) = human_positions(i);
        //     bone_rotations2(0) = human_rotations(i);
        //     bone_velocities2(0) = human_velocities(i);

        //     deform_character_mesh(
        //         character_mesh,
        //         character_data, 
        //         global_bone_positions2,
        //         global_bone_rotations2,
        //         db2.bone_parents);

        //     DrawModel(character_model, (Vector3){0.0f, 0.0f, 0.0f}, 1.0f, BLUE);
        // }
        // if(push_animation){

        // }

        // if(!push_animation)
        // {

        // for (int i = 0; i < human_positions.size-1; i++){

        //     bone_positions2(0) = human_positions(i);
        //     bone_rotations2(0) = human_rotations(i);
        //     bone_velocities2(0) = human_velocities(i);

        //     // for (int j = 1; j < db.bone_parents.size; j++)
        //     // {
        //     //     syn_bone_positions(j) = lerp(
        //     //         init_bone_positions(j), 
        //     //         last_bone_positions(j),
        //     //         0.9f);
                
        //     //     syn_bone_rotations(j) = quat_nlerp_shortest(
        //     //         init_bone_rotations(j),
        //     //         last_bone_rotations(j), 
        //     //         0.9f);

        //     //     last_bone_positions(j) = syn_bone_positions(j);
        //     //     last_bone_rotations(j) = syn_bone_rotations(j);

        //     //     bone_positions2(0) = last_bone_positions(0);
        //     //     bone_rotations2(0) = last_bone_rotations(0);
        //     //     bone_velocities2(0) = vec3();
        //     // }
            

        // }
        // }

        // else
        // {
        //     human_positions(human_num) = bone_positions2(0);
        //     human_rotations(human_num) = bone_rotations2(0);
        //     human_velocities(human_num) = bone_velocities2(0);

        //     deform_character_mesh(
        //         character_mesh,
        //         character_data, 
        //         global_bone_positions2,
        //         global_bone_rotations2,
        //         db2.bone_parents);

        //     DrawModel(character_model, (Vector3){0.0f, 0.0f, 0.0f}, 1.0f, BLUE);
 
        //     // for (int j = 1; j < db.bone_parents.size; j++)
        //     // {
        //     //     syn_bone_positions(j) = lerp(
        //     //         bone_positions2(j),
        //     //         last_bone_positions(j),
        //     //         0.9);
                
        //     //     syn_bone_rotations(j) = quat_nlerp_shortest(
        //     //         bone_rotations2(j),
        //     //         last_bone_rotations(j),
        //     //         0.9);
 
        //     //     last_bone_positions(j) = syn_bone_positions(j);
        //     //     last_bone_rotations(j) = syn_bone_rotations(j);

        //     //     bone_positions2(j) = last_bone_positions(j);
        //     //     bone_rotations2(j) = last_bone_rotations(j);
        //     //     bone_velocities2(j) = vec3();
        //     // }

        // } 
        

        // DrawModel(character_model2, (Vector3){-2.0f, findClosestY(terrain_y, -2.0, -5.0), -5.0f}, 1.0f, BLUE);
        // DrawModel(character_model2, (Vector3){-5.0f, findClosestY(terrain_y, -5.0, 1.0), 1.0f}, 1.0f, BLUE);

        for (int i = 0; i < chair_positions.size; i++){
            DrawSphereWires(to_Vector3(chair_positions(i)), 0.05f, 4, 10, ORANGE);
            // DrawLine3D(to_Vector3(chair_positions), to_Vector3(chair_positions + 0.25f * vec3(0,0,1)), ORANGE);
            DrawModelEx(chair, (Vector3){chair_positions(i).x, chair_positions(i).y - 0.01f, chair_positions(i).z}, (Vector3){0.0f, 1.0f, 0.0f}, chair_rotation_angel(i), (Vector3){1.3f, 1.3f, 1.3f}, chair_color(i));
        }
        // DrawSphereWires(to_Vector3(chair_positions(0)), 0.05f, 4, 10, ORANGE);
        //     // DrawLine3D(to_Vector3(chair_positions), to_Vector3(chair_positions + 0.25f * vec3(0,0,1)), ORANGE);
        // DrawModelEx(chair, to_Vector3(chair_positions(0)), (Vector3){0.0f, 1.0f, 0.0f}, chair_rotation_angel(0), (Vector3){1.3f, 1.3f, 1.3f}, ORANGE);
        
        

        // Draw matched features
        
        array1d<float> current_features = lmm_enabled ? slice1d<float>(features_curr) : db.features(frame_index);
        denormalize_features(current_features, db.features_offset, db.features_scale);        
        draw_features(current_features, bone_positions(0), bone_rotations(0), MAROON);
        

        array1d<float> current_features2 = lmm_enabled ? slice1d<float>(features_curr) : db2.features(frame_index2);
        denormalize_features(current_features2, db2.features_offset, db2.features_scale);        
        draw_features_pushed(current_features2, bone_positions2(0), bone_rotations2(0), MAROON);
        
        // Draw Simuation Bone
        
        DrawSphereWires(to_Vector3(bone_positions(0)), 0.05f, 4, 10, MAROON);

        // for (int k = 0; k < bone_positions.size; k++){
        //     DrawSphereWires(to_Vector3(bone_positions(k)), 0.05f, 4, 10, MAROON);
        // }

        DrawLine3D(to_Vector3(bone_positions(0)), to_Vector3(
            bone_positions(0) + 0.6f * quat_mul_vec3(bone_rotations(0), vec3(0.0f, 0.0f, 1.0f))), MAROON);
        
        // Draw Ground Plane
        
        // DrawModel(ground_plane_model, (Vector3){0.0f, -0.01f, 0.0f}, 1.0f, WHITE);
        DrawGrid(20, 1.0f);
        // draw_axis(vec3(), quat());
        
        EndMode3D();

        // DrawTexturePro(shadow_render_texture.texture, Rectangle{ 0, 0, (float)shadow_render_texture.texture.width, (float)-shadow_render_texture.texture.height }, Rectangle{ 0, 0, (float)screen_width, (float)screen_height }, Vector2{ 0, 0 }, 0.0f, WHITE);

        // UI
        
        //---------
        
        float ui_sim_hei = 20;
        
        GuiGroupBox((Rectangle){ 970+ screen_width - 1280, ui_sim_hei, 290, 250 }, "simulation object");

        GuiSliderBar(
            (Rectangle){ 1100+ screen_width - 1280, ui_sim_hei + 10, 120, 20 }, 
            "velocity halflife", 
            TextFormat("%5.3f", simulation_velocity_halflife), 
            &simulation_velocity_halflife, 0.0f, 0.5f);
            
        GuiSliderBar(
            (Rectangle){ 1100+ screen_width - 1280, ui_sim_hei + 40, 120, 20 }, 
            "rotation halflife", 
            TextFormat("%5.3f", simulation_rotation_halflife), 
            &simulation_rotation_halflife, 0.0f, 0.5f);
            
        GuiSliderBar(
            (Rectangle){ 1100+ screen_width - 1280, ui_sim_hei + 70, 120, 20 }, 
            "run forward speed", 
            TextFormat("%5.3f", simulation_run_fwrd_speed), 
            &simulation_run_fwrd_speed, 0.0f, 10.0f);
        
        GuiSliderBar(
            (Rectangle){ 1100+ screen_width - 1280, ui_sim_hei + 100, 120, 20 }, 
            "run sideways speed", 
            TextFormat("%5.3f", simulation_run_side_speed), 
            &simulation_run_side_speed, 0.0f, 10.0f);
        
        GuiSliderBar(
            (Rectangle){ 1100+ screen_width - 1280, ui_sim_hei + 130, 120, 20 }, 
            "run backwards speed", 
            TextFormat("%5.3f", simulation_run_back_speed), 
            &simulation_run_back_speed, 0.0f, 10.0f);
        
        GuiSliderBar(
            (Rectangle){ 1100+ screen_width - 1280, ui_sim_hei + 160, 120, 20 }, 
            "walk forward speed", 
            TextFormat("%5.3f", simulation_walk_fwrd_speed), 
            &simulation_walk_fwrd_speed, 0.0f, 5.0f);
        
        GuiSliderBar(
            (Rectangle){ 1100+ screen_width - 1280, ui_sim_hei + 190, 120, 20 }, 
            "walk sideways speed", 
            TextFormat("%5.3f", simulation_walk_side_speed), 
            &simulation_walk_side_speed, 0.0f, 5.0f);
        
        GuiSliderBar(
            (Rectangle){ 1100+ screen_width - 1280, ui_sim_hei + 220, 120, 20 }, 
            "walk backwards speed", 
            TextFormat("%5.3f", simulation_walk_back_speed), 
            &simulation_walk_back_speed, 0.0f, 5.0f);
        
        //---------
        
        float ui_inert_hei = 280;
        
        GuiGroupBox((Rectangle){ 970+ screen_width - 1280, ui_inert_hei, 290, 40 }, "inertiaization blending");
        
        GuiSliderBar(
            (Rectangle){ 1100+ screen_width - 1280, ui_inert_hei + 10, 120, 20 }, 
            "halflife", 
            TextFormat("%5.3f", inertialize_blending_halflife), 
            &inertialize_blending_halflife, 0.0f, 0.3f);
        
        //---------
        
        float ui_lmm_hei = 330;
        
        GuiGroupBox((Rectangle){ 970+ screen_width - 1280, ui_lmm_hei, 290, 40 }, "learned motion matching");
        
        GuiCheckBox(
            (Rectangle){ 1000+ screen_width - 1280, ui_lmm_hei + 10, 20, 20 }, 
            "enabled",
            &lmm_enabled);
        
        //---------
        
        float ui_ctrl_hei = 380;
        
        GuiGroupBox((Rectangle){ 1010+ screen_width - 1280, ui_ctrl_hei, 250, 140 }, "controls");
        
        GuiLabel((Rectangle){ 1030+ screen_width - 1280, ui_ctrl_hei +  10, 200, 20 }, "Left Trigger - Strafe");
        GuiLabel((Rectangle){ 1030+ screen_width - 1280, ui_ctrl_hei +  30, 200, 20 }, "Left Stick - Move");
        GuiLabel((Rectangle){ 1030+ screen_width - 1280, ui_ctrl_hei +  50, 200, 20 }, "Right Stick - Camera / Facing (Stafe)");
        GuiLabel((Rectangle){ 1030+ screen_width - 1280, ui_ctrl_hei +  70, 200, 20 }, "Left Shoulder - Zoom In");
        GuiLabel((Rectangle){ 1030+ screen_width - 1280, ui_ctrl_hei +  90, 200, 20 }, "Right Shoulder - Zoom Out");
        GuiLabel((Rectangle){ 1030+ screen_width - 1280, ui_ctrl_hei + 110, 200, 20 }, "A Button - Walk");


        Color transparentblack = { 0, 0, 0, 128 };
        Rectangle box = { screen_width / 2 - 140, 60, 350, 35 };
        Rectangle box2 = { screen_width / 2 - 140, 60 + 35, 350, 35 };
        DrawRectangleRec(box, transparentblack);

        int fontSize = 20;

        if(human_num2 == 5 && chair_num2 == 3){
            const char *text = "!! CLEAR !!";
            Vector2 text3Size = MeasureTextEx(GetFontDefault(), text, fontSize, 1);

            float text3X = box.x + (box.width - text3Size.x) / 2;
            float text3Y = box.y + (box.height - text3Size.y) / 2;  
            DrawRectangleLinesEx(box, 1.5, ORANGE);          
            DrawText(text, text3X, text3Y, fontSize, ORANGE);
        }
        else{
            DrawRectangleRec(box2, transparentblack);

            char blue_character[20];
            sprintf(blue_character, "BLUE CHARACTER: %d / 5", human_num2);
            Vector2 textSize = MeasureTextEx(GetFontDefault(), blue_character, fontSize, 1);

            float textX = box.x + (box.width - textSize.x) / 2;
            float textY = box.y + (box.height - textSize.y) / 2;            
            DrawRectangleLinesEx(box, 1.5, ORANGE);
            DrawText(blue_character, textX, textY, fontSize, WHITE);


            char chair[20];
            sprintf(chair, "CHAIR: %d / 3", chair_num2);
            Vector2 text2Size = MeasureTextEx(GetFontDefault(), chair, fontSize, 1);

            float text2X = box2.x + (box2.width - text2Size.x) / 2;
            float text2Y = box2.y + (box2.height - text2Size.y) / 2;   
            DrawRectangleLinesEx(box2, 1.5, ORANGE);         
            DrawText(chair, text2X, text2Y, fontSize, WHITE);
        }
        


        

        
        //---------
        
        GuiGroupBox((Rectangle){ 20, 20, 290, 190 }, "feature weights");
        
        GuiSliderBar(
            (Rectangle){ 150, 30, 120, 20 }, 
            "foot position", 
            TextFormat("%5.3f", feature_weight_foot_position), 
            &feature_weight_foot_position, 0.001f, 3.0f);
            
        GuiSliderBar(
            (Rectangle){ 150, 60, 120, 20 }, 
            "foot velocity", 
            TextFormat("%5.3f", feature_weight_foot_velocity), 
            &feature_weight_foot_velocity, 0.001f, 3.0f);
        
        GuiSliderBar(
            (Rectangle){ 150, 90, 120, 20 }, 
            "hip velocity", 
            TextFormat("%5.3f", feature_weight_hip_velocity), 
            &feature_weight_hip_velocity, 0.001f, 3.0f);
        
        GuiSliderBar(
            (Rectangle){ 150, 120, 120, 20 }, 
            "trajectory positions", 
            TextFormat("%5.3f", feature_weight_trajectory_positions), 
            &feature_weight_trajectory_positions, 0.001f, 3.0f);
        
        GuiSliderBar(
            (Rectangle){ 150, 150, 120, 20 }, 
            "trajectory directions", 
            TextFormat("%5.3f", feature_weight_trajectory_directions), 
            &feature_weight_trajectory_directions, 0.001f, 3.0f);
            
        // if (GuiButton((Rectangle){ 150, 180, 120, 20 }, "rebuild database"))
        // {
        //     database_build_matching_features(
        //         db,
        //         db2,
        //         feature_weight_foot_position,
        //         feature_weight_foot_velocity,
        //         feature_weight_hip_velocity,
        //         feature_weight_trajectory_positions,
        //         feature_weight_trajectory_directions,
        //         feature_weight_other);

        //     database_blue_build_matching_features(
        //         db2,
        //         db,
        //         feature_weight_foot_position,
        //         feature_weight_foot_velocity,
        //         feature_weight_hip_velocity,
        //         feature_weight_trajectory_positions,
        //         feature_weight_trajectory_directions,
        //         feature_weight_other);
        // }
        
        //---------
        
        float ui_sync_hei = 220;
        
        GuiGroupBox((Rectangle){ 20, ui_sync_hei, 290, 70 }, "synchronization");

        GuiCheckBox(
            (Rectangle){ 50, ui_sync_hei + 10, 20, 20 }, 
            "enabled",
            &synchronization_enabled);

        GuiSliderBar(
            (Rectangle){ 150, ui_sync_hei + 40, 120, 20 }, 
            "data-driven amount", 
            TextFormat("%5.3f", synchronization_data_factor), 
            &synchronization_data_factor, 0.0f, 1.0f);

        //---------
        
        float ui_adj_hei = 300;
        
        GuiGroupBox((Rectangle){ 20, ui_adj_hei, 290, 130 }, "adjustment");
        
        GuiCheckBox(
            (Rectangle){ 50, ui_adj_hei + 10, 20, 20 }, 
            "enabled",
            &adjustment_enabled);    
        
        GuiCheckBox(
            (Rectangle){ 50, ui_adj_hei + 40, 20, 20 }, 
            "clamp to max velocity",
            &adjustment_by_velocity_enabled);    
        
        GuiSliderBar(
            (Rectangle){ 150, ui_adj_hei + 70, 120, 20 }, 
            "position halflife", 
            TextFormat("%5.3f", adjustment_position_halflife), 
            &adjustment_position_halflife, 0.0f, 0.5f);
        
        GuiSliderBar(
            (Rectangle){ 150, ui_adj_hei + 100, 120, 20 }, 
            "rotation halflife", 
            TextFormat("%5.3f", adjustment_rotation_halflife), 
            &adjustment_rotation_halflife, 0.0f, 0.5f);
        
        //---------
        
        float ui_clamp_hei = 440;
        
        GuiGroupBox((Rectangle){ 20, ui_clamp_hei, 290, 100 }, "clamping");
        
        GuiCheckBox(
            (Rectangle){ 50, ui_clamp_hei + 10, 20, 20 }, 
            "enabled",
            &clamping_enabled);      
        
        GuiSliderBar(
            (Rectangle){ 150, ui_clamp_hei + 40, 120, 20 }, 
            "distance", 
            TextFormat("%5.3f", clamping_max_distance), 
            &clamping_max_distance, 0.0f, 0.5f);
        
        GuiSliderBar(
            (Rectangle){ 150, ui_clamp_hei + 70, 120, 20 }, 
            "angle", 
            TextFormat("%5.3f", clamping_max_angle), 
            &clamping_max_angle, 0.0f, PIf);
        
        //---------
        
        float ui_ik_hei = 550;
        
        GuiGroupBox((Rectangle){ 20, ui_ik_hei, 290, 100 }, "inverse kinematics");
        
        bool ik_enabled_prev = ik_enabled;
        
        GuiCheckBox(
            (Rectangle){ 50, ui_ik_hei + 10, 20, 20 }, 
            "enabled",
            &ik_enabled);      
        
        // Foot locking needs resetting when IK is toggled
        if (ik_enabled && !ik_enabled_prev)
        {
            for (int i = 0; i < contact_bones.size; i++)
            {
                vec3 bone_position;
                vec3 bone_velocity;
                quat bone_rotation;
                vec3 bone_angular_velocity;
                
                forward_kinematics_velocity(
                    bone_position,
                    bone_velocity,
                    bone_rotation,
                    bone_angular_velocity,
                    bone_positions,
                    bone_velocities,
                    bone_rotations,
                    bone_angular_velocities,
                    db.bone_parents,
                    contact_bones(i));
                
                contact_reset(
                    contact_states(i),
                    contact_locks(i),
                    contact_positions(i),  
                    contact_velocities(i),
                    contact_points(i),
                    contact_targets(i),
                    contact_offset_positions(i),
                    contact_offset_velocities(i),
                    bone_position,
                    bone_velocity,
                    false);

                vec3 bone_position2;
                vec3 bone_velocity2;
                quat bone_rotation2;
                vec3 bone_angular_velocity2;
                
                forward_kinematics_velocity(
                    bone_position2,
                    bone_velocity2,
                    bone_rotation2,
                    bone_angular_velocity2,
                    bone_positions2,
                    bone_velocities2,
                    bone_rotations2,
                    bone_angular_velocities2,
                    db2.bone_parents,
                    contact_bones(i));
                
                contact_reset(
                    contact_states2(i),
                    contact_locks2(i),
                    contact_positions2(i),  
                    contact_velocities2(i),
                    contact_points2(i),
                    contact_targets2(i),
                    contact_offset_positions2(i),
                    contact_offset_velocities2(i),
                    bone_position2,
                    bone_velocity2,
                    false);
            }
        }
        
        GuiSliderBar(
            (Rectangle){ 150, ui_ik_hei + 40, 120, 20 }, 
            "blending halflife", 
            TextFormat("%5.3f", ik_blending_halflife), 
            &ik_blending_halflife, 0.0f, 1.0f);
        
        GuiSliderBar(
            (Rectangle){ 150, ui_ik_hei + 70, 120, 20 }, 
            "unlock radius", 
            TextFormat("%5.3f", ik_unlock_radius), 
            &ik_unlock_radius, 0.0f, 0.5f);
        
        //---------

        EndDrawing();

    };

#if defined(PLATFORM_WEB)
    std::function<void()> u{update_func};
    emscripten_set_main_loop_arg(update_callback, &u, 0, 1);
#else
    while (!WindowShouldClose())
    {
        update_func();
    }
#endif

    // Unload stuff and finish
    UnloadModel(character_model);
    // UnloadModel(ground_plane_model);
    UnloadShader(character_shader);
    UnloadShader(ground_plane_shader);
    UnloadTexture(texture);
    // UnloadRenderTexture(shadow_render_texture);
    UnloadModel(model);
    UnloadShader(shader);
    CloseWindow();

    return 0;
}