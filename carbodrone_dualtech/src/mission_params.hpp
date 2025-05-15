#pragma once

/**
 * *** THE MISSION NODE CAN EXECUTE ONLY ONE MISSION DURING ITS LIFETIME ***
 *
 * You need to restart the mission node to start a new mission.
 */

/**
 * Terminology:
 *
 * - "Scan Area" = rectangular area that will be scanned
 * - "Landing Pad" = location where the drone starts and ends the mission
 * - "Mission Start" = this event is triggered by arming the drone
 *
 * Mission plan:
 *
 * 1. Launch all software components
 * 2. Wait for the system to stabilize
 * 3. Arm the drone
 * 4. DO NOT TOUCH THE CONTROLS (see: emergency procedures); touching the controls may break the mission logic
 * 5. The drone will automatically take off
 * 6. The drone will fly to the nearest corner of the Scan Area
 * 7. The drone will scan the Scan Area in a zigzag pattern
 * 8. The drone will return to the landing pad and land
 *
 * Emergency procedures:
 *
 * The system is capable of mode switching.
 * *** IT CAN TAKE CONTROL AWAY FROM THE OPERATOR ***
 *
 * In case of danger DISARM THE DRONE.
 * It will not arm after a crash.
 *
 * In case the drone deviates from the planned mission, you can attempt to emergency land:
 * 1. Switch to stabilized mode
 * 2. Try to land at the current position (DO NOT FLY THE DRONE AROUND IN MISSION MODE).
 */

/**
 * The altitude at which the drone will hover above the landing pad after and before landing.
 */
static constexpr double LANDING_PAD_HOVER_ALTITUDE = 5;

/**
 * The altitude at which the drone fly above the Scan Area.
 */
static constexpr double SCAN_ALTITUDE = 5;

static constexpr double MISSION_GOTO_SPEED_MS = 5;

static constexpr double MISSION_SCAN_SPEED_MS = 1;

/**
 * The GPS coordinates of the Scan Area corners.
 */
static const double SCAN_AREA_CORNERS[4][2] = {
    {53.0094351,
    20.929271399999998},

    {53.009442299999996,
    20.9293384},

    {53.0094499,
    20.929392999999997},

    {53.0094664,
    20.9292745},
};

static constexpr double CAMERA_FOCAL_LENGTH_PX = 1280.0;

// --- Internal parameters ---

static constexpr double GLOBAL_LAT_ACCEPTANCE = 0.00001;
static constexpr double GLOBAL_LON_ACCEPTANCE = 0.000016;
static constexpr float GLOBAL_ALT_ACCEPTANCE = 1.0;

static constexpr double LOCAL_XY_ACCEPTANCE = 0.5;
static constexpr double LOCAL_Z_ACCEPTANCE = 0.5;
static constexpr double VEL_ACCEPANCE = 0.2;
