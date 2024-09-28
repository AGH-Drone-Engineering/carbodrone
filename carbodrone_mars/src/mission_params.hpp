#pragma once

/**
 * *** THE MISSION NODE CAN EXECUTE ONLY ONE MISSION DURING ITS LIFETIME ***
 *
 * You need to restart the mission node to start a new mission.
 */

/**
 * Terminology:
 * - "Field" = white banner where balls are placed
 * - "Barrel" = ball drop zone
 * - "Landing pad" = location where the drone starts and ends the mission
 * - "Mission start" = this event is triggered by arming the drone
 *
 * Mission plan:
 *
 * 1. Launch all software components
 * 2. Wait for the system to stabilize
 * 3. Arm the drone
 * 4. DO NOT TOUCH THE CONTROLS (see: emergency procedures); touching the controls may break the mission logic
 * 5. The drone will automatically take off
 * 6. The drone will fly to the first field and scan it
 * 7. If it detects the correct ball, it will land, disarm, and grab it
 * 8. The drone will arm, take off, and fly to the barrel and drop off the ball
 * 9. The drone will continue scanning other fields
 * 10. After all balls are dropped off, it will fly to the landing pad and land
 *
 * Emergency procedures
 *
 * The system is capable of automatic arming and mode switching.
 * *** IT CAN TAKE CONTROL AWAY FROM THE OPERATOR ***
 * The drone will FIGHT the operator.
 * It is IMPOSSIBLE to fully take over control from the drone without shutting down the mission computer.
 *
 * In case of danger DISARM THE DRONE.
 * It will not arm after a crash.
 *
 * In case the drone deviates from the planned mission, you can attempt to emergency land:
 * 1. Switch to stabilized mode
 * 2. CONSTANTLY move the joystick to make the drone listen to you.
 * 3. Try to land at the current position (DO NOT FLY THE DRONE AROUND IN MISSION MODE).
 */

/**
 * The position of the landing pad (and the altitude from which to initiate the landing).
 * The drone will land here after the mission.
 * The lat/lon will be ignored if INITIALIZE_LANDING_PAD_AFTER_MISSION_START is true.
 */
static double LANDING_PAD_WAYPOINT[3] = {0, 0, 6.0};

/**
 * If true, the landing pad location will be set based on the GPS position of the drone after starting the mission.
 */
static constexpr bool INITIALIZE_LANDING_PAD_AFTER_MISSION_START = true;

/**
 * Barrel lat, lon, approach altitude.
 * The drone will fly to this point when dropping off the balls.
 */
static const double BARREL_WAYPOINT[3] = {53.0096402, 20.929278399999998, 2.0};

/**
 * White banner (called a "field") waypoints.
 *
 * The order does not matter.
 */
static const double FIELD_WAYPOINTS[9][2] = {
    {53.0094351,
    20.929271399999998},

    {53.009442299999996,
    20.9293384},

    {53.0094499,
    20.929392999999997},


    {53.0094664,
    20.9292745},

    {53.009454999999996,
    20.929336},

    {53.009467,
    20.929390299999998},


    {53.009507799999994,
    20.9292674},

    {53.009507,
    20.9293154},

    {53.0095113,
    20.9293718}
};

/**
 * Number of white banners (called "fields").
 */
static constexpr int NUM_FIELDS = 9;

/**
 * Delay between firebase map updates.
 */
static constexpr int MAP_UPLOADER_DELAY_MS = 1000;

/**
 * Focal length of the camera in pixels.
 * This is used to calculate the distance to the target.
 */
static constexpr double CAMERA_FOCAL_LENGTH_PX = 1397.2235870361328;

/**
 * The drone will take off to this altitude.
 */
static constexpr double MISSION_START_ALT = 6.0;

/**
 * The drone will fly to white banners at this altitude.
 */
static constexpr double FIELD_REPOSITION_ALT = 4.0;

/**
 * The drone will descend to this altitude when scanning a field.
 * It will use the camera to stay above the white banner.
 */
static constexpr double FIELD_SCAN_ALT = 2.0;

/**
 * The drone will wait for this amount of time to stabilize after beginning the field descent.
 * 1 unit is 100 ms.
 */
static constexpr int FIELD_DESCENT_DELAY = 100;

/**
 * Time needed for the grabber to open and close.
 * 1 unit is 100 ms.
 */
static constexpr int GRABBER_DELAY = 20;

static constexpr double GLOBAL_LAT_ACCEPTANCE = 0.00001;
static constexpr double GLOBAL_LON_ACCEPTANCE = 0.000016;
static constexpr float GLOBAL_ALT_ACCEPTANCE = 1.0;

static constexpr double LOCAL_XY_ACCEPTANCE = 0.5;
static constexpr double LOCAL_Z_ACCEPTANCE = 0.5;
static constexpr double VEL_ACCEPANCE = 0.2;

static constexpr int REPOSITION_DELAY = 40;
static constexpr int GOTO_DELAY = 40;

/**
 * Order in which the balls should be picked up.
 *
 * * 0 = blue
 * * 1 = green
 * * 2 = purple
 * * 3 = red
 */
static const int BALL_COLOR_PICKUP_ORDER[3] = { 0, 2, 3 };

/**
 * Override the color of the balls on specific fields.
 *
 * * 0 = blue
 * * 1 = green
 * * 2 = purple
 * * 3 = red
 */
static const int BALL_COLOR_OVERRIDE[9] = {
    1,
    1,
    0,

    3,
    1,
    1,

    1,
    2,
    1,
};

/**
 * If true, the ball color override will be used.
 */
static constexpr bool USE_BALL_COLOR_OVERRIDE = false;
