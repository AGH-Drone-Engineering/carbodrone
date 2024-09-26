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
static const double LANDING_PAD_WAYPOINT[3] = {0, 0, 10.0};

/**
 * If true, the landing pad location will be set based on the GPS position of the drone after starting the mission.
 */
static constexpr bool INITIALIZE_LANDING_PAD_AFTER_MISSION_START = true;

/**
 * Barrel lat, lon, approach altitude.
 * The drone will fly to this point when dropping off the balls.
 */
static const double BARREL_WAYPOINT[3] = {47.398132805733496, 8.54616487844852, 10.0};

/**
 * White banner (called a "field") waypoints.
 *
 * The order does not matter.
 */
static const double FIELD_WAYPOINTS[9][2] = {
    {47.39794428391416,
    8.546124508703029},

    {47.39794345305943,
    8.546163475080547},

    {47.397944639996325,
    8.546205423551793},


    {47.39797073863487,
    8.546205348996033},

    {47.39796990342122,
    8.546164727922147},

    {47.39796978094731,
    8.546123857278653},


    {47.39799703247861,
    8.546121949343245},

    {47.397997851411425,
    8.546163149292989},

    {47.39799618218166,
    8.546203601239528}
};

/**
 * Number of white banners (called "fields").
 */
static constexpr int NUM_FIELDS = 9;

/**
 * The drone will take off to this altitude.
 */
static constexpr double MISSION_START_ALT = 10.0;

/**
 * The drone will descend to this altitude when scanning a field.
 * It will use the camera to stay above the white banner.
 */
static constexpr double FIELD_SCAN_ALT = 3.0;

/**
 * Time needed for the grabber to open and close.
 * 1 unit is 100 ms.
 */
static constexpr int GRABBER_DELAY = 20;

static constexpr double GLOBAL_LAT_ACCEPTANCE = 0.001;
static constexpr double GLOBAL_LON_ACCEPTANCE = 0.001;
static constexpr float GLOBAL_ALT_ACCEPTANCE = 0.5;

static constexpr double LOCAL_XY_ACCEPTANCE = 0.1;
static constexpr double LOCAL_Z_ACCEPTANCE = 0.1;
static constexpr double VEL_ACCEPANCE = 0.5;
