#ifndef ROBOT_ROLE_H
#define ROBOT_ROLE_H

/**
 * @enum ERobotRole
 * @brief Defines the roles a robot plays in the flux network.
 *
 * NORMAL robots relay flux as intermediate chains; SOURCE robots inject food and
 * pressure at the food location; TARGET robots act as the sink that consumes the
 * flow. The role also drives the LED color coding used for visual debugging.
 */
enum class ERobotRole {
    NORMAL,
    SOURCE,
    TARGET
};

#endif // ROBOT_ROLE_H
