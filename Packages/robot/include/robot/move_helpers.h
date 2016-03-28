#ifndef MOVE_HELPERS_H
#define MOVE_HELPERS_H

enum TurnType {
    LEFT, NONE, RIGHT
};

struct Range {
    int low_lim_;
    int high_lim_;
};

struct MoveSpecs {
    /**
     * @brief Minimum proximity distance that the robot can have from the wall it
     * is following of the walls in front
     */
    double high_security_distance_;

    /**
     * @brief Minimum proximity distance that the robot can have from the wall on
     * the opposite side of the wall it is following
     */
    double low_security_distance_;

    /**
     * @brief Maximum distance the robot can be away from the anchor wall
     */
    double wall_follow_distance_;

    /**
     * @brief Linear velocity for a single robot movement
     */
    double linear_velocity_;

    /**
     * @brief Angular velocity for a single robot movement
     */
    double angular_velocity_;

    /**
     * @brief Specifies the type of turn the robot should make when it is far
     * from the wall
     */
    TurnType turn_type_;

    /**
     * @brief Range of LRF values to the right of the robot
     */
    Range right_range_;

    /**
     * @brief Range of LRF values to the left of the robot
     */
    Range left_range_;

    /**
     * @brief Range of 90 degrees in front of the robot
     */
    Range center_range_;
};

struct MoveStatus {
    /**
     * @brief Can the robot continue walking in a straight line or not
     */
    bool can_continue_;

    /**
     * @brief Is the robot close enough to the anchor wall
     */
    bool is_close_to_wall_;

    /**
     * @brief Is the robot anchored to a wall or not
     */
    bool is_following_wall_;

    /**
     * @brief Is the robot in circle hit mode
     */
    bool circle_hit_mode_;

    /**
     * @brief Is the robot positioned appropriately to go straight to the goal
     */
    bool hit_goal_;
};

#endif
