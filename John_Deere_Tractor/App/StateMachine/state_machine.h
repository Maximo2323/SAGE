/* ============================================================================
 * ROBOT STATE MACHINE - PROVISIONAL DESIGN
 *
 * THIS FILE CONTAINS COMMENTS ONLY. No logic is active. No file includes it.
 * Design can change after tests. Each name that ends in _TBD is unknown.
 * Remove the comment marks of one state at a time, after a test of its sensors.
 *
 * Principle
 *   Distance and heading bring the car near each landmark. The physical
 *   feature of the landmark then sets the final position.
 *   A checkpoint is a position that the team defines. The goal is to stop at
 *   the same position in each run, with a difference of 1 cm or less.
 *   This design does not prove that goal.
 *
 * Sensors (provisional: positions and types can change after tests)
 *   Three VL53L1X sensors, read through App/Sensors/distance.h:
 *     DIST_FRONT      : front of the car, points forward (obstacles)
 *     DIST_RIGHT_LOW  : right side, near the ground (low landmarks: sidewalk edge)
 *     DIST_RIGHT_HIGH : right side, points up (landmarks above the low sensor)
 *   Each reading has a status:
 *     DIST_OK        = a target at the given distance
 *     DIST_NO_TARGET = the sensor operates correctly and sees nothing in range
 *     DIST_NOT_VALID = fault, no sensor, or data older than DIST_STALE_MS
 *   "NOT_VALID" is never "NO_TARGET".
 *
 *     status = distance_read(DIST_RIGHT_LOW, &mm);
 *
 *   "Side sensor" below = the right sensor that the route table gives for the
 *   landmark of the segment (low or high).
 *   Because the sensors are on the right, each landmark must be on the right
 *   side of the car in the direction of travel.
 *
 *   Angle to the landmark: the two right sensors are at different heights, not
 *   at different positions along the car. Thus their difference does not give
 *   the angle. Use the CHANGE of the side distance over the travelled distance:
 *     angle = atan(change of side distance / distance travelled)
 *     no change = the car is parallel to the landmark.
 *   This method needs movement. It gives no angle when the car is stopped.
 *   If DIST_RIGHT_HIGH points up at an angle, its reading is not the horizontal
 *   distance. Multiply by cos(HIGH_SENSOR_TILT_DEG_TBD).
 *
 * Route table (one row for each segment; the logic is written one time)
 *   typedef struct {
 *     float heading_deg;        // segment heading, relative to the start
 *     float dist_min_mm;        // window in which the landmark is accepted
 *     float dist_max_mm;
 *     int   landmark;           // LANDMARK_NONE, or a type that the tests define
 *     int   side_sensor;        // DIST_RIGHT_LOW or DIST_RIGHT_HIGH
 *     float clearance_mm;       // lateral distance to hold
 *     float final_offset_mm;    // distance from the reference edge to the checkpoint
 *   } Segment;
 *   static const Segment route[] = { ... ROUTE_TBD ... };
 *   Landmark types, order and values: TBD after the landmark tests.
 *
 * ---------------------------------------------------------------------------
 * States
 *
 *   typedef enum {
 *     ST_START, ST_MANUAL,
 *     ST_APPROACH_SEARCH, ST_CONFIRM_LANDMARK, ST_FOLLOW_AND_ALIGN,
 *     ST_FIND_REFERENCE, ST_FINAL_APPROACH, ST_REALIGN,
 *     ST_WAIT_AT_CHECKPOINT,
 *     ST_OBSTACLE_WAIT, ST_AVOID_REVERSE, ST_AVOID_OUT, ST_AVOID_PASS, ST_AVOID_RETURN,
 *     ST_STOP
 *   } State;
 *
 * ---------------------------------------------------------------------------
 * ST_START
 *   Purpose : wait for a command. Motors off.
 *   Needs   : fresh heading, encoder counts, valid distance readings.
 *   Advance : manual command                         -> ST_MANUAL
 *             automatic start AND all readings valid -> ST_APPROACH_SEARCH
 *   Failure : a reading is not valid -> stay here, report which one.
 *
 * ST_MANUAL
 *   Purpose : keyboard control. This is the firmware that operates today.
 *   Advance : operator command -> ST_START
 *
 * ---------------------------------------------------------------------------
 * DRIVE: the five states below run for the current segment.
 *
 * ST_APPROACH_SEARCH
 *   Purpose : go toward the expected landmark with heading hold.
 *             Decrease the speed SLOW_BEFORE_MM_TBD before the window.
 *   Needs   : distance along the segment, heading, the side sensor.
 *   Advance : side readings in the band AND distance inside the window
 *             -> ST_CONFIRM_LANDMARK
 *             landmark is LANDMARK_NONE AND distance reached -> ST_FINAL_APPROACH
 *   Failure : readings outside the window are ignored (wrong object).
 *             Window end with no landmark -> ST_STOP (LANDMARK_NOT_FOUND)
 *
 *   // if (seg->landmark == LANDMARK_NONE && dist >= seg->dist_max_mm) state = ST_FINAL_APPROACH;
 *   // else if (in_window(dist) && side_in_band()) { confirm_start = dist; state = ST_CONFIRM_LANDMARK; }
 *   // else if (dist > seg->dist_max_mm) stop(LANDMARK_NOT_FOUND);
 *
 * ST_CONFIRM_LANDMARK
 *   Purpose : reject short detections. Continue at low speed.
 *   Needs   : side readings with status OK in the band CLEARANCE_BAND_MM_TBD
 *             for CONFIRM_DIST_MM_TBD of travel. Use distance, not time.
 *   Advance : confirmed -> ST_FOLLOW_AND_ALIGN
 *   Failure : pattern lost -> ST_APPROACH_SEARCH while the window is open.
 *   Note    : a sensor sees a surface at its own height only. It does not
 *             identify the type of object.
 *
 *   // if (!side_in_band()) state = ST_APPROACH_SEARCH;
 *   // else if (dist - confirm_start >= CONFIRM_DIST_MM_TBD) state = ST_FOLLOW_AND_ALIGN;
 *
 * ST_FOLLOW_AND_ALIGN
 *   Purpose : hold the lateral clearance and become parallel to the landmark.
 *   Needs   : the side sensor, distance, heading.
 *   Control : steering = K_CLEAR_TBD x clearance error + K_ANGLE_TBD x angle
 *             (angle from the change of the side distance, see "Sensors").
 *   Advance : clearance error < CLEAR_TOL_MM_TBD AND angle < ALIGN_TOL_DEG_TBD
 *             for ALIGN_DIST_MM_TBD -> record the heading as the landmark
 *             heading -> ST_FIND_REFERENCE
 *   Failure : the side reading NOT_VALID for more than GAP_DIST_MM_TBD
 *             -> ST_STOP (REFERENCE_LOST)
 *             For a shorter gap: hold the last heading. Do not steer on old data.
 *
 * ST_FIND_REFERENCE
 *   Purpose : find the reference feature, for example the end of the sidewalk.
 *             Continue to follow at low speed.
 *   Needs   : the side sensor with its status.
 *   Advance : a CONFIRMED edge = readings change from OK in the band to
 *             NO_TARGET (or OK and far) for EDGE_CONFIRM_DIST_MM_TBD, inside
 *             the window. Record the distance at the first such reading
 *             -> ST_FINAL_APPROACH
 *   Failure : NOT_VALID is not an edge. A gap shorter than
 *             EDGE_CONFIRM_DIST_MM_TBD is not an edge.
 *             Window end with no edge -> ST_STOP (REFERENCE_NOT_FOUND)
 *   Note    : there is only one sensor at each height. Thus no second sensor
 *             confirms the edge. The confirmation distance and the window are
 *             the only protection against a false edge.
 *
 * ST_FINAL_APPROACH
 *   Purpose : move final_offset_mm from the recorded edge, at the lowest speed,
 *             on the landmark heading. Then stop.
 *             Include SENSOR_OFFSET_MM_TBD: the distance from the sensor to the
 *             point of the car that defines the checkpoint position.
 *   Needs   : distance from the edge, heading, wheel speed.
 *   Advance : offset reached AND wheel speed = 0 for STANDSTILL_MS_TBD:
 *               position and heading in tolerance -> ST_WAIT_AT_CHECKPOINT
 *               not in tolerance                  -> ST_REALIGN
 *   Note    : encoder counts and a stop command do not prove 1 cm. Wheel slip
 *             and the edge detection error add to the result.
 *
 * ST_REALIGN
 *   Purpose : correct the position with a reverse move.
 *             A car with Ackermann steering cannot move sideways. Thus it
 *             reverses REALIGN_REVERSE_MM_TBD with the steering toward the
 *             correct line, then drives forward again.
 *   Needs   : the side sensor (if the landmark is still in range),
 *             heading, distance.
 *   Advance : reverse distance done -> ST_FINAL_APPROACH (same recorded edge)
 *   Failure : more than REALIGN_MAX_TRIES_TBD -> ST_STOP (POSITION_ERROR)
 *   Note    : in reverse, the same steering angle turns the heading in the
 *             opposite direction. The firmware already corrects this sign.
 *
 * ST_WAIT_AT_CHECKPOINT
 *   Purpose : stay stopped. Motors at zero.
 *   Needs   : a timestamp from the moment that standstill was confirmed.
 *   Advance : WAIT_AT_CHECKPOINT_MS_TBD (the challenge says 10 s)
 *             -> next segment: ST_APPROACH_SEARCH
 *             -> after the last segment: ST_STOP (FINISHED)
 *   Failure : the car moves during the wait -> ST_STOP (MOVED_AT_CHECKPOINT)
 *
 * ---------------------------------------------------------------------------
 * OBSTACLES (a challenge requirement)
 *
 * Trigger, from the DRIVE states: DIST_FRONT is OK and less than
 *   OBSTACLE_STOP_MM_TBD for OBSTACLE_CONFIRM_SAMPLES_TBD readings.
 *   Remember the state and the distance along the segment. -> ST_OBSTACLE_WAIT
 *   Exception: in ST_FINAL_APPROACH and ST_REALIGN the car only waits.
 *   It does not start the avoidance, because that loses the reference edge.
 *
 * ST_OBSTACLE_WAIT
 *   Purpose : stop and wait. A temporary obstacle goes away.
 *   Advance : front clear for OBSTACLE_CLEAR_MS_TBD -> the remembered state
 *             still blocked after OBSTACLE_WAIT_MS_TBD -> ST_AVOID_REVERSE
 *
 * ST_AVOID_REVERSE
 *   Purpose : move back AVOID_REVERSE_MM_TBD in a straight line, to get space
 *             to steer.
 *   Failure : nothing detects an object behind the car. Keep the distance short.
 *
 * ST_AVOID_OUT
 *   Purpose : drive forward with the steering to the left until the heading is
 *             AVOID_ANGLE_DEG_TBD from the segment heading.
 *   Failure : DIST_FRONT blocked again -> ST_STOP (OBSTACLE_BLOCKED)
 *
 * ST_AVOID_PASS
 *   Purpose : drive to pass the obstacle: first on the offset heading, then
 *             parallel to the segment for AVOID_PASS_MM_TBD.
 *   Failure : DIST_FRONT blocked -> ST_STOP (OBSTACLE_BLOCKED)
 *
 * ST_AVOID_RETURN
 *   Purpose : steer to the right, go back to the segment line, then steer to
 *             the segment heading.
 *   Advance : lateral offset near 0 AND heading = segment heading
 *             -> ST_APPROACH_SEARCH of the same segment. The landmark must be
 *             confirmed again. Do not go back into the middle of a follow state.
 *
 * Necessary for the avoidance:
 *   - Distance ALONG the segment = sum of (distance step x cos(heading - segment
 *     heading)). The wheel distance alone is too large after a detour.
 *   - Lateral offset = sum of (distance step x sin(heading - segment heading)).
 *   - AVOID_MAX_LATERAL_MM_TBD: the car has no sensor that sees the grass.
 *     The limit of the lateral move must come from a measurement of the path.
 *
 * ---------------------------------------------------------------------------
 * ST_STOP
 *   Purpose : motors off, steering centered, reason recorded and sent.
 *   Reasons : FINISHED, MANUAL_STOP, LANDMARK_NOT_FOUND, REFERENCE_NOT_FOUND,
 *             REFERENCE_LOST, POSITION_ERROR, MOVED_AT_CHECKPOINT,
 *             OBSTACLE_BLOCKED, STALL, HEADING_STALE, LINK_LOST.
 *   Advance : operator command only -> ST_START
 *
 * From any state: manual stop key, stall, stale heading or lost link -> ST_STOP.
 *   The firmware already detects these four conditions.
 *
 * Filter rule (all states)
 *   Reject single outliers with a median of DIST_FILTER_SAMPLES_TBD readings.
 *   Keep the filter short, so that it does not hide a real edge.
 *   Count the readings that are NOT_VALID. Do not replace them with old values.
 *
 * Telemetry (each cycle)
 *   state, segment, distance along the segment, lateral offset, heading,
 *   the three raw distance readings with their status (the "dist" telemetry line).
 *   The dashboard uses these values to set the _TBD constants.
 *
 * ---------------------------------------------------------------------------
 * Note: the distance sensor interface exists (App/Sensors/distance.h), but it is off
 *   (DISTANCE_ENABLE = 0) and its XSHUT pins are placeholders. It is not tested with
 *   real sensors. The ultrasonic driver (hcsr04.c) is not connected to it.
 * ==========================================================================*/
#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H
#endif /* STATE_MACHINE_H */
