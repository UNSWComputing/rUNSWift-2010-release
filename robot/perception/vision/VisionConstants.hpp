/*
Copyright 2010 The University of New South Wales (UNSW).

This file is part of the 2010 team rUNSWift RoboCup entry. You may
redistribute it and/or modify it under the terms of the GNU General
Public License as published by the Free Software Foundation; either
version 2 of the License, or (at your option) any later version as
modified below. As the original licensors, we add the following
conditions to that license:

In paragraph 2.b), the phrase "distribute or publish" should be
interpreted to include entry into a competition, and hence the source
of any derived work entered into a competition must be made available
to all parties involved in that competition under the terms of this
license.

In addition, if the authors of a derived work publish any conference
proceedings, journal articles or other academic papers describing that
derived work, then appropriate academic citations to the original work
must be included in that publication.

This rUNSWift source is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License along
with this source code; if not, write to the Free Software Foundation,
Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

#pragma once

/**
 * FIELD_EDGE_DETECTION
 **/

/**
 * The minimum number of green points along the top row of the image
 * for the robot to be considered to be viewing the field in the case
 * where no field edge lines are detected
 **/
#define MIN_GREEN_THRESHOLD 25

/**
 * GOAL DETECTION
 **/

/**
 * The minimum number for the histogram point to be
 * considered a maximum
 **/
#define Y_HIST_MAX_LIMIT 3     // 10
#define X_HIST_MAX_LIMIT 10
/**
 * The number of times below a peak the histogram
 * value has to be be for new peaks to be considered
 **/
#define HIST_MIN_LIMIT 3
/**
 * The largest gaps allowed between goal colours 
 **/
#define LARGEST_GAP_VERTICAL 7
#define LARGEST_GAP_HORIZONTAL 3
/**
 * The minimum goal dimensions allowable
 **/
#define MINIMUM_GOAL_WIDTH 3
#define MINIMUM_GOAL_HEIGHT 20
/**
 * The height of the goals must be at least this amount
 * greater than the width it to be considered as a goal
 **/
#define MINIMUM_GOAL_RATIO 1.5
/**
 * The minimum value in the histogram entry for the goals to be
 * consiered finished
 **/
#define HIST_GOAL_END_THRESH 1
/**
 * The distance travelled from the midpoint of the goals
 * until a point in the histogram where HIST_GOAL_END_THRESH
 * is reached must be this amount greater in one direction
 * before the post can be determined to be left or right
 **/
#define HIST_GOAL_SEP_THRESH 7
/**
 * The number of adjacent times the histogram values can
 * be below HIST_GOAL_END_THRESH before the goals are
 * considered to have finished
 **/
#define HIST_SKIP_ALLOWANCE 3
/**
 * The maximum distance where accurate distance measurements can be
 * made from the width of each goal post
 **/
#define MAX_DIST_FOR_GOAL_POST_WIDTH 4500
/**
 * The minimum difference in the y, u, v values for an edge to 
 * be detected
 **/
// #define GOAL_EDGE_THRESHOLD 10
/**
 * The minimum allowable height of a goal when both the top and bottom
 * of the goal can be seen
 **/
#define MINIMUM_TOTAL_GOAL_HEIGHT 90
/**
 * The minimum allowable ratio between the height and width of the goal
 * when the top of the goal can't be seen
 **/
#define MINIMUM_GOAL_PARTIAL_RATIO 0.7

/**
 * REGION BUILDING
 **/

/**
 * The maximum number of regions allowable for a single frame
 **/
#define MAX_NUM_REGIONS 200
/**
 * The maximum number of scan points that a single region can store
 **/
#define MAX_NUM_SCAN_POINTS 200
/**
 * The allowable number of non-orange pixels in a row before a scan
 * line is finished
 **/
#define BALL_CLASS_THRESH 1
/**
 * The number of green pixels that can be seen before a scan is stopped
 **/
#define REGION_CLASS_THRESH 1
/**
 * The smallest length length allowed for a scan line to be added to
 * a region
 **/
#define MINIMUM_SCAN_LENGTH 0
/**
 * Used to determine if a region should be joined, such as the maximum
 * number of white scan lines that there can be to join the region to
 * a scan line with robot colours, or the maximum number of scan lines
 * with no robot colours that there can be before a new region is 
 * created
 **/
#define REGION_JOIN_GAP 5
/**
 * The same as REGION_JOIN_GAP, but used on regions that start from the
 * field edge line. As these are more likely to be robot regions, a more
 * generous threshold can be applied
 **/
#define REGION_JOIN_GAP_ON_FIELD_EDGE 12
/**
 * The maximum number of times a scan length can be different from the
 * average scan length of the region for it to be joined to the region
 **/
#define AVERAGE_SCAN_LENGTH_THRESH 2
/**
 * The maximum number of image regions that can make up a robot region
 **/
#define MAX_NUM_ROBOT_REGIONS 20

/**
 * BALL DETECTION
 **/

/**
 * The difference in the sum of the absolute values of the y, u and v
 * components of two pixels needed for the pixels to be considered
 * an edge
 **/
// #define BALL_EDGE_THRESHOLD 35
// #define BALL_EDGE_THRESHOLD 10

/**
 * The maximum number of points on the edge of the ball that can be 
 * recorded
 **/
#define MAX_BALL_EDGE_POINTS 1000

/**
 * The number of repeats used when calculating the ball centre
 * and radius
 **/
#define NUM_CENTRE_REPEATS 40

/**
 * The maximum number of pixels in the ball region that can be present
 * for the ball detection to scan every line of the full res image.
 **/
#define FINE_SCAN_THRESHOLD 15

/**
 * The maximum number of pixels in the ball region that can be present
 * before scanning for the ball edges at the lowest resolution
 **/
#define LARGE_SCAN_THRESHOLD 150

/**
 * The maximum radius a ball can have before it is thrown out
 **/
#define MAXIMUM_BALL_RADIUS 82

/**
 * The minimum radius a ball can have for it to be considered
 **/
#define MINIMUM_BALL_RADIUS 2

/**
 * The minimum difference in edge points that is required for a
 * ball to be considered detected
 **/
#define MINIMUM_BALL_WIDTH 3

/**
 * The minimum number of edge points that can be found to proceed
 * with further ball processing
 **/
#define MINIMUM_NUM_EDGE_POINTS 3

/**
 * The amount the distance calculated from the radius can be different 
 * from the kinematics distance for the radius distance to be used.
 **/
#define BALL_DISTANCE_TOLERANCE 3

/**
 * The number of pixels on each side of a small ball that are searched
 * to determine if the ball is a background ball that should be
 * deleted
 **/
#define EXCLUDE_BALL_PIXELS 10

/**
 * ROBOT DETECTION
 **/

/**
 * The number of robot colours required in a potential robot region
 * for a robot to be reported if the colours are above the field line
 * and if the colours are below the field line
 **/
#define NUM_ROBOT_COLOURS_REQUIRED_ABOVE 3
#define NUM_ROBOT_COLOURS_REQUIRED_BELOW 15
/**
 * The number of rows in the saliency scan above the field edge that
 * are scanned when search for robot colours
 **/
#define NUM_ROWS_ABOVE 15
/**
 * The minimum width of a region for it to be considered a robot
 **/
#define MIN_ROBOT_REGION_WIDTH 4
/**
 * The minimum number of white pixels in a potential robot region
 * for the region to be considered a robot
 **/
#define MIN_NUM_WHITE_IN_ROBOT 5

