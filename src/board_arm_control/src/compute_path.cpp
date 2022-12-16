/*
 * Kenneth Moore
 * December 2022
 */

#include <cstdio>
#include <cstdlib>
#include <vector>
#include <boost/array.hpp>
#include "ros/ros.h"
#include "board_arm_control/FindLatticePath.h"

using board_arm_control::FindLatticePath;

// Detects which quadrant position lies in and updates returnQuadrant to match (1 = Top Right, 2 = Top Left, 3 = Bottom Left, 3 = Bottom Right)
void detectQuadrant(int8_t* position, int8_t* returnQuadrant){
    const int8_t MIDDLE = 10;
    if(position[0] < MIDDLE && position[1] > MIDDLE){
        *returnQuadrant = 1;
        return;
    }
    else if(position[0] < MIDDLE && position[1] < MIDDLE){
        *returnQuadrant = 2;
        return;
    }
    else if(position[0] > MIDDLE && position[1] < MIDDLE){
        *returnQuadrant = 3;
        return;
    }
    else{
        *returnQuadrant = 4;
        return;
    }
}

// Detects which border position relates to. Border is a array as follows [(0 = no border || 1 = top border || 2 = bottom border), (0 = no border || 1 = left border || 2 = right border)]
void detectBorder(int8_t* position, int8_t* border){
    // Set the correct border values
    if(position[0]==1){
        border[0] = 1;
    }
    else if(position[0]==19){
        border[0] = 2;
    }
    else{
        border[0] = 0;
    }
    if(position[1]==1){
        border[1] = 1;
    }
    else if(position[1]==19){
        border[1] = 2;
    }
    else{
        border[1] = 0;
    }
    return;
}

void moveAvoidMiddle(int8_t* from, int8_t* to, int8_t* moves){
    int8_t fromQuadrant, toQuadrant;
    int8_t fromBorder[2], toBorder[2];
    detectQuadrant(from, &fromQuadrant);
    detectQuadrant(to, &toQuadrant);
    detectBorder(from, fromBorder);
    detectBorder(to, toBorder);

    // If column remains the same after move, don't move to nearest row-edge
    if(from[1]==to[1]){
        moves[0] = 0;
    }
    // If starting position is a border
    if(fromBorder[0] != 0){
        if(fromBorder[0]==1){
            moves[0] = 1;
        }
        else{
            moves[0] = -1;
        }
    }
    else{
        // If in top quadrants move to row edge above current pos, otherwise move to row edge below current pos
        if(fromQuadrant == 1 || fromQuadrant == 2){
            moves[0] = -1;
        }
        else{
            moves[0] = 1;
        }
    }

    // If row remains the same after move, enter
    if(from[0]==to[0]){
      // Move to correct column along current row edge
      moves[1] = to[1]-from[1];
      // Place piece in position (off of row edge)
      moves[2] = moves[0]*-1;
      moves[3] = 0;
    }
    // If row changes after move, enter
    else{
        // If to position is in Q2 or Q3 and not on the left border, or position is along right border, go to left edge of target, otherwise go to right
        int8_t shift = 1;
        if(((toQuadrant == 2 || toQuadrant == 3) && toBorder[1]!=1) || toBorder[1] == 2){
            shift = -1;
        }
        moves[1] = to[1]-from[1] + shift;
        moves[2] = to[0]-from[0] + (-1*moves[0]);
        moves[3] = -shift;
    }

}

bool handler(
        board_arm_control::FindLatticePath::Request& req,
        board_arm_control::FindLatticePath::Response& res)
{
    int8_t moves[4];
    int8_t p1[2];
    int8_t p2[2];
    p1[0] = req.positions[0].point[0];
    p1[1] = req.positions[0].point[1];
    p2[0] = req.positions[1].point[0];
    p2[1] = req.positions[1].point[1];

    printf("(%i, %i)\n", p1[0], p1[1]);
    printf("(%i, %i)\n", p2[0], p2[1]);
    moveAvoidMiddle(p1, p2, moves);
    for (int8_t i = 0; i < 4; i += 1) {
        printf("%i ", moves[i]);
    }
    printf("\n");
    res.path[0].point[0] = p1[0] + moves[0];
    res.path[0].point[1] = p1[1];
    res.path[1].point[0] = res.path[0].point[0];
    res.path[1].point[1] = res.path[0].point[1] + moves[1];
    res.path[2].point[0] = res.path[1].point[0] + moves[2];
    res.path[2].point[1] = res.path[1].point[1];
    res.path[3].point[0] = res.path[2].point[0];
    res.path[3].point[1] = res.path[2].point[1] + moves[3];
    return true;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "find_path_server");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("find_path", handler);
    printf("awaiting requests...\n");
    ros::spin();
    return 0;
}
