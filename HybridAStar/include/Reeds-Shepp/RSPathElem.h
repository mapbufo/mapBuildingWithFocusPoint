#pragma once
#include <vector>
#include <unordered_map>

int RS_NOP = 0;
int RS_LEFT = 1;
int RS_STRAIGHT = 2;
int RS_RIGHT = 3;

std::unordered_map<std::string, std::vector<int>>
    RSPathType =
        {
            {"Type01", {RS_LEFT, RS_RIGHT, RS_LEFT, RS_NOP, RS_NOP}},
            {"Type02", {RS_RIGHT, RS_LEFT, RS_RIGHT, RS_NOP, RS_NOP}},
            {"Type03", {RS_LEFT, RS_RIGHT, RS_LEFT, RS_RIGHT, RS_NOP}},
            {"Type04", {RS_RIGHT, RS_LEFT, RS_RIGHT, RS_LEFT, RS_NOP}},
            {"Type05", {RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_NOP}},
            {"Type06", {RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_NOP}},
            {"Type07", {RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_LEFT, RS_NOP}},
            {"Type08", {RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_RIGHT, RS_NOP}},
            {"Type09", {RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_NOP}},
            {"Type10", {RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_NOP}},
            {"Type11", {RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_LEFT, RS_NOP}},
            {"Type12", {RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_LEFT, RS_NOP}},
            {"Type13", {RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_LEFT, RS_NOP}},
            {"Type14", {RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_NOP, RS_NOP}},
            {"Type15", {RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_NOP, RS_NOP}},
            {"Type16", {RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_NOP, RS_NOP}},
            {"Type17", {RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_RIGHT}},
            {"Type18", {RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_LEFT}},

};
