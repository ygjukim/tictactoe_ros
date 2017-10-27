#pragma once

#include "tictactoe_ros/globals.h"

namespace tictactoe_ros {

class Board
{
public:
    int board[9];
    void Init();
    void SetSquare(int location, int player);
    void UnsetSquare(int location);
    int GetSquare(int location);
    GameState Evaluate(int player);
};

extern Board board;

}	// namespace tictactoe_ros