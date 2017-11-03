#pragma once

namespace tictactoe_ros {

enum GameState { WIN = 6, LOSE = -6, TIE = 0, PLAY = 1, END = -1};
enum { X1 = 2, X = 1, O = -1, O1 = -2, EMPTY = 0 };

struct Move
{
    int score;
    int location;

    Move(){}
    Move(int s) : score(s) {}
};

struct CurrentPly
{
    int player;
    int cell;

    CurrentPly() : player(X), cell(-1) {}
    inline void Toggle()
    {
        player *= -1;
    }
};

extern int displayBoard[9];
extern CurrentPly currentPly;

}   // namespace tictactoe_ros