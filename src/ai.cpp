#include "tictactoe_ros/ai.h"
#include "tictactoe_ros/board.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace tictactoe_ros {

Move Ai::Search(int player)
{
        switch(board.Evaluate(player))
        {
        case WIN: return WIN_SCORE;
        case LOSE: return -WIN_SCORE;
        case TIE: return TIE_SCORE;
        }

        Move bestMove;
        bestMove.score = -INF;
        for(int i = 0; i < 9; i++)
        {
            if(board.GetSquare(i) == EMPTY)
            {
                Move move;
                move.location = i;
                board.SetSquare(i, player);
                move.score = -Search(-player).score;
                board.UnsetSquare(i);
                if(move.score > bestMove.score)
                    bestMove = move;
            }
        }
        return bestMove;
}

}   // namespace tictactoe_ros