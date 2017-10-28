#include "tictactoe_ros/gamewidget.h"
#include <QPainter>
#include <QMouseEvent>
#include <QMessageBox>

#define RECT_SIZE       120

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace tictactoe_ros {

using namespace Qt;


int displayBoard[9];
Board board;
CurrentPly currentPly;
Ai ai;


GameWidget::GameWidget(QWidget *parent) : QWidget(parent)
{
    // Create the gameboard as a matrix of
    // 9 rects each of 100 x 100 size
    QSize size(RECT_SIZE, RECT_SIZE);
    boardRects[0] = QRect(QPoint(0, 0), size);
    boardRects[1] = QRect(QPoint(0, RECT_SIZE), size);
    boardRects[2] = QRect(QPoint(0, RECT_SIZE*2), size);
    boardRects[3] = QRect(QPoint(RECT_SIZE, 0), size);
    boardRects[4] = QRect(QPoint(RECT_SIZE, RECT_SIZE), size);
    boardRects[5] = QRect(QPoint(RECT_SIZE, RECT_SIZE*2), size);
    boardRects[6] = QRect(QPoint(RECT_SIZE*2, 0), size);
    boardRects[7] = QRect(QPoint(RECT_SIZE*2, RECT_SIZE), size);
    boardRects[8] = QRect(QPoint(RECT_SIZE*2, RECT_SIZE*2), size);

    // Set player symbol to X
    player = X;
}

GameWidget::~GameWidget()
{

}

/***************************************
 * paintEvent function                 *
 ***************************************/
void GameWidget::paintEvent(QPaintEvent *e)
{
//    painter = new QPainter(this);
    QPainter painter(this);

    painter.fillRect(this->rect(), QColor(186, 168, 210));

    QPen borderPen(Qt::yellow);
    borderPen.setWidth(4);

    painter.setPen(borderPen);
    painter.drawRects(boardRects, 9);

    for(int i = 0; i < 9; i++)
    {
        QString path;
        switch(displayBoard[i])
        {
        case EMPTY: path = ""; break;
        case X: path = ":/images/X.png"; break;
        case O: path = ":/images/O.png"; break;
        }

        if (path != "") {
            painter.drawPixmap(boardRects[i].x()+10, boardRects[i].y()+10,
                               boardRects[i].width()-20, boardRects[i].height()-20,
                               QPixmap(path));
        }
    }
}

/***************************************
 * mousePressEvent() is called when    *
 * the player presses a mouse button.  *
 * It contains the game loop           *
 ***************************************/
void GameWidget::mousePressEvent(QMouseEvent *e)
{
    for(int i = 0; i < 9; i++)
    {
        if(boardRects[i].contains(e->pos()) && displayBoard[i] == EMPTY)
        {
            if(CheckGame() == PLAY)
            {
                board.SetSquare(i, currentPly.player);
                displayBoard[i] = currentPly.player;
                QWidget::update();
            }

            if(CheckGame() == WIN)
            {
                if(currentPly.player == player)
                    Win();
                else
                    Lose();
                return;
            }
            if(CheckGame() == LOSE)
            {
                if(currentPly.player == player)
                    Lose();
                else
                    Win();
                return;
            }
            if(CheckGame() == TIE)
            {
                Tie();
                return;
            }
            if(CheckGame() == PLAY)
            {
                currentPly.Toggle();

                PlayAi();
                QWidget::update();
            }

            if(CheckGame() == WIN)
            {
                if(currentPly.player == player)
                    Win();
                else
                    Lose();
                return;
            }
            if(CheckGame() == LOSE)
            {
                if(currentPly.player == player)
                    Lose();
                else
                    Win();
                return;
            }
            if(CheckGame() == TIE)
            {
                Tie();
                return;
            }

            if(CheckGame() == PLAY)
            {
                currentPly.Toggle();
                return;
            }

            break;
        }
    }
}

void GameWidget::onObjectDetected(int pos, int type)
{
    if (displayBoard[pos] != type) {
        displayBoard[pos] = type;
        QWidget::update();
    }
}

/***************************************
 * Shows "tie" message                 *
 ***************************************/
void GameWidget::Tie()
{
    QMessageBox tie;
    tie.setWindowTitle("Result");
    tie.setText("Tie!");
    tie.setFixedSize(600, 400);
    tie.exec();
    Reset();
}

/***************************************
 * Shows "lose" message, when player   *
 * loses the game                      *
 ***************************************/
void GameWidget::Lose()
{
    QMessageBox lose;
    lose.setWindowTitle("Result");
    lose.setText("You lose!");
    lose.setFixedSize(600, 400);
    lose.exec();
    Reset();
}

/***************************************
 * Shows "win" message, when player    *
 * wins the game. No way this function *
 * will be called in a perfect search. *
 ***************************************/
void GameWidget::Win()
{
    QMessageBox win;
    win.setWindowTitle("Result");
    win.setText("You win!");
    win.setFixedSize(600, 400);
    win.exec();
    Reset();
}

/***************************************
 * CheckGame() checks the game status  *
 * using board.Evaluate() to control   *
 * the game flow appropriately         *
 ***************************************/
GameState GameWidget::CheckGame()
{
    return board.Evaluate(currentPly.player);
}

/****************************************
 * Resets the game for a new match      *
 ****************************************/
void GameWidget::Reset()
{
    board.Init();
    currentPly.player = X;
    for(int i = 0; i < 9; i++)
    {
        displayBoard[i] = EMPTY;
    }

    QWidget::update();
}

/****************************************
 * plays AI turn                        *
 ****************************************/
void GameWidget::PlayAi()
{
    Move bestMove = ai.Search(currentPly.player);
    board.SetSquare(bestMove.location, currentPly.player);
    displayBoard[bestMove.location] = currentPly.player;
}

}   // namespace tictactoe_ros
