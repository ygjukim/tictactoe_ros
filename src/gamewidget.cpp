#include "tictactoe_ros/gamewidget.h"

#include <ros/ros.h>
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

#ifdef  SUPPORT_MOUSE_EVENT
enum GameControlState { PLAYER_PLAY = 0, PLAYER_MOVE, COMPUTER_PLAY, COMPUTER_MOVE };
enum ControlEventType { MOUSE_EVENT = 0, OBJECT_DETECTED };
#else
enum GameControlState { PLAYER_PLAY = 0, COMPUTER_PLAY, COMPUTER_MOVE };
enum ControlEventType { MOUSE_EVENT = 0, OBJECT_DETECTED };
#endif

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
    controlState = PLAYER_PLAY;

    connect(this, SIGNAL(updateControlState(int)), this, SLOT(onUpdateControlState(int)), Qt::QueuedConnection);
//    connect(this, SIGNAL(updateControlState(int)), this, SLOT(onUpdateControlState(int)));
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
        case O1: path = ":/images/O1.png"; break;
        case O: path = ":/images/O.png"; break;
        case X: path = ":/images/X.png"; break;
        case X1: path = ":/images/X1.png"; break;
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
#ifdef  SUPPORT_MOUSE_EVENT    
    if (controlState != PLAYER_PLAY)  return;

    for(int i = 0; i < 9; i++)
    {
        if(boardRects[i].contains(e->pos()) && displayBoard[i] == EMPTY)
        {
            if(CheckGame() == PLAY)
            {
                ROS_INFO("[CONTROL] Mouse clicked - Pos=%d", i);
                board.SetSquare(i, currentPly.player);
                displayBoard[i] = currentPly.player + ((currentPly.player > 0) ? 1 : -1);
                currentPly.cell = i;
                QWidget::update();             
                Q_EMIT updateControlState(MOUSE_EVENT);
            }
            break;
        }
    }
#endif    
}

void GameWidget::onObjectDetected(int pos, int type)
{
    if ((board.GetSquare(pos) == EMPTY) && (currentPly.player == type))
    {
        ROS_INFO("[CONTROL] Object detected - Pos=%d, Type=%d", pos, type);
        board.SetSquare(pos, type);
        displayBoard[pos] = type;
        QWidget::update();             
        Q_EMIT updateControlState(OBJECT_DETECTED);
    }
}

void GameWidget::onUpdateControlState(int eventType) {
    ROS_INFO("[CONTROL] State=%d, Event=%d", controlState, eventType);

#ifdef  SUPPORT_MOUSE_EVENT
    switch(controlState) {
    case PLAYER_PLAY:
        if (eventType == MOUSE_EVENT) {
            moveObject();
            controlState += 1;
        }
        break;

    case PLAYER_MOVE:
        if (eventType == OBJECT_DETECTED) {
            if (judgeGame() != PLAY) {
                controlState = PLAYER_PLAY;
            }
            else {
                currentPly.Toggle();
                controlState += 1;
                Q_EMIT updateControlState(eventType);
            }
        }
        break;

    case COMPUTER_PLAY:
        PlayAi();
        moveObject();
        controlState += 1;
        break;

    case COMPUTER_MOVE:
        if (eventType == OBJECT_DETECTED) {
            if (judgeGame() == PLAY) {
                currentPly.Toggle();
            }
            controlState = PLAYER_PLAY;
        }
        break;
    }
#else

    switch(controlState) {
    case PLAYER_PLAY:
        if (eventType == OBJECT_DETECTED) {
            if (judgeGame() != PLAY) {
                Reset();
            }
            else {
                currentPly.Toggle();
                controlState += 1;
                Q_EMIT updateControlState(eventType);
            }
        }
        break;

    case COMPUTER_PLAY:
        moveObject(PlayAi(), currentPly.player);
        controlState += 1;
        break;

    case COMPUTER_MOVE:
        if (eventType == OBJECT_DETECTED) {
            if (judgeGame() == PLAY) {
                currentPly.Toggle();
                controlState = PLAYER_PLAY;
            }
            else {
                Reset();
            }
        }
        break;
    }
#endif

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
//    Reset();
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
//    Reset();
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
//    Reset();
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
    currentPly.cell = -1;
    for(int i = 0; i < 9; i++)
    {
        displayBoard[i] = EMPTY;
    }

    controlState = PLAYER_PLAY;

    QWidget::update();
}

/****************************************
 * plays AI turn                        *
 ****************************************/
int GameWidget::PlayAi()
{
    Move bestMove = ai.Search(currentPly.player);

#ifdef  SUPPORT_MOUSE_EVENT
    board.SetSquare(bestMove.location, currentPly.player);
    displayBoard[bestMove.location] = currentPly.player + ((currentPly.player > 0) ? 1 : -1);
    currentPly.cell = bestMove.location;
    QWidget::update();
    return 0;
#else
    displayBoard[bestMove.location] = currentPly.player + ((currentPly.player > 0) ? 1 : -1);
    QWidget::update();
    return bestMove.location;
#endif                 
}

int GameWidget::judgeGame() {
    int gameState = CheckGame();

    switch (gameState) {
    case WIN:
        if(currentPly.player == player) Win();
        else Lose();
        break;
    case LOSE:
        if(currentPly.player == player) Lose();
        else Win();
        break;
    case TIE:
        Tie();
        break;
    }

    return gameState;
}

void GameWidget::moveObject(int pos, int type) {
    // no operation...
}


}   // namespace tictactoe_ros
