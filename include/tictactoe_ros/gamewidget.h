#ifndef GAMEWIDGET_H
#define GAMEWIDGET_H

#include "tictactoe_ros/ai.h"
#include "tictactoe_ros/board.h"
#include <QWidget>


namespace tictactoe_ros {

class GameWidget : public QWidget
{
    Q_OBJECT
public:
    explicit GameWidget(QWidget *parent = nullptr);
    ~GameWidget();

    void Reset();

signals:
    void updateControlState(int eventType);

public slots:
    void onObjectDetected(int pos, int type);
    void onUpdateControlState(int eventType);
    
protected:
    void paintEvent(QPaintEvent *e);
    void mousePressEvent(QMouseEvent *e);
    void moveObject(int pos, int type);

private:
//    bool userTurn;
    int player;
    int controlState;
    QRect boardRects[9];

    int judgeGame();
    GameState CheckGame();
    void Tie();
    void Lose();
    void Win();
    int PlayAi();
};

}   // namespace tictactoe_ros

#endif // GAMEWIDGET_H
