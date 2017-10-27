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

signals:

public slots:

protected:
    void paintEvent(QPaintEvent *e);
    void mousePressEvent(QMouseEvent *e);

private:
//    bool userTurn;
    int player;
    QRect boardRects[9];

    GameState CheckGame();
    void Reset();
    void Tie();
    void Lose();
    void Win();
    void PlayAi();
};

}   // namespace tictactoe_ros

#endif // GAMEWIDGET_H
