#ifndef ENCMINIM_H
#define ENCMINIM_H
#include <Arduino.h>


#pragma once
#define _ME_ENC_FAST 30
#define _ME_BUTTON_DEB 80
#define _ME_BUTTON_HOLD 2000
// ================= УПРАВЛЕНИЕ =================
#define ENCODER_TYPE 1        // тип энкодера (0 или 1). Если энкодер работает некорректно (пропуск шагов/2 шага), смените тип
#define ENC_REVERSE 1         // 1 - инвертировать направление энкодера, 0 - нет
#define CONTROL_TYPE 1        // тип управления энкодером: 0 - удерживание и поворот для изменения значения, 1 - клик для входа в изменение, повторный клик для выхода (стрелочка меняется на галочку)
#define FAST_TURN 1           // 1 - вкл быстрый поворот
#define FAST_TURN_STEP 10     // изменение при быстром повороте



class encMinim
{
  public:
    encMinim(uint8_t clk, uint8_t dt, int8_t sw, bool dir, bool type = 0);
    encMinim(const encMinim&) = default;
    encMinim& operator=(const encMinim&) = default;
    encMinim(encMinim&&) = delete;
    encMinim& operator=(encMinim&&) = delete;
    void tick(bool hold = false);
    bool isClick();
    bool isHolded();
    bool isTurn();
    bool isRight();
    bool isLeft();
    bool isRightH();
    bool isLeftH();
    bool isFast();

    // 0 - nothing, 1 - left, 2 - right, 3 - rightPressed, 4 - leftPressed, 5 - click, 6 - held
    byte getState();

    // reset state
    void rst();

  private:
    int8_t _clk, _dt, _sw;
    volatile bool _fast = false;
    volatile bool _swState, _swFlag, _turnState, _holdFlag;
    volatile byte _state, _lastState, _encState;
    volatile bool _resetFlag = false;
    volatile uint32_t _debTimer;
    // _encState: 0 - nothing, 1 - left, 2 - right, 3 - rightPressed, 4 - leftPressed, 5 - click, 6 - held
};


extern encMinim enc;


#endif // ENCMINIM_H_

