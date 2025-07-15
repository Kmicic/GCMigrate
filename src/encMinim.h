#ifndef ENCMINIM_H
#define ENCMINIM_H
#endif

#pragma once
#define _ME_ENC_FAST 30
#define _ME_BUTTON_DEB 80
#define _ME_BUTTON_HOLD 2000

class encMinim
{
  public:
    encMinim(uint8_t clk, uint8_t dt, int8_t sw, boolean dir, boolean type = 0);
    void tick(bool hold = false);
    boolean isClick();
    boolean isHolded();
    boolean isTurn();
    boolean isRight();
    boolean isLeft();
    boolean isRightH();
    boolean isLeftH();
    boolean isFast();

    // 0 - nothing, 1 - left, 2 - right, 3 - rightPressed, 4 - leftPressed, 5 - click, 6 - held
    byte getState();

    // reset state
    void rst();

  private:
    int8_t _clk, _dt, _sw;
    volatile bool _fast = false;
    volatile boolean _swState, _swFlag, _turnState, _holdFlag;
    volatile byte _state, _lastState, _encState;
    volatile bool _resetFlag = false;
    volatile uint32_t _debTimer;
    // _encState: 0 - nothing, 1 - left, 2 - right, 3 - rightPressed, 4 - leftPressed, 5 - click, 6 - held
};

encMinim::encMinim(uint8_t clk, uint8_t dt, int8_t sw, boolean dir, boolean type) {
  if (dir) {
    _clk = clk;
    _dt = dt;
  } else {
    _clk = dt;
    _dt = clk;
  }
  _sw = sw;
  pinMode(_clk, INPUT);
  pinMode(_dt, INPUT);
  pinMode(_sw, INPUT_PULLUP); // for mode without button
}

void encMinim::tick(bool hold) {
  //static uint32_t thisMls = millis();
  //if (millis() - thisMls < 5) return;
  uint32_t thisMls = millis();
  uint32_t debounce = thisMls - _debTimer;

  // znak
  _state = digitalRead(_clk) | digitalRead(_dt) << 1;
  if (_resetFlag && _state == 0b11) {
    if (_lastState == 0b10) _encState = (!_swState || hold) ? 3 : 1;
    if (_lastState == 0b01) _encState = (!_swState || hold) ? 4 : 2;
    if (_encState != 0 && debounce < _ME_ENC_FAST) _fast = true;
    else _fast = false;
    _debTimer = thisMls;
    _turnState = true;
    _resetFlag = 0;
  }
  if (_state == 0b00) _resetFlag = 1;
  _lastState = _state;

  // button
  _swState = digitalRead(_sw);
  if (!_swState && !_swFlag && debounce > _ME_BUTTON_DEB) {
    _debTimer = thisMls;
    debounce = 0;
    _swFlag = true;
    _turnState = false;
    _holdFlag = false;
  }

  if (!_swState && _swFlag && !_holdFlag) {
    if (_encState != 0 && debounce < _ME_BUTTON_HOLD) {
      _holdFlag = true;
    }
    if (_encState == 0 && debounce > _ME_BUTTON_HOLD) {
      _encState = 6;
      _holdFlag = true;
    }
  }

  if (_swState && _swFlag) {
    if (_holdFlag) {
      debounce = 0;
    } else {
      if (debounce > _ME_BUTTON_DEB) {
        if (!_turnState) _encState = 5;
      }
    }
    _debTimer = thisMls;
    _swFlag = false;
  }
}

byte encMinim::getState() {
  return _encState;
}
void encMinim::rst() {
  _encState = 0;
}
boolean encMinim::isFast() {
  if (_fast) return true;
  else return false;
}
boolean encMinim::isTurn() {
  if (_encState > 0 && _encState < 5) {
    return true;
  } else return false;
}
boolean encMinim::isRight() {
  if (_encState == 1) {
    _encState = 0;
    return true;
  } else return false;
}
boolean encMinim::isLeft() {
  if (_encState == 2) {
    _encState = 0;
    return true;
  } else return false;
}
boolean encMinim::isRightH() {
  if (_encState == 3) {
    _encState = 0;
    return true;
  } else return false;
}
boolean encMinim::isLeftH() {
  if (_encState == 4) {
    _encState = 0;
    return true;
  } else return false;
}
boolean encMinim::isClick() {
  if (_encState == 5) {
    _encState = 0;
    return true;
  } else return false;
}
boolean encMinim::isHolded() {
  if (_encState == 6) {
    _encState = 0;
    return true;
  } else return false;
}
