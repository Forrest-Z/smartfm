
class Blinker {

  public:
    Blinker(uint8_t pin) : _pin(pin), _blink(0), _state(0) {
      pinMode(pin, OUTPUT);
      half_period(0.5);
    }

    void blink() {
      if( _blink ) return; //already blinking
      digitalWrite(_pin, HIGH);
      _lastTime = millis();
      _blink = 1;
      _state = 1;
    }

    void off() {
      if( !_blink ) return; //already off
      digitalWrite(_pin, LOW);
      _blink = 0;
      _state = 0;
    }

    void set(int b) { b ? blink() : off(); }

    bool isOn() { return _blink!=0; }
    bool isOff() { return _blink==0; }

    void half_period(float sec) { _half_period = (unsigned long) (sec*1000); }


    void run() {
      if( !_blink )
        return;
      unsigned long t = millis();
      if( t - _lastTime > _half_period ) {
        _lastTime = t;
        _state = !_state;
        if( _state )
          digitalWrite(_pin, HIGH);
        else
          digitalWrite(_pin, LOW);
      }
    }

  private:
    uint8_t _pin;
    int _blink; // are we in blinking mode
    int _state; // is the light on or off
    unsigned long _lastTime;
    unsigned long _half_period; //in micro seconds
};
