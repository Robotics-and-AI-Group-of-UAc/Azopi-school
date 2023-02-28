#ifndef PID_simple_h
#define  PID_simple_h

class PID {
public:
  

  PID(double *,
      double *,
      double *,
      double,
      double,
      double);

  bool   Compute();
  //@Initialize...
  bool   Initialize();
  double GetP();
  double GetI();
  double GetD();

private:
  int initialize; //Starting PID (derivative = 0)
  double *_setpoint;
  double *_encread;
  double *_output;

  double _kp;
  double _ki;
  double _kd;

  double dispP;
  double dispI;
  double dispD;

  double last_proportional;
  double error, proportional, integral, derivative;
};

#endif // ifndef PID_simple_h
