#ifndef EXO_COMMAND_HEADER
#define EXO_COMMAND_HEADER

class Exoskeleton;
class Leg;
class Joint;

template <class T>
class Command{
public:
  virtual ~Command();
  virtual void execute(T* context) = 0;
};

class StartTrialCommand:Command<Exoskeleton>{
public:
  virtual void execute();
};

#endif
