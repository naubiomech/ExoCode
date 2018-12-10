#ifndef EXO_COMMAND_HEADER
#define EXO_COMMAND_HEADER

class Exoskeleton;
class Leg;
class Joint;


template <class T>
class Command{
public:
	virtual void execute(T* context) = 0;
};

class JointCommand:public Command<Joint>{};
class LegCommand:public Command<Leg>{};
class ExoCommand:public Command<Exoskeleton>{};

class StartTrialCommand:ExoCommand{
public:
	virtual void execute();
};
#endif
