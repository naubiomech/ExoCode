#ifndef JOINT_SELECT_HEADER
#define JOINT_SELECT_HEADER

typedef int AreaID;
class AreaIDSelect{
public:
	static const AreaID LEFT_LEG = 0;
	static const AreaID RIGHT_LEG = 1;
};

typedef int JointID;
class LegJointIDSelect{
public:
	static const JointID ANKLE_JOINT = 0;
	static const JointID KNEE_JOINT = 1;
};

class JointSelect{
public:
	AreaIDSelect area_id;
	LegJointIDSelect leg_joint_id;
};

#endif
