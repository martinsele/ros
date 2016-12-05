#ifndef TECH_TASK_COMMUNICATORCALLBACK_H
#define TECH_TASK_COMMUNICATORCALLBACK_H


/**
 * Callback for information about reached task
 */
class CommunicatorCallback{
public:
    virtual ~CommunicatorCallback(){};
    virtual void missionArrived() = 0;
};

#endif //TECH_TASK_COMMUNICATORCALLBACK_H
