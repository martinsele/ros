#ifndef TECH_TASK_TASKREACHEDCALLBACK_H
#define TECH_TASK_TASKREACHEDCALLBACK_H


/**
 * Callback for information about reached task
 */
class TaskReachedCallback{
public:
    virtual ~TaskReachedCallback(){};
    virtual void taskReached() = 0;
};

#endif //TECH_TASK_TASKREACHEDCALLBACK_H
