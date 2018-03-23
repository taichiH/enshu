#ifndef __NEGOMO_PLANNER_INCLUDE__
#define __NEGOMO_PLANNER_INCLUDE__

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <negomo/WaitInterpolationRequest.h>
#include <negomo/PlannerInteractionCall.h>
#include <negomo/PlannerDefaultInteractionCall.h>
#include <negomo/VpActivate.h>
#include <negomo/VpConnect.h>
#include <negomo/PlannerActionCall.h>
#include <negomo/PlannerBridgeRequest.h> // for option consts
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <functional>
#include <stack>
#include <mutex>
#include <boost/property_tree/ptree.hpp>


namespace negomo_lib
{
  typedef std::function<int(int,int&)> ActionFunc;
  // in_usedhands, out_usedhands, list_of_forwardfuncs, list_of_backwardfuncs, keyword
  typedef std::vector<std::tuple<int, int, ActionFunc, ActionFunc, std::string>> ActionList;
  typedef std::shared_ptr<ActionList> ActionListPtr;

  struct jumpSettings {
    jumpSettings() {
      preaction = 0; // action id for entering interaction
      postaction_true = 0; // action id for ending interaction
      postaction_false = 0; // action id for ending false interaction
    };

    int preaction;
    int postaction_true;
    int postaction_false;
  };

  struct waitSettings {
    waitSettings() { // see PlannerBridgeRequest.srv for details on parameters
      target = -1;
      threshold = 0.3;
      options = 0;
      interact_only_once = true;
      enable_preinteraction = true;
      warn_avoid = false;
      exec_time_ms = -1;
      interaction_flag = negomo::PlannerBridgeRequest::Request::OPENALL;
      at_end = 0;
      backward_allowed = 1;
      max_queue_size = 3;
    };

    int target;
    float threshold;
    int options;
    bool interact_only_once;
    bool enable_preinteraction;
    bool warn_avoid;
    int exec_time_ms;
    int interaction_flag;
    int at_end;
    int backward_allowed;
    int max_queue_size;
  };

  // NegomoPlanner is a task planner using negomo.
  // Planner allows interactive interruptions during a task.
  class NegomoPlanner
  {
  // @brief Constructor.
  // @param[in] _nh: ROS node handle.
  // @param[in] _ns: Namespace of corresponding negomo node.
  // @param[in] _wsf: moveToWorkspace binding function.
  // @param[in] _use_default: Use shortInteraction() as interaction callback.
  // @param[in] _use_base: Call base method getWaitInterpolation(). 
  public: explicit NegomoPlanner
  (ros::NodeHandle _nh, std::string _ns, ActionFunc _wsf, bool _use_default=false, bool _use_base=false);

  public: ~NegomoPlanner();

  // @brief Disable iJoin, iStart (for task debugging).
  public: void iOff();

  // @brief Enable iJoin, iStart
  public: void iOn();

  // @brief Postpone iStart for _ms milliseconds.
  // @param[in] _ms: How long to postpone iStart.
  public: void iOnTimered(int _ms);

  // @brief Save error message before returning exception -404.
  // @param[in] _bt: Error message.
  public: inline void setBackTrack(std::string _bt) {backtrack_ = _bt;};

  // @brief Get latest error log. (error log set when exception detected)
  // @return Saved error message.
  public: inline std::string getBackTrack() {return backtrack_;};

  // @brief Sets current task priority to highest.
  public: inline void setMaxPriority() {priority_ = 0;};

  // @brief Sets current task priority to normal priority.
  public: inline void resetPriority() {priority_ = 2;};

  // @brief Redo this action.
  public: inline void redoThis() {--curaction_;};

  // @brief Completely abort this task (note: next=finishTask_).
  public: inline void goToEnd()
  {curaction_ = static_cast<int>(actionlist_->size() - 2);};

  // @brief Check whether task queue is over given number.
  // @return True if over given number.
  public: inline bool queueOver(int _num) {return (taskq_.size() >= _num);};

  // @brief Task transition handler (runs until main task reaches finish).
  //     Assumes exception and temp actions to be same for all tasks.
  // @param[in] _entid: ID of main task before any interruption.
  // @param[in] _als: List of actions of all tasks.
  // @param[in] _el: Actions when entered exception. e.g. call person
  // @param[in] _tl: Actions when entered temp escape. e.g. place object
  // @param[in] _from: From which action main task (before interruption) starts.
  public: void runTask
  (int _entid, std::vector<ActionList> _als, ActionList *_el, ActionList *_tl, int _from);

  // @brief Task transition handler (escapes when task changes).
  // @param[in] _entid: ID of current task.
  // @param[in] _al: List of actions.
  // @param[in] _el: Actions when entered exception. e.g. call person
  // @param[in] _tl: Actions when entered temp escape. e.g. place object
  // @param[in] _from: From which action to start task.
  // @return ID of next task to conduct.
  private: int runTask
  (int _entid, ActionList *_al, ActionList *_el, ActionList *_tl, int _from);

  // @brief Use planner as interruption checker without plans.
  public: void noPlanning();

  // @brief Get next task from queue. Stopped task is prioritized.
  // @param[in] _task: Variable to save returned entid.
  // @param[in] _from: Variable to save returned action start id.
  // @return True if queue is not empty, false if empty.
  public: bool getTaskFromQueue(int &_task, int &_from);

  // @brief Common setup in parent and child class for iStart.
  // @param[in] _js: Settings for actions during interaction.
  // @param[in] _ws: Settings for interactiveWaitInterpolation.
  // @param[in] _srv: Variable to save settings.
  // @return false if iStart should escape due to iOff settings, etc.
  protected: bool iStartSetup
  (jumpSettings _js, waitSettings _ws, negomo::WaitInterpolationRequest &_srv);

  // @brief Check and start interruption interaction in background.
  //     Function will handle any interrupting interactions.
  // @param[in] _js: Settings for actions during interaction.
  // @param[in] _ws: Settings for interactiveWaitInterpolation.
  public: virtual void iStart
  (jumpSettings _js=jumpSettings(), waitSettings _ws=waitSettings());

  // @brief Finish interaction in background.
  //     Function will prepare for next task if task was interrupted.
  // @param[in] _usedhands: Variable to save num of used hands after action.
  // @param[in] _nexttask: Variable to save next task index.
  // @return Value of _nexttask. iJoin should be used like: if (iJoin() != -1)
  public: int iJoin(int &_usedhands, int &_nexttask);

  // Below iJoin not recommended. Settings should be set in iStart.
  // @brief Finish interaction in background.
  //     Function will prepare for next task if task was interrupted.
  // @param[in] _usedhands: Variable to save num of used hands after action.
  // @param[in] _nexttask: Variable to save next task index.
  // @param[in] _atEnd: 1 if called near end of action. -1 if in exception.
  //     Must be -1 when in exception.
  //     Expects 1 when num of used hands is not 0.
  // @param[in] _backwardAllowed: True if task is reverse operatable.
  //     0: forward only, -1: forward nor backward allowed
  //     Must be -1 when in exception.
  // @return Value of _nexttask. iJoin should be used like: if (iJoin() != -1)
  public: int iJoin // to be deprecated
  (int &_usedhands, int &_nexttask, int _atEnd, int _backwardAllowed=1);

  // @brief Set error state after action finish.
  // @param[in] _error: Error state of action.
  // @param[in] _backtrack: Error messages.
  public: void setError(bool _error, std::string _backtrack);

  // @brief Pushes move action to tempq_ when escaping from task.
  // @param[in] _nexttask: Entity id of next task.
  private: void pushMoveTemp(int _nexttask);

  // @brief Plan action steps for setting used hands to zero.
  // @param[in] _exception: True if task is at error state.
  // @param[in] _backwardAllowed: 1 if task is reverse operatable.
  // @param[in] _forward: Variable to save whether plan will forward task.
  //     When _forward is false, plan will backward task.
  // @return List of planned actions.
  private: std::vector<ActionFunc> planToNext
  (bool _exception, int _backwardAllowed, bool &_forward);

  // @brief Conduct actions for switching to a different task.
  // @param[in] _nexttask: Planned next task.
  // @param[in] _usedhands: Variable to save num of used hands after action.
  // @param[in] _exception: True if task is at error state.
  // @param[in] _atEnd: 1 if called near end of action. -1 if in exception.
  // @param[in] _backwardAllowed: 1 if task is reverse operatable.
  //     0: forward only, -1: forward nor backward allowed
  // @return True if preparation succeeded, false if failed.
  private: bool prepareForNext
  (int _nexttask, int &_usedhands, bool _exception=false, int _atEnd=0, int _backwardAllowed=1);

  // @brief Get results of interaction interruptions. Used with NegomoBridge2.
  // @return ID of next task to conduct.
  private: virtual int getWaitInterpolation();

  // @brief Returns saved entities.
  // @return Entities of task _id (overwrite possible).
  public: inline boost::property_tree::ptree& getEntities() {return entities_[curentid_];};

  // @brief Temporary pushes entity values for next task.
  //     CreateTask must be called after using nextEntities.
  // @return Temporary entity slot.
  public: boost::property_tree::ptree& nextEntities();

  // @brief Create a new task.
  // @param[in] _taskid: ID of task from capabilities vector.
  // @return ID of new task. -1 if task is a duplicate of existing or queued task.
  public: int createTask(int _taskid);

  // @brief Check whether new task is a duplicate of existing task.
  // @param[in] _refentid: Task id of existing or queued task.
  // @param[in] _taskid: Task id of new task in capabilities vector.
  // @return True if is a duplicate.
  private: bool checkTaskDuplicate(int _refentid, int _taskid);

  // @brief Initiate planner with task entities.
  // @param[in] _entities: List of entities in each task. Size must match tasks.
  public: void init(std::vector<std::vector<std::string> > _entities);

  // @brief Set default queue to planner.
  public: void setDefaultQueue(std::vector<int> _q);

  // @brief Push new tasks to default queue.
  //     Not interrupted and lowest priority. Push without filling task queue.
  public: void appendDefaultQueue(std::vector<int> _q);

  // @brief Used when calling U.I. input from viewer.
  // @param[in] _type: Name of U.I. to trigger.
  // @return Input from user.
  private: std::string waitUserInput(std::string _type);

  // @brief Creates anonymous new person. Used for anonymous non-duplicates.
  // @return A non-duplicate wild ID.
  public: inline std::string newWild() {++wildid_; return std::to_string(wildid_);};

  // @brief Default interaction
  protected: bool shortInteraction(negomo::PlannerInteractionCall::Request &_req,
                                   negomo::PlannerInteractionCall::Response &_res);



  // predefined actions

  // @brief Used for auto-setting default entity values.
  private: int initTask_(int _a, int &_b);

  // @brief Dummy action. Used when no backward action.
  private: int emptyAction_(int _a, int &_b);

  // @brief Used when actions are looped during a task. Declares loop start.
  private: int loopStart_(int _a, int &_b);

  // @brief Used when actions are looped during a task. Declares loop end.
  private: int loopEnd_(int _a, int &_b);

  // @brief Must be used at end of task. Clears all saved entities.
  private: int finishTask_(int _a, int &_b);

  // @brief Used when ending exception from UI.
  private: int uiExceptionHandle_(int _a, int &_b);

  // @brief Dummy task that sleeps (for interaction debug).
  private: int tOff_(int _a, int &_b);

  // @brief Accessable initTask_.
  public: ActionFunc initTask;

  // @brief Set at constructor.
  public: ActionFunc moveToWorkspace;

  // @brief Accessable emptyAction_.
  public: ActionFunc emptyAction;

  // @brief Accessable loopStart_.
  public: ActionFunc loopStart;

  // @brief Accessable loopEnd_.
  public: ActionFunc loopEnd;

  // @brief Accessable finishTask_.
  public: ActionFunc finishTask;

  // @brief Accessable uiExceptionHandle_.
  public: ActionFunc uiExceptionHandle;

  // @brief Accessable tOff_.
  public: ActionFunc tOff;



  // planner variables

  // @brief Used for sending debug msgs and calling iWaitInterpolation.
  private: ros::NodeHandle nh_;

  // @brief Used to start wait interpolation function.
  private: ros::ServiceClient waitinterpolation_client_;

  // @brief Used to get results of wait interpolation function.
  private: ros::ServiceClient waitinterpolation_result_client_;

  // @brief List of actions in task.
  private: ActionListPtr actionlist_;

  // @brief Default task queue. First is taskid, second is actionid.
  private: std::vector<std::pair<int, int> > defaultq_;

  // @brief Interrupted task queue. First is entid, second is actionid.
  protected: std::vector<std::pair<int, int> > taskq_;

  // @brief Tasks escaped with temporary placement. First is entid.
  private: std::vector<std::pair<int, std::stack<ActionFunc>> > tmpq_;

  // @brief ID of current task.
  private: int curentid_;

  // @brief ID of current action.
  private: int curaction_;

  // @brief Priority of current task.
  protected: int priority_;

  // @brief Variable to save error log. Used for getting help during exception.
  private: std::string backtrack_;

  // @brief Tree to hold entities.
  private: std::map<int, boost::property_tree::ptree> entities_;

  // @brief List of entites(first) and its default values(second) in each task.
  private: std::vector<std::vector<std::pair<std::string, std::string>> >
  default_entities_;

  // @brief List of capabilities, used in default interaction.
  protected: std::vector<std::string> capability_names_;

  // @brief An id between 0 ~ (num_capabilities-1) is applied to each task. 
  private: int num_capabilities_;

  // @brief Save any task postponed from exception. Resets in getTaskFromQueue.
  private: int exception_escaped_id_;

  // @brief Used in nextEntities. Resets in createTask.
  private: int created_tmpentity_;

  // @brief Used when joining from wait interpolation.
  private: bool waitinterpolation_call_failed_;

  // @brief Used in newWild().
  private: int wildid_;

  // @brief Saved task and entity lists.
  private: negomo::PlannerDefaultInteractionCall intsrv_;

  // @brief Used for robot interaction in shortInteraction (default interaction).
  private: ros::ServiceClient planner_interaction_client_;

  private: ros::CallbackQueue defint_queue_;

  private: ros::AsyncSpinner defint_spinner_;

  private: ros::ServiceServer defint_interact_server_;

  // @brief Disable entering interaction in iStart, iJoin.
  protected: bool ioff_;

  // @brief Set to true to postpone iStart. e.g. Avoid continuous interactions.
  protected: bool timer_;

  // @brief Time point iStart was postponed.
  protected: std::chrono::high_resolution_clock::time_point timerstart_;

  // @brief How many milliseconds to postpone iStart.
  protected: int timertime_;

  // @brief Parameter used in iJoin but set in iStart.
  //     True if task can be interrupted with any number of used hands.
  private: bool iJoinset_ignore_used_hands_;

  // @brief Parameters passed to iJoin from iStart.
  private: int iJoinset_at_end_; // to be deprecated

  // @brief Parameters passed to iJoin from iStart.
  private: int iJoinset_backward_allowed_; // to be deprecated

  // @brief Record usedhands as new version allows escape with occupied hands.
  private: int usedhands_;

  // @brief iJoin must be called after iStart. Variable check code validity.
  private: bool waitfor_iJoin;



  // node planner variables

  // @brief Passes variables and calls action to user node.
  private: int callActionF
  (int _a, int &_b, bool _backward, ros::ServiceClient *_client, int *_action,
   std::vector<std::pair<std::string, std::vector<std::string>> > *_id2str);

  // @brief Calls moveToWorkspace action in user node.
  private: int callMoveTo_(int _a, int &_b);

  // @brief Passes variables and calls action to user node.
  private: int callAction_(int _a, int &_b);

  // @brief Passes variables and calls action to user node.
  private: int callException_(int _a, int &_b);

  // @brief Passes variables and calls action to user node.
  private: int callTemp_(int _a, int &_b);

  // @brief Passes variables and calls action to user node.
  private: int callRevAction_(int _a, int &_b);

  // @brief Passes variables and calls action to user node.
  private: int callRevException_(int _a, int &_b);

  // @brief Passes variables and calls action to user node.
  private: int callRevTemp_(int _a, int &_b);

  // @brief Accessable callAction_.
  public: ActionFunc callAction;

  // @brief Accessable callAction_.
  public: ActionFunc callException;

  // @brief Accessable callAction_.
  public: ActionFunc callTemp;

  // @brief Accessable callAction_.
  public: ActionFunc callRevAction;

  // @brief Accessable callAction_.
  public: ActionFunc callRevException;

  // @brief Accessable callAction_.
  public: ActionFunc callRevTemp;

  // @brief Accessable callMoveTo_.
  public: ActionFunc callMoveTo;

  // @brief Used as action function.
  private: ros::ServiceClient callaction_client_;

  // @brief Used as action function.
  private: ros::ServiceClient callexception_client_;

  // @brief Used as action function.
  private: ros::ServiceClient calltemp_client_;

  // @brief Action ID to string values.
  public: std::vector<std::pair<std::string, std::vector<std::string>> > id2str_;

  // @brief Exception ID to string values.
  public: std::vector<std::pair<std::string, std::vector<std::string>> > eid2str_;

  // @brief Temp ID to string values.
  public: std::vector<std::pair<std::string, std::vector<std::string>> > tid2str_;

  // @brief Action id number of exception.
  private: int eaction_;

  // @brief Action id number of temp.
  private: int taction_;



  // ui variables

  private: void uiCallback(const std_msgs::String::ConstPtr &_msg);

  private: ros::Publisher uipub_;

  private: ros::CallbackQueue uiqueue_;

  private: ros::AsyncSpinner uispinner_;

  private: ros::Subscriber uisub_;

  private: std::string input_;

  private: std::mutex input_mutex_;



  // debug variables

  // @brief Updates current action in view planner.
  // @param[in] _id: Action id. Exception/temp actions auto-handled in viewer.
  private: inline void vpUpdate(int _id=-1) {
    std_msgs::Int32 msg; msg.data = _id; vppub_update_.publish(msg);
    usleep(10 * 1000); // ROS msg may enter in wrong queue order without sleep
  };

  // @brief Updates current action in view planner for forwarding/backwarding.
  // @param[in] _f: True if forwarding.
  // @param[in] _id: How much forwarded(backwarded).
  private: inline void vpUpdate(bool _f, int _id) {
    std_msgs::Int32 msg;
    msg.data = (_f ? curaction_ + _id + 1 : curaction_ - _id);
    vppub_update_.publish(msg);
    usleep(10 * 1000); // ROS msg may enter in wrong queue order without sleep
  };

  // @brief Push current task to queue in view planner.
  private: inline void vpQueue() {
    std_msgs::Int32 msg; msg.data = curentid_; vppub_toqueue_.publish(msg);
    usleep(10 * 1000); // ROS msg may enter in wrong queue order without sleep
  };

  // @brief Updates plan before switching task in view planner.
  // @param[in] _at: Current action id. (-1=replan from forward)
  // @param[in] _to: How much forward/backward planned.
  private: inline void vpInterrupt(int _at, int _to) {
    negomo::VpConnect msg; msg.at = _at; msg.to = curaction_ + _to;
    vppub_enterinterruption_.publish(msg); 
    usleep(10 * 1000); // ROS msg may enter in wrong queue order without sleep
  }

  // @brief Set enter exception flag in view planner. 
  private: inline void vpException() {
    std_msgs::Empty msg; vppub_enterexception_.publish(msg);
    usleep(10 * 1000); // ROS msg may enter in wrong queue order without sleep
  };

  // @brief Set enter temp flag in view planner.
  private: inline void vpTemp(std::string _type="") {
    std_msgs::String msg; msg.data = _type; vppub_entertemp_.publish(msg);
    usleep(10 * 1000); // ROS msg may enter in wrong queue order without sleep
  };

  protected: bool base_;

  private: ros::Publisher vppub_cleanall_;

  private: ros::Publisher vppub_activate_;

  protected: ros::Publisher vppub_addtoqueue_;

  private: ros::Publisher vppub_deactivate_;

  private: ros::Publisher vppub_update_;

  private: ros::Publisher vppub_toqueue_;

  private: ros::Publisher vppub_enterinterruption_;

  private: ros::Publisher vppub_enterexception_;

  private: ros::Publisher vppub_entertemp_;
  };

  typedef std::shared_ptr<NegomoPlanner> NegomoPlannerPtr;
}

#endif
