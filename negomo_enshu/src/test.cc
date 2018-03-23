#include "negomo.hh"

//////////////////////////////////////////////////
void estimate(negomo::NegomoPtr _nm, std::string _seq, std::string _seqf,
              std::string _q0, std::string _q1, std::string _truth, std::string &_result)
{
  double score, score_from_future;
  std::vector<std::string> states = _nm->Estimate(_nm->hmm, _seq, score, 1, false, "");
  std::vector<std::string> predicted_states =
    _nm->EstimateFromFuture(_nm->hmm, _seq, score_from_future, false, "");
  negomo::weighted_state state = _nm->Score(states, predicted_states, score, score_from_future, 1.0);

  // first, assume robot wants to continue current action
  std::string actionp = "nperceptual0";
  auto const pos = _seq.find_last_of("_");
  if (pos != std::string::npos)
    actionp = _seq.substr(pos + 1);
  std::string actionstr = actionp;
  // next, check whether robot wanted to be proactive
  std::string actionf = _seqf;
  auto const posf = _seqf.find_last_of("_");
  if (posf != std::string::npos)
    actionf = _seqf.substr(posf + 1);
  if (actionf.find("proactive") != std::string::npos)
    actionstr = actionf;

  negomo::labelled_action action = negomo::label::cat(actionstr);
  std::string state_name = state.name; // save as will be overwritten
  float confidence = state.confidence; // save as will be overwritten
  bool conflict = _nm->Conflict(action.label, state); // overwrite state from future estimate
  negomo::labelled_action next_action = _nm->DecideAction(action, state);

  _result += _seq + ":" + actionp + "->" + next_action.name + "(" + std::to_string(confidence) + ")~" + actionf + "," + _q0 + "->" + _q1 + "," + state_name + "~" + _truth + "&&&";
}

//////////////////////////////////////////////////
void negomo::Test(ros::NodeHandle _nh, negomo::parameters _param)
{
  std::string ini_model_name;
  _nh.param<std::string>("ifile", ini_model_name, "initial70.hmm");
  std::string dur_model_name;
  _nh.param<std::string>("dfile", dur_model_name, "during70.hmm");
  negomo::NegomoPtr nmi(new negomo::Negomo(_nh, ini_model_name, _param));
  negomo::NegomoPtr nmd(new negomo::Negomo(_nh, dur_model_name, _param));
  negomo::NegomoPtr nm = nmi;

  // for some reason, comma cannot be found correctly from rosparam string, use @ mark instead
  std::string sequence;
  _nh.param<std::string>("seq", sequence,
      "looktoward_nperceptual0@----@looktoward_reactive1@looktoward_reactive0@lookaway_reactive0@lookaway_reactive0@===@lookaway_reactive1@lookaway_reactive1@lookaway_reactive1@===");
  std::string ground_truth;
  _nh.param<std::string>("truth", ground_truth, "N@----@N@N@N@N@===@N@N@N@===");
  std::string good_or_bad;
  _nh.param<std::string>("rating", good_or_bad, "N/A|N/A@----@0|0@0|0@0|0@0|0@===@0|0@0|0@0|0@===");

  std::vector<std::string> seqs;
  std::vector<std::string> truths;
  std::vector<std::string> qualities;
  size_t pos = 0, last = 0;
  while ((pos = sequence.find("@", last)) && pos != std::string::npos) {
    std::string seq = sequence.substr(last, pos - last);
    last = pos + 1;
    seqs.push_back(seq);
  }
  seqs.push_back(sequence.substr(last, sequence.length()));
  pos = 0; last = 0;
  while ((pos = ground_truth.find("@", last)) && pos != std::string::npos) {
    std::string truth = ground_truth.substr(last, pos - last);
    last = pos + 1;
    truths.push_back(truth);
  }
  truths.push_back(ground_truth.substr(last, ground_truth.length()));
  pos = 0; last = 0;
  while ((pos = good_or_bad.find("@", last)) && pos != std::string::npos) {
    std::string quality = good_or_bad.substr(last, pos - last);
    last = pos + 1;
    qualities.push_back(quality);
  }
  qualities.push_back(good_or_bad.substr(last, good_or_bad.length()));

  std::string result = "";
  std::string mark = "I:";
  if (seqs.size() != truths.size()) {
    result = "seq != truth --error!!!!!!!!";
  } else if (seqs.size() != qualities.size()) {
    result = "seq != rating --error!!!!!!!!";
  } else {
    std::string seqv = "";
    for (unsigned int i = 0; i < seqs.size(); ++i) {
      if (seqs[i] == "----") {
        seqv = seqv.substr(0, seqv.length() - 1);
        result += mark;
        estimate(nm, seqv, seqs.at(i+1), qualities.at(i-1), qualities.at(i+1), truths.at(i-1), result);
        seqv += ",";
      } else if (seqs[i] == "===") {
        int j = std::min(static_cast<int>(i) + 1,
                         static_cast<int>(seqs.size()) - 1);
        seqv = seqv.substr(0, seqv.length() - 1);
        result += mark;
        estimate(nm, seqv, seqs.at(j), qualities.at(i-1), qualities.at(j), truths.at(i-1), result);
        seqv = "";
        if (nm == nmi) { // switch to during
          nm = nmd;
          mark = "D:";
        } else { // switch to initial
          nm = nmi;
          mark = "I:";
        }
      } else if (seqs[i] == "") {
        result += "&&&";
        nm = nmi; // switch to initial
        mark = "I:";
        seqv = "";
      } else {
        seqv += seqs[i] + ",";
      }
    }
  }

  printf(result.c_str());

  seqs.clear();
  truths.clear();
  qualities.clear();
}
